/**
 * Copyright @ 2021 - 2023 JIDU AUTO CO.,LTD.
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are NOT permitted except as agreed by
 * JIDU AUTO CO.,LTD.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#include "control/src/feature/virtual_sensors.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/math/control_math.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/motion_compensator.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::math::Clamp;
using jiduauto::pnc::util::TimeUtility;
using jiduauto::pnc::vehicle::VehicleConfig;
using pnc::chassis::DrivingMode;
using pnc::chassis::GearPosition;
using pnc::common::VehicleState;

bool VirtualSensors::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("VirtualSensors initializing ...");
    virtual_sensors_conf_ = control_conf.control_preprocess_conf().virtual_sensors_conf();
    enable_csv_debug_ = control_conf.enable_csv_debug();
    vehicle_paras_ = VehicleConfig::Instance()->GetParas();
    post_process_conf_ = control_conf.post_process_conf();

    // Setup filters
    is_kf_reset_ = ResetKalmanFilter();
    // digital filter
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    ControlMath::GetLpfCoefficient(control_conf.control_period(), virtual_sensors_conf_.cutoff_frequency(), &den, &num);
    standstill_steer_angle_filter_.SetCoefficients(den, num);

    if (post_process_conf_.enable_terminal_status_reaction() && !LoadSteerTorqueScheduler()) {
        LOG_ERROR("LoadSteerTorqueScheduler failed");
        return false;
    }

    return true;
}

bool VirtualSensors::Reset() { return true; }

const VehicleState& VirtualSensors::GetSenseView() const { return sense_view_; }

void VirtualSensors::Update(const InputStream& input_stream, pnc::control::ControlDebug* const debug) {
    bool curr_backward = ControlMath::CheckBackward(&input_stream.chassis);
    if (curr_backward != last_backward_) {
        last_backward_ = curr_backward;
        Reset();
        LOG_INFO("Check backward! Reset().");
    }

    VehicleState sense_view;
    sense_view.Clear();
    ConstructPositionAndLocSpd(input_stream, &sense_view, debug->diagnosis_debug_v2());
    ConstructEulerAnglesAndRate(input_stream, &sense_view, debug->diagnosis_debug_v2());
    ConstructChassis(input_stream, &sense_view, debug->diagnosis_debug_v2());
    ConstructTrajectory(input_stream, &sense_view, debug->diagnosis_debug_v2());
    ConstructHistoryCmd(input_stream, &sense_view, debug->diagnosis_debug_v2());
    ConstructTimestamp(input_stream, &sense_view, debug->diagnosis_debug_v2());
    ConstructStandstill(input_stream, &sense_view, debug->diagnosis_debug_v2());
    ConstructModeAndGear(input_stream, &sense_view, debug->diagnosis_debug_v2());

    if (post_process_conf_.enable_terminal_status_reaction()) {
        ConstructTerminalSteering(input_stream, debug->diagnosis_debug_v2(), &sense_view);
    }

    sense_view_.CopyFrom(sense_view);
    debug->mutable_virtual_sensors_debug()->mutable_sense_view()->CopyFrom(sense_view);
}

void VirtualSensors::ConstructEulerAnglesAndRate(const InputStream& input_stream, VehicleState* const sense_view,
                                                 const pnc::control::DiagnosisDebugV2 diag_result) {
    if (diag_result.localization_check_code() &
        (0x00000001 << (pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_BIT - 1))) {
        LOG_DEBUG("Localization general check failed, NO angle and rate update.");
        return;
    }

    // Set euler angles
    const auto& pose = input_stream.localization.loc_pose().pose();
    const auto& orientation = pose.orientation();
    jiduauto::pnc::EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
    if (!q.IsValid()) {
        LOG_ERROR("Invalid euler angles.");
        return;
    }

    sense_view->set_roll(q.roll());
    sense_view->set_pitch(q.pitch());
    sense_view->set_yaw(q.yaw());
    sense_view->set_heading(q.yaw());

    const auto& angular_velocity = pose.angular_velocity();
    sense_view->set_angular_velocity(angular_velocity.z());
    sense_view->set_yaw_rate(angular_velocity.z());
    sense_view->set_roll_rate(angular_velocity.x());
    sense_view->set_pitch_rate(angular_velocity.y());

    const auto& speed3d = pose.linear_velocity();
    const double cur_speed = std::hypot(speed3d.x(), speed3d.y());
    static constexpr double kEpsilon = 1e-6;
    double curvature = angular_velocity.z() / cur_speed;
    if (std::abs(cur_speed) < kEpsilon) {
        // low speed
        curvature = 0.0;
    }
    sense_view->set_curvature(curvature);
}

void VirtualSensors::ConstructPositionAndLocSpd(const InputStream& input_stream, VehicleState* const sense_view,
                                                const pnc::control::DiagnosisDebugV2 diag_result) {
    if (diag_result.localization_check_code() &
        (0x00000001 << (pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_BIT - 1))) {
        is_kf_reset_ = ResetKalmanFilter();
        LOG_DEBUG("Localization general check failed, NO position and speed update.");
        return;
    }

    // Set coordinate
    const double timestamp_sec = input_stream.localization.header().timestamp_sec();
    const auto& pose = input_stream.localization.loc_pose().pose();
    const auto& position = pose.position();
    double x_measurement = position.x();
    double y_measurement = position.y();
    double z_measurement = position.z();

    const auto& orientation = pose.orientation();
    jiduauto::pnc::EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());

    const auto& acceleration_ego_3d = pose.linear_acceleration();
    double acc_x = acceleration_ego_3d.x() * std::cos(q.yaw()) + acceleration_ego_3d.y() * std::sin(q.yaw());
    double acc_y = -acceleration_ego_3d.x() * std::sin(q.yaw()) + acceleration_ego_3d.y() * std::cos(q.yaw());
    double acc_z = acceleration_ego_3d.z();

    const auto& speed3d = pose.linear_velocity();

    if (is_kf_reset_) {
        // Update filter config
        Eigen::Matrix<double, 6, 1> x_in;
        x_in << x_measurement, y_measurement, z_measurement, speed3d.x(), speed3d.y(), speed3d.z();

        // update state belief covariance
        Eigen::Matrix<double, 6, 6> P_in = position_filter_.GetStateCovariance();
        position_filter_.SetStateEstimate(x_in, P_in);

        previous_timestamp_sec_ = timestamp_sec;

        is_kf_reset_ = false;
    }

    double current_timestamp_sec = timestamp_sec;
    double delta_t = current_timestamp_sec - previous_timestamp_sec_;
    previous_timestamp_sec_ = current_timestamp_sec;

    Eigen::Matrix<double, 3, 1> u_in;
    u_in << acc_x, acc_y, acc_z;

    // update transition matrix
    Eigen::Matrix<double, 6, 6> F_in = position_filter_.GetTransitionMatrix();
    F_in(0, 3) = delta_t;
    F_in(1, 4) = delta_t;
    F_in(2, 5) = delta_t;
    position_filter_.SetTransitionMatrix(F_in);

    // update control matrix
    Eigen::Matrix<double, 6, 3> B_in = position_filter_.GetControlMatrix();
    B_in(0, 0) = B_in(0, 0) * std::pow(delta_t, 2);
    B_in(1, 1) = B_in(1, 0) * std::pow(delta_t, 2);
    B_in(2, 2) = B_in(2, 0) * std::pow(delta_t, 2);
    position_filter_.SetControlMatrix(B_in);

    // Update position
    position_filter_.Predict(u_in);

    Eigen::Matrix<double, 6, 1> z_in;
    z_in << x_measurement, y_measurement, z_measurement, speed3d.x(), speed3d.y(), speed3d.z();

    position_filter_.Correct(z_in);

    Eigen::Matrix<double, 6, 1> x_out = position_filter_.GetStateEstimate();

    sense_view->set_x(x_out(0, 0));
    sense_view->set_y(x_out(1, 0));
    sense_view->set_z(x_out(2, 0));

    sense_view->mutable_velocity_component_enu()->set_x(x_out(3, 0));
    sense_view->mutable_velocity_component_enu()->set_y(x_out(4, 0));
    sense_view->mutable_velocity_component_enu()->set_z(x_out(5, 0));

    if (virtual_sensors_conf_.enable_kf_loc_speed()) {
        sense_view->set_localization_velocity(std::hypot(x_out(3, 0), x_out(4, 0)));
    } else {
        sense_view->set_localization_velocity(std::hypot(speed3d.x(), speed3d.y()));
    }

    LOG_DEBUG("Virtual Sensors:raw state:x:%.4f | y:%.4f | z:%.4f", position.x(), position.y(), position.z());
    LOG_DEBUG("Virtual Sensors:est state:x:%.4f | y:%.4f | z:%.4f", x_out(0, 0), x_out(1, 0), x_out(2, 0));
}

void VirtualSensors::ConstructChassis(const InputStream& input_stream, VehicleState* const sense_view,
                                      const pnc::control::DiagnosisDebugV2 diag_result) {
    if (diag_result.chassis_check_code() & (0x00000001 << (pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT - 1))) {
        LOG_DEBUG("Chassis general check failed, NO chassis update.");
        return;
    }

    const auto& chassis = input_stream.chassis;

    const double linear_acceleration_with_compensation =
        jiduauto::pnc::vehicle::MotionCompensator::CorretAccelerationBySlope(chassis, input_stream.localization);

    sense_view->set_linear_acceleration_with_compensation(linear_acceleration_with_compensation);
    sense_view->set_linear_velocity(chassis.speed_mps());
    sense_view->set_wheelspd_resultant_velocity(chassis.average_wheel_speed());
    sense_view->set_linear_acceleration(chassis.chassis_lon_acc());
    sense_view->set_steering_percentage(chassis.steering_percentage());
    sense_view->set_steering_angle(chassis.steering_percentage() *
                                   virtual_sensors_conf_.revise_percentage_max_steer_angle_deg() / 100.0);
    sense_view->set_steering_angle_rate(chassis.steering_angle_spd());
    sense_view->set_steering_torque(chassis.steering_torque_nm());
    sense_view->set_motor_torque(chassis.motor_info().motor_torque_nm());
    sense_view->set_motor_speed(chassis.motor_info().motor_speed());
    sense_view->set_parking_brake(chassis.parking_brake());
    sense_view->set_chassis_ready_st(chassis.chassis_ready_st());
}

void VirtualSensors::ConstructTrajectory(const InputStream& input_stream, VehicleState* const sense_view,
                                         const pnc::control::DiagnosisDebugV2 diag_result) {
    if (diag_result.trajectory_check_code() &
        (0x00000001 << (pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_BIT - 1))) {
        LOG_DEBUG("Trajectory general check failed, NO planning update.");
        return;
    }

    const auto& trajectory = input_stream.planning_info;

    sense_view->mutable_trajectory()->CopyFrom(trajectory);
}

void VirtualSensors::ConstructHistoryCmd(const InputStream& input_stream, VehicleState* const sense_view,
                                         const pnc::control::DiagnosisDebugV2 diag_result) {
    const auto& cmd = input_stream.control_command;

    sense_view->mutable_history_cmd()->CopyFrom(cmd);
    sense_view->set_last_acceleration_cmd(cmd.acceleration());
    sense_view->set_last_steering_target(cmd.steering_target());
}

void VirtualSensors::ConstructTimestamp(const InputStream& input_stream, VehicleState* const sense_view,
                                        const pnc::control::DiagnosisDebugV2 diag_result) {
    const auto& localization = input_stream.localization;
    double loc_measure_time = localization.loc_pose().measurement_time();
    double loc_header_time = localization.header().timestamp_sec();

    const auto& chassis = input_stream.chassis;
    double chas_header_time = chassis.header().timestamp_sec();

    double cur_proc_time = TimeUtility::GetCurrentTimesecond();

    switch (virtual_sensors_conf_.time_source()) {
    case pnc::control::VirtualSensorsConf::CUR_PROC_TIME:
        sense_view->set_timestamp_sec(cur_proc_time);
        break;
    case pnc::control::VirtualSensorsConf::CHAS_HEADER_TIME:
        sense_view->set_timestamp_sec(chas_header_time);
        break;
    case pnc::control::VirtualSensorsConf::LOC_MEASURE_TIME:
        sense_view->set_timestamp_sec(loc_measure_time);
        break;
    case pnc::control::VirtualSensorsConf::LOC_HEADER_TIME:
    default:
        sense_view->set_timestamp_sec(loc_header_time);
        break;
    }
}

void VirtualSensors::ConstructStandstill(const InputStream& input_stream, VehicleState* const sense_view,
                                         const pnc::control::DiagnosisDebugV2 diag_result) {
    if ((diag_result.chassis_check_code() & (0x00000001 << (pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT - 1))) ||
        (diag_result.localization_check_code() &
         (0x00000001 << (pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_BIT - 1)))) {
        LOG_DEBUG("Localization or chassis general check failed, NO standstill update.");
        return;
    }

    const bool chassis_standstill = input_stream.chassis.standstill();

    const auto& speed3d = input_stream.localization.loc_pose().pose().linear_velocity();
    double localization_velocity = std::hypot(speed3d.x(), speed3d.y());
    double linear_velocity = input_stream.chassis.speed_mps();
    const bool internal_standstill = IsStandStill(localization_velocity, linear_velocity);

    if (virtual_sensors_conf_.enable_chassis_standstill()) {
        sense_view->set_standstill(chassis_standstill);
    } else {
        sense_view->set_standstill(internal_standstill);
    }

    const double last_steering_cmd_deg = input_stream.control_command.steering_target();
    const double steering_angle = input_stream.chassis.steering_percentage() *
                                  virtual_sensors_conf_.revise_percentage_max_steer_angle_deg() / 100.0;
    bool require_standstill_steering = false;
    double standingstill_swa = 0.0;
    double mean_effect_kappa_debug = 0.0;
    if (input_stream.planning_info.trajectory_point_size() != 0) {
        require_standstill_steering =
            IsRequireStandStillSteering(steering_angle, last_steering_cmd_deg, sense_view->standstill(),
                                        input_stream.planning_info, &standingstill_swa, &mean_effect_kappa_debug);
    }

    if (virtual_sensors_conf_.enable_standstill_swa_filter()) {
        standingstill_swa = standstill_steer_angle_filter_.Filter(standingstill_swa);
        if (std::isnan(standingstill_swa)) {
            standingstill_swa = 0.0;
        }
    }

    sense_view->set_require_standstill_steering(require_standstill_steering);
    sense_view->set_standstill_swa(standingstill_swa);
    sense_view->set_mean_effect_standstill_kappa(mean_effect_kappa_debug);
}

void VirtualSensors::ConstructModeAndGear(const InputStream& input_stream, VehicleState* const sense_view,
                                          const pnc::control::DiagnosisDebugV2 diag_result) {
    if (diag_result.chassis_check_code() & (0x00000001 << (pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT - 1))) {
        LOG_DEBUG("Chassis general check failed, NO mode and gear update.");
        return;
    }

    sense_view->set_driving_mode(input_stream.chassis.driving_mode());
    sense_view->set_gear(input_stream.chassis.gear_location());
}

bool VirtualSensors::ResetKalmanFilter() {
    // Initial state
    Eigen::Matrix<double, 6, 1> x;
    x.setZero();

    Eigen::Matrix<double, 6, 1> u;
    u.setZero();

    // Initial state belief covariance
    Eigen::Matrix<double, 6, 6> P;
    P.setZero();
    P(0, 0) = 1.0;
    P(1, 1) = 1.0;
    P(2, 2) = 1.0;
    P(3, 3) = 100.0;
    P(4, 4) = 100.0;
    P(5, 5) = 100.0;

    // Transition matrix
    Eigen::Matrix<double, 6, 6> F;
    F.setIdentity();

    // Transition noise covariance
    Eigen::Matrix<double, 6, 6> Q;
    Q.setIdentity();

    // Observation matrix
    Eigen::Matrix<double, 6, 6> H;
    H.setIdentity();

    // Observation noise covariance
    Eigen::Matrix<double, 6, 6> R;
    R.setZero();
    if (virtual_sensors_conf_.kf_conf().noise_cov_size() == R.rows()) {
        R(0, 0) = virtual_sensors_conf_.kf_conf().noise_cov(0);
        R(1, 1) = virtual_sensors_conf_.kf_conf().noise_cov(1);
        R(2, 2) = virtual_sensors_conf_.kf_conf().noise_cov(2);
        R(3, 3) = virtual_sensors_conf_.kf_conf().noise_cov(3);
        R(4, 4) = virtual_sensors_conf_.kf_conf().noise_cov(4);
        R(5, 5) = virtual_sensors_conf_.kf_conf().noise_cov(5);
    } else {
        LOG_WARN("Wrong kf covariance config");
    }

    // Control matrix
    Eigen::Matrix<double, 6, 3> B;
    B.setZero();
    B(0, 0) = 0.5;
    B(1, 1) = 0.5;
    B(2, 2) = 0.5;
    B(3, 0) = 1.0;
    B(4, 1) = 1.0;
    B(5, 2) = 1.0;

    position_filter_.SetStateEstimate(x, P);
    position_filter_.SetTransitionMatrix(F);
    position_filter_.SetTransitionNoise(Q);
    position_filter_.SetObservationMatrix(H);
    position_filter_.SetObservationNoise(R);
    position_filter_.SetControlMatrix(B);

    return true;
}

const bool VirtualSensors::IsStandStill(double localization_velocity, double chassis_velocity) {
    bool is_standstill = false;
    const double kStandStillThreshold = 0.1;
    if (std::abs(localization_velocity) < kStandStillThreshold && std::abs(chassis_velocity) < kStandStillThreshold) {
        is_standstill = true;
    }
    standstill_list_.push_back(is_standstill);
    const int32_t kStandstillJudgeLength{5};
    if (standstill_list_.size() > kStandstillJudgeLength) {
        standstill_list_.pop_front();
    }
    if (accumulate(standstill_list_.begin(), standstill_list_.end(), 0) == kStandstillJudgeLength) {
        return true;
    } else {
        return false;
    }
}

const bool VirtualSensors::IsRequireStandStillSteering(double current_steering_angle, double last_cmd, bool standstill,
                                                       const pnc::planning::ADCTrajectory& planning_info,
                                                       double* standstill_swa, double* mean_effect_kappa_out) {
    bool require_standstill_steering = false;

    int32_t current_trajectory_point_size{planning_info.trajectory_point_size()};
    pnc::common::TrajectoryPoint last_point = planning_info.trajectory_point(current_trajectory_point_size - 1);
    const double kMinActiveTrajectoryLength = 0.25;  // [m]
    double mean_effect_kappa = 0.0;
    double kappa_steering_angle = last_cmd;
    bool disable_by_pause =
        (planning_info.planning_to_control() == pnc::planning::ADCTrajectory_PlanningToControl_PAUSE);

    if (last_point.path_point().s() > kMinActiveTrajectoryLength) {
        int32_t kPreviewPathPointLength{virtual_sensors_conf_.standstill_preview_index()};
        double sum_path_kappa_for_steering = 0.0;
        for (int32_t i = 0; i < kPreviewPathPointLength; i++) {
            sum_path_kappa_for_steering =
                sum_path_kappa_for_steering + planning_info.trajectory_point(i).path_point().kappa();
        }
        mean_effect_kappa = sum_path_kappa_for_steering / static_cast<double>(kPreviewPathPointLength);

        // todo(zhaobolin): risk max_steer_angle is not the true vehicle steering angle limit, and not aligen with
        // percentage
        kappa_steering_angle = Clamp(
            std::atan(vehicle_paras_.wheel_base() * mean_effect_kappa) / M_PI * 180.0 * vehicle_paras_.steer_ratio(),
            -1.0 * vehicle_paras_.max_steer_angle() / M_PI * 180.0, vehicle_paras_.max_steer_angle() / M_PI * 180.0);
    }

    // reach flag judgement
    const double kReachTargetSWADiffThrethod = 30.0;  // [deg]
    if ((std::abs(kappa_steering_angle - current_steering_angle) < kReachTargetSWADiffThrethod)) {
        reach_flag_list_.push_back(true);
    } else {
        reach_flag_list_.push_back(false);
    }

    if (reach_flag_list_.size() > virtual_sensors_conf_.swa_reach_judge_size()) {
        reach_flag_list_.pop_front();
    }
    bool standstill_swa_reach_flag = false;
    if (accumulate(reach_flag_list_.begin(), reach_flag_list_.end(), 0) ==
        virtual_sensors_conf_.swa_reach_judge_size()) {
        standstill_swa_reach_flag = true;
    }

    // hold for first switch trigger
    if (last_standstill_swa_reach_flag_ == false && standstill_swa_reach_flag == true) {
        first_reach_hold_cout_ = virtual_sensors_conf_.swa_reach_hold_size();
    }
    first_reach_hold_cout_ = std::max(first_reach_hold_cout_ - 1, 0);  // anti-re jump in

    last_standstill_swa_reach_flag_ = standstill_swa_reach_flag;
    if (standstill && !standstill_swa_reach_flag && first_reach_hold_cout_ == 0 && !disable_by_pause) {
        require_standstill_steering = true;
    }

    *mean_effect_kappa_out = mean_effect_kappa;
    *standstill_swa = kappa_steering_angle;
    return require_standstill_steering;
}

void VirtualSensors::ConstructTerminalSteering(const InputStream& input_stream,
                                               const pnc::control::DiagnosisDebugV2 diag_result,
                                               pnc::common::VehicleState* const sense_view) {
    if (diag_result.chassis_check_code() & (0x00000001 << (pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT - 1))) {
        LOG_DEBUG("Chassis general check failed, NO chassis update.");
        return;
    }
    if (diag_result.trajectory_check_code() &
        (0x00000001 << (pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_BIT - 1))) {
        LOG_DEBUG("Trajectory general check failed, NO planning update.");
        return;
    }

    const auto& chassis = input_stream.chassis;
    const auto& trajectory = input_stream.planning_info;

    double cur_steer_torque = chassis.steering_torque_nm();
    double cur_steer_angle =
        chassis.steering_percentage() * virtual_sensors_conf_.revise_percentage_max_steer_angle_deg() / 100.0;
    double terminal_steering_angle{0.0};
    bool bflag_terminal_swa_hold{false};
    bool require_terminal_steering{false};

    const int32_t kSteerTorqueVecSize = post_process_conf_.terminal_process_steer_torque_vector_size();
    const int32_t kSteerAngleListSize = post_process_conf_.terminal_process_steady_steer_angle_list_size();
    const double kSteadySteerThr = post_process_conf_.terminal_process_steady_steer_angle_deg_max();
    const int32_t kTermSwaHoldSize = post_process_conf_.terminal_process_swa_hold_list_size();
    if (!terminal_steer_process_params_.bflag_steady_steer_torque) {
        terminal_steer_process_params_.steer_torque_vec.emplace_back(cur_steer_torque);
        terminal_steer_process_params_.steady_steer_angle_list.push_back(std::fabs(cur_steer_angle) < kSteadySteerThr);
        if (static_cast<int32_t>(terminal_steer_process_params_.steer_torque_vec.size()) > kSteerTorqueVecSize) {
            terminal_steer_process_params_.steer_torque_vec.erase(
                terminal_steer_process_params_.steer_torque_vec.begin());
        }
        if (static_cast<int32_t>(terminal_steer_process_params_.steady_steer_angle_list.size()) > kSteerAngleListSize) {
            terminal_steer_process_params_.steady_steer_angle_list.pop_front();
        }
    }
    if (trajectory.has_terminal_status() && trajectory.terminal_status() &&
        post_process_conf_.enable_terminal_status_reaction()) {
        const double kSteerTorqueDeadZone = post_process_conf_.terminal_process_steer_torque_mean_dz();
        const double kSteerTorqueStdThr = post_process_conf_.terminal_process_steer_torque_std_max();

        require_terminal_steering = true;

        double steering_torque_mean{0.0};
        double steering_torque_stddev{0.0};
        if (!terminal_steer_process_params_.bflag_steady_steer_torque) {
            if (static_cast<int32_t>(terminal_steer_process_params_.steer_torque_vec.size()) == kSteerTorqueVecSize) {
                auto mean_std_pair = pnc::math::CalMeanAndStddev(terminal_steer_process_params_.steer_torque_vec);
                steering_torque_mean = mean_std_pair.first;
                steering_torque_stddev = mean_std_pair.second;
            }
            bool is_steer_angle_steady =
                std::accumulate(terminal_steer_process_params_.steady_steer_angle_list.begin(),
                                terminal_steer_process_params_.steady_steer_angle_list.end(), 0) == kSteerAngleListSize;
            if (std::fabs(steering_torque_mean) > kSteerTorqueDeadZone && steering_torque_stddev < kSteerTorqueStdThr &&
                is_steer_angle_steady) {
                terminal_steer_process_params_.bflag_steady_steer_torque = true;
                terminal_steer_process_params_.steer_torque_mean = steering_torque_mean;
            }
            terminal_steering_angle = 0.0;
        } else {
            bflag_terminal_swa_hold = true;
            terminal_steering_angle = -1.0 * (terminal_steer_process_params_.steer_torque_mean) *
                                      terminal_steer_process_params_.terminal_steer_interpolation->LinearInterpolate(
                                          terminal_steer_process_params_.steer_torque_mean);
        }
    }
    terminal_steer_process_params_.terminal_swa_hold_list.push_back(bflag_terminal_swa_hold);
    if (terminal_steer_process_params_.terminal_swa_hold_list.size() > kTermSwaHoldSize) {
        terminal_steer_process_params_.terminal_swa_hold_list.pop_front();
    }
    if (std::accumulate(terminal_steer_process_params_.terminal_swa_hold_list.begin(),
                        terminal_steer_process_params_.terminal_swa_hold_list.end(), 0) == kTermSwaHoldSize) {
        terminal_steering_angle = 0.0;
    }
    sense_view->set_control_terminal_swa(terminal_steering_angle);
    sense_view->set_require_terminal_process(require_terminal_steering);
}

bool VirtualSensors::LoadSteerTorqueScheduler() {
    const auto& steer_ratio_scheduler = post_process_conf_.steer_ratio_scheduler();
    Interpolation1D::DataType xy1;
    for (const auto& scheduler : steer_ratio_scheduler.scheduler()) {
        xy1.push_back(std::make_pair(scheduler.steering_torque(), scheduler.ratio()));
    }
    if (!terminal_steer_process_params_.terminal_steer_interpolation->Init(xy1)) {
        LOG_ERROR("Fail to load steer ratio scheduler for terminal steer process");
        return false;
    }
    return true;
}

}  // namespace control
}  // namespace jiduauto
