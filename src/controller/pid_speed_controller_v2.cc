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

#include "control/src/controller/pid_speed_controller_v2.h"

#include <algorithm>
#include <cstdint>
#include <utility>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/math/control_math.h"
#include "control/src/math/trajectory_analyzer.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

using pnc::EulerAnglesZXYf;
using pnc::chassis::Chassis;
using pnc::common::VehicleState;
using pnc::control::ControlCommand;
using pnc::control::ControlDebug;
using pnc::control::LongitudinalControlContext;
using pnc::control::PidSpeedDebug;
using pnc::interpolation::Interpolation1D;
using pnc::localization::LocalizationEstimate;
using pnc::math::Clamp;
using pnc::math::NormalizeAngle;
using pnc::planning::ADCTrajectory;
using pnc::util::TimeUtility;
using pnc::vehicle::VehicleConfig;

constexpr double GRA_ACC = 9.8;

PidSpeedControllerV2::PidSpeedControllerV2() { LOG_INFO("%s used.", Name().c_str()); }

PidSpeedControllerV2::~PidSpeedControllerV2() { Stop(); }

bool PidSpeedControllerV2::Init(const pnc::control::ControlConfig& conf) {
    LOG_INFO("%s initializing...", Name().c_str());
    vehicle_params_.CopyFrom(VehicleConfig::Instance()->GetParas());
    control_conf_.CopyFrom(conf);
    const pnc::control::PidSpeedControllerConf& pid_controller_conf = control_conf_.pid_speed_controller_conf();
    double ts = control_conf_.control_period();

    if (!PidConfigCheck()) {
        LOG_ERROR("pid_lon_controller PidConfigCheck() failed! Please check config file.");
        return false;
    }

    // setup controllers
    station_pid_controller_.Init(pid_controller_conf.forward_paras(0).station_pid());
    speed_pid_controller_.Init(pid_controller_conf.forward_paras(0).speed_pid());

    if (pid_controller_conf.enable_leadlag_compensation()) {
        station_leadlag_controller_.Init(pid_controller_conf.station_leadlag_conf(), ts);
        speed_leadlag_controller_.Init(pid_controller_conf.speed_leadlag_conf(), ts);
    }

    // load pid calibration table
    if (!LoadPIDCalibrationTable(pid_controller_conf)) {
        LOG_ERROR("PID calibration table is empty or has wrong size! Controllor Init failed!");
        return false;
    }

    is_controller_initialized_ = true;

    return true;
}

bool PidSpeedControllerV2::ControlMain(const ControlInputStream& control_input_stream,
                                       ControlCommandStream* const control_command_stream) {
    int64_t start_time = TimeUtility::GetCurrentTimeMillsecond();

    if (control_command_stream == nullptr || !is_controller_initialized_) {
        LOG_ERROR("PidSpeedControllerV2 initialize failed or input control_command_stream is nullptr.");
        return false;
    }

    const pnc::control::PidSpeedControllerConf& pid_controller_conf = control_conf_.pid_speed_controller_conf();

    if (is_current_backward_ != ControlMath::CheckBackward(&control_input_stream.input_stream.chassis)) {
        is_current_backward_ = (is_current_backward_ == false) ? true : false;
        Reset();
    }

    LongitudinalControlContext control_context;
    control_context.Clear();
    control_context.set_timestamp_sec(TimeUtility::GetCurrentTimesecond());

    // set control params
    UpdatePIDInterpolatedParams(pid_controller_conf, control_input_stream.vehicle_state.linear_velocity(),
                                &control_context);

    // produce longitudinal control command
    if (!ComputeLongitudinalCommand(&control_context, control_input_stream.input_stream.planning_info,
                                    control_input_stream.vehicle_state, pid_controller_conf)) {
        LOG_ERROR("Failed to compute longitudinal control command!");
        return false;
    }

    // generate pedal command
    GeneratePedalPercentage(&control_context);

    double accel_to_exec = control_context.accel_cmd();
    double throttle_to_exec = control_context.throttle_cmd();
    double brake_to_exec = control_context.brake_cmd();

    // set command to execute
    control_command_stream->control_command.set_acceleration(accel_to_exec);
    control_command_stream->control_command.set_throttle(throttle_to_exec);
    control_command_stream->control_command.set_brake(brake_to_exec);

    control_command_stream->control_command.mutable_debug()->mutable_urban_lon_debug()->set_throttle_control(
        control_command_stream->control_command.throttle());
    control_command_stream->control_command.mutable_debug()->mutable_urban_lon_debug()->set_brake_control(
        control_command_stream->control_command.brake());
    control_command_stream->control_command.mutable_debug()->mutable_urban_lon_debug()->set_speed_error(
        control_context.speed_error());
    control_command_stream->control_command.mutable_debug()->mutable_urban_lon_debug()->set_station_error(
        control_context.station_error());

    int32_t total_time = static_cast<int32_t>(TimeUtility::GetCurrentTimeMillsecond() - start_time);

    // output debug info
    SummaryDebugInfo(total_time, control_context, &control_command_stream->control_debug);

    return true;
}

bool PidSpeedControllerV2::Control(const LocalizationEstimate* const localization, const Chassis* const chassis,
                                   const ADCTrajectory* const trajectory, ControlCommand* const cmd) {
    return true;
}

bool PidSpeedControllerV2::Reset() {
    station_pid_controller_.Reset();
    speed_pid_controller_.Reset();
    station_leadlag_controller_.Reset();
    speed_leadlag_controller_.Reset();

    if (!LoadPIDCalibrationTable(control_conf_.pid_speed_controller_conf())) {
        LOG_ERROR("Unsucceed to load PID calibration table ! Reset() failed!");
        return false;
    }
    LOG_INFO("Reset()");
    return true;
}

std::string PidSpeedControllerV2::Name() const { return "PidSpeedControllerV2"; }

void PidSpeedControllerV2::Stop() {
    // reserved
}

bool PidSpeedControllerV2::PidConfigCheck() const {
    return control_conf_.pid_speed_controller_conf().backward_paras_size() >= 1 &&
           control_conf_.pid_speed_controller_conf().forward_paras_size() >= 1;
}

bool PidSpeedControllerV2::ComputeLongitudinalCommand(LongitudinalControlContext* const context,
                                                      const ADCTrajectory& trajectory,
                                                      const pnc::common::VehicleState& vehicle_state,
                                                      const pnc::control::PidSpeedControllerConf& pid_conf) {
    LongitudinalControlContext* pid_control_context = context;
    double ts = control_conf_.control_period();

    if (!ComputeLongitudinalError(pid_control_context, trajectory, vehicle_state)) {
        LOG_ERROR("Failed to compute longitudinal errors.");
        return false;
    }

    if (!ComputeClosedLoopAccel(pid_control_context, pid_conf, ts)) {
        LOG_ERROR("Failed to compute PID closed-loop command.");
        return false;
    }

    // add slope compensation
    double accel_cmd = pid_control_context->accel_closedloop() +
                       pid_conf.slope_compensation_switch() * SlopeOffsetCompensation(vehicle_state);

    // near-standstill case, turn into open-loop control and replace the computed command with a certain deceleration.
    if (isNearStandstill(pid_control_context, trajectory, pid_conf)) {
        accel_cmd = GetOpenLoopDecel(pid_control_context);
        pid_control_context->set_is_near_stop(true);
        LOG_INFO("Near stop case, replace closed-loop accel with open-loop cmd, accel = %.4f.", accel_cmd);
    }

    pid_control_context->set_accel_cmd(accel_cmd);

    return true;
}

bool PidSpeedControllerV2::ComputeLongitudinalError(LongitudinalControlContext* const context,
                                                    const ADCTrajectory& trajectory,
                                                    const pnc::common::VehicleState& vehicle_state) {
    if (context == nullptr || !trajectory.has_header() || !vehicle_state.has_timestamp_sec()) {
        LOG_ERROR("ComputeLongitudinalErrors() input is nullptr.");
        return false;
    }

    LongitudinalControlContext* pid_control_context = context;

    std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer = std::make_unique<TrajectoryAnalyzer>(&trajectory);

    // find matched point on trajectory
    pid_control_context->mutable_matched_point()->CopyFrom(
        trajectory_analyzer->QueryMatchedPathPoint(vehicle_state.x(), vehicle_state.y()));

    double current_station = 0.0;
    double current_lateral_dis = 0.0;
    double frenet_lon_speed = 0.0;
    double frenet_lat_speed = 0.0;

    trajectory_analyzer->ToTrajectoryFrame(vehicle_state.x(), vehicle_state.y(), vehicle_state.heading(),
                                           vehicle_state.linear_velocity(), pid_control_context->matched_point(),
                                           &current_station, &frenet_lon_speed, &current_lateral_dis,
                                           &frenet_lat_speed);

    pid_control_context->set_current_station(current_station);
    pid_control_context->set_frenet_lon_speed(frenet_lon_speed);
    pid_control_context->set_current_lateral_dis(current_lateral_dis);
    pid_control_context->set_frenet_lat_speed(frenet_lat_speed);

    // find reference & preview point on trajectory
    double absolute_time = TimeUtility::GetCurrentTimesecond();
    double absolute_preview_time = absolute_time + pid_control_context->pid_params().forsee_time();

    pid_control_context->mutable_target_point()->CopyFrom(
        trajectory_analyzer->QueryNearestPointByAbsoluteTime(absolute_time));
    pid_control_context->mutable_preview_point()->CopyFrom(
        trajectory_analyzer->QueryNearestPointByAbsoluteTime(absolute_preview_time));

    // compute station & speed & acceleration error
    pid_control_context->set_heading_error(
        NormalizeAngle(vehicle_state.heading() - pid_control_context->matched_point().theta()));
    pid_control_context->set_current_speed(vehicle_state.linear_velocity() *
                                           std::cos(pid_control_context->heading_error()));
    pid_control_context->set_current_accel(vehicle_state.linear_acceleration() *
                                           std::cos(pid_control_context->heading_error()));

    double one_minus_kappa_lat_error = 1 - pid_control_context->target_point().path_point().kappa() *
                                               vehicle_state.linear_velocity() *
                                               std::sin(pid_control_context->heading_error());
    pid_control_context->set_frenet_lon_accel(pid_control_context->current_accel() / one_minus_kappa_lat_error);
    pid_control_context->set_frenet_lat_accel(vehicle_state.linear_acceleration() *
                                              std::sin(pid_control_context->heading_error()));

    pid_control_context->set_station_target(pid_control_context->target_point().path_point().s());
    pid_control_context->set_station_error(pid_control_context->station_target() -
                                           pid_control_context->current_station());
    pid_control_context->set_speed_target(pid_control_context->target_point().v());
    pid_control_context->set_speed_error(pid_control_context->speed_target() - pid_control_context->frenet_lon_speed());
    pid_control_context->set_accel_target(pid_control_context->target_point().a());
    pid_control_context->set_accel_error(pid_control_context->accel_target() - pid_control_context->frenet_lon_accel());

    pid_control_context->set_preview_station_target(pid_control_context->preview_point().path_point().s());
    pid_control_context->set_preview_station_error(pid_control_context->preview_station_target() -
                                                   pid_control_context->current_station());
    pid_control_context->set_preview_speed_target(pid_control_context->preview_point().v());
    pid_control_context->set_preview_speed_error(pid_control_context->preview_speed_target() -
                                                 pid_control_context->frenet_lon_speed());
    pid_control_context->set_preview_accel_target(pid_control_context->preview_point().a());
    pid_control_context->set_preview_accel_error(pid_control_context->preview_accel_target() -
                                                 pid_control_context->frenet_lon_accel());
    return true;
}

bool PidSpeedControllerV2::ComputeClosedLoopAccel(LongitudinalControlContext* const context,
                                                  const pnc::control::PidSpeedControllerConf& pid_conf,
                                                  const double ts) {
    if (context == nullptr) {
        LOG_ERROR("ComputeClosedLoopAccel() input context is nullptr");
        return false;
    }
    LongitudinalControlContext* pid_control_context = context;

    // station loop control
    double station_error_limited = pid_control_context->station_error();
    if (pid_conf.enable_control_preview()) {
        station_error_limited = pid_control_context->preview_station_error();
    }
    pid_control_context->set_station_loop_input(
        Clamp(station_error_limited, -pid_conf.station_error_limit(), pid_conf.station_error_limit()));

    double station_loop_output = station_pid_controller_.Control(pid_control_context->station_loop_input(), ts);
    if (pid_conf.enable_leadlag_compensation()) {
        station_loop_output = station_leadlag_controller_.Control(station_loop_output, ts);
    }
    pid_control_context->set_station_loop_output(station_loop_output);

    // speed loop control
    double speed_error_limited = pid_control_context->station_loop_output() + pid_control_context->speed_error();
    if (pid_conf.enable_control_preview()) {
        speed_error_limited = pid_control_context->station_loop_output() + pid_control_context->preview_speed_error();
    }
    pid_control_context->set_speed_loop_input(
        Clamp(speed_error_limited, -pid_conf.speed_error_limit(), pid_conf.speed_error_limit()));

    double speed_loop_output = speed_pid_controller_.Control(pid_control_context->speed_loop_input(), ts);
    if (pid_conf.enable_leadlag_compensation()) {
        speed_loop_output = speed_leadlag_controller_.Control(speed_loop_output, ts);
    }
    pid_control_context->set_speed_loop_output(speed_loop_output);

    // acceleration compensation
    double control_value = pid_control_context->speed_loop_output() + pid_control_context->accel_error();
    if (pid_conf.enable_control_preview()) {
        control_value = pid_control_context->speed_loop_output() + pid_control_context->preview_accel_error();
    }
    pid_control_context->set_accel_closedloop(control_value);
    pid_control_context->set_is_near_stop(false);

    return true;
}

bool PidSpeedControllerV2::GeneratePedalPercentage(LongitudinalControlContext* const context) {
    double accel_cmd = context->accel_cmd();
    double throttle_cmd = 0.0;
    double brake_cmd = 0.0;

    if (accel_cmd > 0.0) {
        double rough_throttle = ControlMath::GetAccelerationThrottle(
            accel_cmd, vehicle_params_.acceleration_at_min(), vehicle_params_.acceleration_per_001_throttle(),
            vehicle_params_.acceleration_throttle_max(), vehicle_params_.acceleration_throttle_min());

        throttle_cmd = std::min(rough_throttle, vehicle_params_.acceleration_throttle_max());
        brake_cmd = 0.0;
    } else {
        double rough_brake = ControlMath::GetDecelerationBrake(
            accel_cmd, vehicle_params_.deceleration_at_min(), vehicle_params_.deceleration_per_001_brake(),
            vehicle_params_.deceleration_brake_max(), vehicle_params_.deceleration_brake_min());

        brake_cmd = std::min(rough_brake, vehicle_params_.deceleration_brake_max());
        throttle_cmd = 0.0;
    }

    context->set_throttle_cmd(throttle_cmd);
    context->set_brake_cmd(brake_cmd);

    return true;
}

bool PidSpeedControllerV2::isNearStandstill(LongitudinalControlContext* const context, const ADCTrajectory& trajectory,
                                            const pnc::control::PidSpeedControllerConf& pid_conf) {
    // get path remain by extract a standstill point
    LongitudinalControlContext* pid_control_context = context;

    int32_t standstill_index = 0;
    const double speed_threshold = vehicle_params_.max_abs_speed_when_stopped();
    const double accel_threshold = pid_conf.full_stop_accel();

    while (standstill_index < trajectory.trajectory_point_size()) {
        const auto& current_point = trajectory.trajectory_point(standstill_index);
        if (current_point.v() < speed_threshold && current_point.a() > accel_threshold && current_point.a() < 0.0) {
            break;
        }
        ++standstill_index;
    }
    // pick the last trajectory point as the end of current motion
    if (trajectory.trajectory_point_size() == standstill_index) {
        --standstill_index;
    }

    pid_control_context->set_path_remain(trajectory.trajectory_point(standstill_index).path_point().s() -
                                         pid_control_context->current_station());
    pid_control_context->set_open_loop_time(trajectory.header().timestamp_sec() +
                                            trajectory.trajectory_point(standstill_index).relative_time() -
                                            TimeUtility::GetCurrentTimesecond());

    // take parking, stop/starting as near-standstill case
    if (trajectory.trajectory_type() == pnc::planning::ADCTrajectory::NORMAL &&
        (pid_control_context->path_remain() < pid_conf.parking_full_stop_length() ||
         (std::fabs(pid_control_context->preview_accel_target()) <= vehicle_params_.deceleration_at_min() &&
          pid_control_context->preview_speed_target() <= vehicle_params_.max_abs_speed_when_stopped()))) {
        return true;
    }

    return false;
}

bool PidSpeedControllerV2::LoadPIDCalibrationTable(const pnc::control::PidSpeedControllerConf& pid_conf) {
    Interpolation1D::DataType xy[10];

    // init interpolator list
    if (!is_controller_initialized_) {
        for (int32_t i = 0; i < sizeof(xy) / sizeof(xy[0]); ++i) {
            interpolator_list_.emplace(i, std::make_unique<Interpolation1D>());
        }
    }

    google::protobuf::RepeatedPtrField<pnc::control::PidSpeedTable> pid_table;

    if (!is_current_backward_) {
        pid_table = pid_conf.forward_paras();
        LOG_INFO("PID calibration load forward table size is %d.", pid_conf.forward_paras_size());
    } else {
        pid_table = pid_conf.backward_paras();
        LOG_INFO("PID calibration load backward table size is %d.", pid_conf.backward_paras_size());
    }

    // setup interpolator list
    for (const auto& calibration : pid_table) {
        xy[0].push_back(std::make_pair(calibration.vel(), calibration.forsee_time()));
        xy[1].push_back(std::make_pair(calibration.vel(), calibration.forsee_min_distance()));
        xy[2].push_back(std::make_pair(calibration.vel(), calibration.speed_pid().kp()));
        xy[3].push_back(std::make_pair(calibration.vel(), calibration.speed_pid().ki()));
        xy[4].push_back(std::make_pair(calibration.vel(), calibration.speed_pid().kd()));
        xy[5].push_back(std::make_pair(calibration.vel(), calibration.speed_pid().integrator_saturation_level()));
        xy[6].push_back(std::make_pair(calibration.vel(), calibration.station_pid().kp()));
        xy[7].push_back(std::make_pair(calibration.vel(), calibration.station_pid().ki()));
        xy[8].push_back(std::make_pair(calibration.vel(), calibration.station_pid().kd()));
        xy[9].push_back(std::make_pair(calibration.vel(), calibration.station_pid().integrator_saturation_level()));
    }

    // for (const auto& interpolator : interpolator_list_) {
    for (int32_t i = 0; i < interpolator_list_.size(); ++i) {
        LOG_INFO("Initializing linear interpolator [%d]!", i);

        if (!interpolator_list_[i]->Init(xy[i])) {
            LOG_ERROR("Fail to initialize linear interpolator [%d]!", i);
            return false;
        }
    }

    return true;
}

bool PidSpeedControllerV2::UpdatePIDInterpolatedParams(const pnc::control::PidSpeedControllerConf& pid_conf,
                                                       const double curr_velocity,
                                                       LongitudinalControlContext* const context) {
    LongitudinalControlContext* pid_control_context = context;
    pnc::control::PidSpeedTable curr_pid_params;

    // extract interpolate value
    curr_pid_params.set_forsee_time(interpolator_list_[InterpolationType::kVelForseeTime]->Interpolate(curr_velocity));
    curr_pid_params.set_forsee_min_distance(
        interpolator_list_[InterpolationType::kVelForseeMinDis]->Interpolate(curr_velocity));
    curr_pid_params.mutable_speed_pid()->set_kp(
        interpolator_list_[InterpolationType::kVelSpdLoopKp]->Interpolate(curr_velocity));
    curr_pid_params.mutable_speed_pid()->set_ki(
        interpolator_list_[InterpolationType::kVelSpdLoopKi]->Interpolate(curr_velocity));
    curr_pid_params.mutable_speed_pid()->set_kd(
        interpolator_list_[InterpolationType::kVelSpdLoopKd]->Interpolate(curr_velocity));
    curr_pid_params.mutable_speed_pid()->set_integrator_saturation_level(
        interpolator_list_[InterpolationType::kVelSpdLoopIntgrSatLvl]->Interpolate(curr_velocity));
    curr_pid_params.mutable_station_pid()->set_kp(
        interpolator_list_[InterpolationType::kVelStatLoopKp]->Interpolate(curr_velocity));
    curr_pid_params.mutable_station_pid()->set_ki(
        interpolator_list_[InterpolationType::kVelStatLoopKi]->Interpolate(curr_velocity));
    curr_pid_params.mutable_station_pid()->set_kd(
        interpolator_list_[InterpolationType::kVelStatLoopKd]->Interpolate(curr_velocity));
    curr_pid_params.mutable_station_pid()->set_integrator_saturation_level(
        interpolator_list_[InterpolationType::kVelStatLoopIntgrSatLvl]->Interpolate(curr_velocity));

    // update pid controller parameters
    station_pid_controller_.SetPID(curr_pid_params.station_pid());
    speed_pid_controller_.SetPID(curr_pid_params.speed_pid());

    pid_control_context->mutable_pid_params()->set_forsee_time(curr_pid_params.forsee_time());
    pid_control_context->mutable_pid_params()->set_forsee_min_distance(curr_pid_params.forsee_min_distance());
    pid_control_context->mutable_pid_params()->set_station_kp(curr_pid_params.station_pid().kp());
    pid_control_context->mutable_pid_params()->set_station_ki(curr_pid_params.station_pid().ki());
    pid_control_context->mutable_pid_params()->set_station_kd(curr_pid_params.station_pid().kd());
    pid_control_context->mutable_pid_params()->set_station_integrator_saturation_level(
        curr_pid_params.station_pid().integrator_saturation_level());
    pid_control_context->mutable_pid_params()->set_speed_kp(curr_pid_params.speed_pid().kp());
    pid_control_context->mutable_pid_params()->set_speed_ki(curr_pid_params.speed_pid().ki());
    pid_control_context->mutable_pid_params()->set_speed_kd(curr_pid_params.speed_pid().kd());
    pid_control_context->mutable_pid_params()->set_speed_integrator_saturation_level(
        curr_pid_params.speed_pid().integrator_saturation_level());

    return true;
}

const double PidSpeedControllerV2::SlopeOffsetCompensation(const pnc::common::VehicleState& vehicle_state) {
    int32_t iflag = (vehicle_state.gear() == pnc::chassis::GEAR_REVERSE) ? -1 : 1;
    double slope_offset_compensation = iflag * GRA_ACC * std::sin(vehicle_state.pitch());

    if (std::isnan(slope_offset_compensation)) {
        slope_offset_compensation = 0;
    }

    return slope_offset_compensation;
}

const double PidSpeedControllerV2::GetOpenLoopDecel(const LongitudinalControlContext* context) {
    if (context->open_loop_time() < 0.0) {
        LOG_ERROR("Wrong open loop time!");
    }

    double decel = -std::sqrt(std::pow(context->current_speed(), 2) + std::pow(context->open_loop_time(), 2));
    if (decel <= vehicle_params_.deceleration_at_min()) {
        decel = vehicle_params_.deceleration_at_min();
    }

    return decel;
}

bool PidSpeedControllerV2::SummaryDebugInfo(const int32_t time_elapsed, const LongitudinalControlContext& context,
                                            ControlDebug* const control_debug) const {
    PidSpeedDebug* pid_speed_debug = control_debug->mutable_pid_speed_debug();
    pid_speed_debug->set_enable_controller(is_controller_initialized_);
    pid_speed_debug->set_cost_time_ms(time_elapsed);
    pid_speed_debug->mutable_lon_context()->CopyFrom(context);

    return true;
}

}  // namespace control
}  // namespace jiduauto
