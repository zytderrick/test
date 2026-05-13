/**
 * Copyright @ 2021 - 2023 JIDU AUTO CO.,LTD.
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are NOT permitted except as agreed by
 * JIDU AUTO CO.,LTD.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an " IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#include "control/src/controller/kin_mpc_controller.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "control/src/common/control_gflags.h"
#include "control/src/math/control_math.h"
#include "control/src/math/mpc_osqp_solver.h"
#include "pnc_common/src/common/pnc_gflag.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"
#include "pnc_planning.pb.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::math::Clamp;
using jiduauto::pnc::util::TimeUtility;
using pnc::chassis::Chassis;
using pnc::control::ControlCommand;
using pnc::control::ControlModeInfo;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

KinMpcController::KinMpcController() { LOG_INFO("%s used. ", Name().c_str()); }

KinMpcController::~KinMpcController() { Stop(); }

bool KinMpcController::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("%s begin init.", Name().c_str());
    use_debug_csv_file_ = (control_conf.enable_csv_debug() && control_conf.enable_controller_csv_debug());
    control_period_s_ = control_conf.control_period();
    kin_mpc_conf_ = control_conf.kin_mpc_controller_conf();
    vehicle_param_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();
    ts_ = kin_mpc_conf_.model_sample_time();
    pred_horizon_ = kin_mpc_conf_.pred_horizon();
    enable_localization_speed_ = control_conf.enable_localization_speed();
    enable_compensatory_acceleration_ = control_conf.enable_compensatory_acceleration();

    if (!MatrixInit()) {
        LOG_ERROR("MatrixInit failed");
        return false;
    }

    if (!LoadMPCGainScheduler()) {
        LOG_ERROR("LoadMPCGainScheduler failed");
        return false;
    }

    // init memeber vector
    reference_local_matrixs_.reserve(pred_horizon_ + 1);
    for (int32_t i = 0; i < pred_horizon_ + 1; ++i) {
        Eigen::MatrixXd state_matrix(KinModelStateIndex::kSize, 1);
        reference_local_matrixs_.push_back(state_matrix);
    }

    TimeGridGenerate(kin_mpc_conf_.step_gain());

    controller_initialized_ = true;

    return true;
}

bool KinMpcController::Reset() { return true; }

std::string KinMpcController::Name() const { return "KinMpcController"; }

void KinMpcController::Stop() {
    if (kin_mpc_log_file_ != nullptr) {
        fclose(kin_mpc_log_file_);
        kin_mpc_log_file_ = nullptr;
    }
    localization_msg_ptr_ = nullptr;
    chassis_msg_ptr_ = nullptr;
    trajectory_msg_ptr_ = nullptr;
}

bool KinMpcController::MatrixInit() {
    // Matrix init operations.
    // forward
    kin_osqp_model_.matrix_ad = Eigen::MatrixXd::Zero(KinModelStateIndex::kSize, KinModelStateIndex::kSize);
    kin_osqp_model_.matrix_bd = Eigen::MatrixXd::Zero(KinModelStateIndex::kSize, KinModelInputIndex::kSize);
    kin_osqp_model_.matrix_gd = Eigen::MatrixXd::Zero(KinModelStateIndex::kSize, 1);
    kin_osqp_model_.matrix_state = Eigen::MatrixXd::Zero(KinModelStateIndex::kSize, 1);
    kin_osqp_model_.matrix_r = Eigen::MatrixXd::Zero(KinModelInputIndex::kSize, KinModelInputIndex::kSize);
    kin_osqp_model_.matrix_q = Eigen::MatrixXd::Zero(KinModelStateIndex::kSize, KinModelStateIndex::kSize);

    // conf init
    // sanity check
    int32_t q_param_size = kin_mpc_conf_.base_conf().matrix_q_size();
    if (q_param_size != KinModelStateIndex::kSize) {
        LOG_ERROR("MPC controller error: matrix_r size: [%f]", q_param_size,
                  " in parameter file not equal to basic_state_size_: [%f]", KinModelStateIndex::kSize);
        return false;
    }

    for (int32_t i = 0; i < q_param_size; ++i) {
        kin_osqp_model_.matrix_q(i, i) = kin_mpc_conf_.base_conf().matrix_q(i);
    }

    int32_t r_param_size = kin_mpc_conf_.base_conf().matrix_r_size();
    if (r_param_size != KinModelInputIndex::kSize) {
        LOG_ERROR("MPC controller error: matrix_r size: [%f]", r_param_size,
                  " in parameter file not equal to basic_control_size_: [%f]", KinModelInputIndex::kSize);
        return false;
    }

    for (int32_t i = 0; i < r_param_size; ++i) {
        kin_osqp_model_.matrix_r(i, i) = kin_mpc_conf_.base_conf().matrix_r(i);
    }

    return true;
}

bool KinMpcController::LoadMPCGainScheduler() {
    const auto& x_state_scheduler = kin_mpc_conf_.x_state_scheduler();
    const auto& y_state_scheduler = kin_mpc_conf_.y_state_scheduler();
    const auto& theta_state_scheduler = kin_mpc_conf_.theta_state_scheduler();
    const auto& speed_state_scheduler = kin_mpc_conf_.speed_state_scheduler();
    const auto& steer_state_scheduler = kin_mpc_conf_.steer_state_scheduler();

    const auto& acc_scheduler = kin_mpc_conf_.acc_scheduler();
    const auto& steer_gain_scheduler = kin_mpc_conf_.steer_gain_scheduler();

    Interpolation1D::DataType xy1, xy2, xy3, xy4, xy5, xy6, xy7;
    for (const auto& scheduler : acc_scheduler.scheduler()) {
        xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : steer_gain_scheduler.scheduler()) {
        xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : x_state_scheduler.scheduler()) {
        xy3.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : y_state_scheduler.scheduler()) {
        xy4.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : theta_state_scheduler.scheduler()) {
        xy5.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : speed_state_scheduler.scheduler()) {
        xy6.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : steer_state_scheduler.scheduler()) {
        xy7.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }

    if (!acc_control_interpolation_->Init(xy1)) {
        LOG_ERROR("Fail to load acc gain scheduler for MPC controller");
        return false;
    }

    if (!steer_gain_control_interpolation_->Init(xy2)) {
        LOG_ERROR("Fail to load steer gain scheduler for MPC controller");
        return false;
    }

    if (!x_state_interpolation_->Init(xy3)) {
        LOG_ERROR("Fail to load x scheduler for MPC controller");
        return false;
    }

    if (!y_state_interpolation_->Init(xy4)) {
        LOG_ERROR("Fail to load y scheduler for MPC controller");
        return false;
    }

    if (!theta_state_interpolation_->Init(xy5)) {
        LOG_ERROR("Fail to load theta scheduler for MPC controller");
        return false;
    }

    if (!speed_state_interpolation_->Init(xy6)) {
        LOG_ERROR("Fail to load speed scheduler for MPC controller");
        return false;
    }

    if (!steer_state_interpolation_->Init(xy7)) {
        LOG_ERROR("Fail to load steer scheduler for MPC controller");
        return false;
    }

    return true;
}

void KinMpcController::UpdateCostWeightsBySpeed(double current_speed) {
    kin_osqp_model_.matrix_q(KinModelStateIndex::kX, KinModelStateIndex::kX) =
        kin_osqp_model_.matrix_q(KinModelStateIndex::kX, KinModelStateIndex::kX) *
        x_state_interpolation_->LinearInterpolate(current_speed);
    kin_osqp_model_.matrix_q(KinModelStateIndex::kY, KinModelStateIndex::kY) =
        kin_osqp_model_.matrix_q(KinModelStateIndex::kY, KinModelStateIndex::kY) *
        y_state_interpolation_->LinearInterpolate(current_speed);
    kin_osqp_model_.matrix_q(KinModelStateIndex::kTheta, KinModelStateIndex::kTheta) =
        kin_osqp_model_.matrix_q(KinModelStateIndex::kTheta, KinModelStateIndex::kTheta) *
        theta_state_interpolation_->LinearInterpolate(current_speed);
    kin_osqp_model_.matrix_q(KinModelStateIndex::kSpeed, KinModelStateIndex::kSpeed) =
        kin_osqp_model_.matrix_q(KinModelStateIndex::kSpeed, KinModelStateIndex::kSpeed) *
        speed_state_interpolation_->LinearInterpolate(current_speed);
    kin_osqp_model_.matrix_q(KinModelStateIndex::kDelta, KinModelStateIndex::kDelta) =
        kin_osqp_model_.matrix_q(KinModelStateIndex::kDelta, KinModelStateIndex::kDelta) *
        steer_state_interpolation_->LinearInterpolate(current_speed);

    kin_osqp_model_.matrix_r(KinModelInputIndex::kAcc, KinModelInputIndex::kAcc) =
        kin_osqp_model_.matrix_r(KinModelInputIndex::kAcc, KinModelInputIndex::kAcc) *
        acc_control_interpolation_->LinearInterpolate(current_speed);
    kin_osqp_model_.matrix_r(KinModelInputIndex::kDeltaRate, KinModelInputIndex::kDeltaRate) =
        kin_osqp_model_.matrix_r(KinModelInputIndex::kDeltaRate, KinModelInputIndex::kDeltaRate) *
        steer_gain_control_interpolation_->LinearInterpolate(current_speed);
}

void KinMpcController::UpdateCostWeightsByMode(const ControlModeInfo::ControlScenarioMode& mode) {
    if (!kin_mpc_conf_.enable_mode_control() || mode == ControlModeInfo::NONE) {
        kin_osqp_model_.matrix_q(KinModelStateIndex::kX, KinModelStateIndex::kX) =
            kin_mpc_conf_.base_conf().matrix_q(KinModelStateIndex::kX);
        kin_osqp_model_.matrix_q(KinModelStateIndex::kY, KinModelStateIndex::kY) =
            kin_mpc_conf_.base_conf().matrix_q(KinModelStateIndex::kY);
        kin_osqp_model_.matrix_q(KinModelStateIndex::kTheta, KinModelStateIndex::kTheta) =
            kin_mpc_conf_.base_conf().matrix_q(KinModelStateIndex::kTheta);
        kin_osqp_model_.matrix_q(KinModelStateIndex::kSpeed, KinModelStateIndex::kSpeed) =
            kin_mpc_conf_.base_conf().matrix_q(KinModelStateIndex::kSpeed);
        kin_osqp_model_.matrix_q(KinModelStateIndex::kDelta, KinModelStateIndex::kDelta) =
            kin_mpc_conf_.base_conf().matrix_q(KinModelStateIndex::kDelta);

        kin_osqp_model_.matrix_r(KinModelInputIndex::kAcc, KinModelInputIndex::kAcc) =
            kin_mpc_conf_.base_conf().matrix_r(KinModelInputIndex::kAcc);
        kin_osqp_model_.matrix_r(KinModelInputIndex::kDeltaRate, KinModelInputIndex::kDeltaRate) =
            kin_mpc_conf_.base_conf().matrix_r(KinModelInputIndex::kDeltaRate);
    } else {
        switch (mode) {
        case ControlModeInfo::APPROACH_TO_STOP:
            kin_osqp_model_.matrix_q(KinModelStateIndex::kX, KinModelStateIndex::kX) =
                kin_mpc_conf_.stop_conf().matrix_q(KinModelStateIndex::kX);
            kin_osqp_model_.matrix_q(KinModelStateIndex::kY, KinModelStateIndex::kY) =
                kin_mpc_conf_.stop_conf().matrix_q(KinModelStateIndex::kY);
            kin_osqp_model_.matrix_q(KinModelStateIndex::kTheta, KinModelStateIndex::kTheta) =
                kin_mpc_conf_.stop_conf().matrix_q(KinModelStateIndex::kTheta);
            kin_osqp_model_.matrix_q(KinModelStateIndex::kSpeed, KinModelStateIndex::kSpeed) =
                kin_mpc_conf_.stop_conf().matrix_q(KinModelStateIndex::kSpeed);
            kin_osqp_model_.matrix_q(KinModelStateIndex::kDelta, KinModelStateIndex::kDelta) =
                kin_mpc_conf_.stop_conf().matrix_q(KinModelStateIndex::kDelta);
            kin_osqp_model_.matrix_r(KinModelInputIndex::kAcc, KinModelInputIndex::kAcc) =
                kin_mpc_conf_.stop_conf().matrix_r(KinModelInputIndex::kAcc);
            kin_osqp_model_.matrix_r(KinModelInputIndex::kDeltaRate, KinModelInputIndex::kDeltaRate) =
                kin_mpc_conf_.stop_conf().matrix_r(KinModelInputIndex::kDeltaRate);
            break;
        default:
            break;
        }
    }
}

void KinMpcController::UpdateCurrentState(const KinematicModelState& current_state) {
    kin_osqp_model_.matrix_state(0, 0) = current_state.X;
    kin_osqp_model_.matrix_state(1, 0) = current_state.Y;
    kin_osqp_model_.matrix_state(2, 0) = current_state.theta;
    kin_osqp_model_.matrix_state(3, 0) = current_state.speed;
    kin_osqp_model_.matrix_state(4, 0) = current_state.delta;
}

bool KinMpcController::Solve() {
    control_cmd_matrix_ = Eigen::MatrixXd::Zero(KinModelInputIndex::kSize, pred_horizon_);
    control_state_matrix_ = Eigen::MatrixXd::Zero(KinModelStateIndex::kSize, pred_horizon_ + 1);

    double max_angle_rad = vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio();

    double max_angle_rad_rate = vehicle_param_.max_steer_angle_rate() / vehicle_param_.steer_ratio();  // [rad/s]

    double min_accel = vehicle_param_.max_deceleration();
    double max_accel = vehicle_param_.max_acceleration();

    Eigen::MatrixXd lower_input_bound(KinModelInputIndex::kSize, 1);
    lower_input_bound << -std::abs(min_accel), -max_angle_rad_rate;

    Eigen::MatrixXd upper_input_bound(KinModelInputIndex::kSize, 1);
    upper_input_bound << max_accel, max_angle_rad_rate;

    const double max = std::numeric_limits<double>::max();
    Eigen::MatrixXd lower_state_bound(KinModelStateIndex::kSize, 1);
    Eigen::MatrixXd upper_state_bound(KinModelStateIndex::kSize, 1);

    if (is_current_backward_) {
        lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max_angle_rad;
        upper_state_bound << max, max, M_PI, 0.0, max_angle_rad;
    } else {
        lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, 0.0, -1.0 * max_angle_rad;
        upper_state_bound << max, max, M_PI, max, max_angle_rad;
    }

    std::vector<double> control_cmds(KinModelInputIndex::kSize * pred_horizon_, 0);
    std::vector<double> control_states(KinModelStateIndex::kSize * (pred_horizon_ + 1), 0);

    MpcOSQPSolver mpc_kin_solver(kin_osqp_model_.matrix_ad, kin_osqp_model_.matrix_bd, kin_osqp_model_.matrix_q,
                                 kin_osqp_model_.matrix_r, kin_osqp_model_.matrix_gd, kin_osqp_model_.matrix_state,
                                 lower_input_bound, upper_input_bound, lower_state_bound, upper_state_bound,
                                 reference_local_matrixs_, kin_mpc_conf_.max_iteration(), pred_horizon_,
                                 kin_mpc_conf_.eps());

    if (!mpc_kin_solver.Execute(&control_cmds, &control_states)) {
        LOG_ERROR("MPC OSQP solver failed");
        return false;
    }

    for (int32_t i = 0; i < pred_horizon_; ++i) {
        control_cmd_matrix_(KinModelInputIndex::kAcc, i) =
            control_cmds.at(i * KinModelInputIndex::kSize + KinModelInputIndex::kAcc);
        control_cmd_matrix_(KinModelInputIndex::kDeltaRate, i) =
            control_cmds.at(i * KinModelInputIndex::kSize + KinModelInputIndex::kDeltaRate);
    }
    for (int32_t i = 0; i < (pred_horizon_ + 1); ++i) {
        control_state_matrix_(KinModelStateIndex::kX, i) =
            control_states.at(i * KinModelStateIndex::kSize + KinModelStateIndex::kX);
        control_state_matrix_(KinModelStateIndex::kY, i) =
            control_states.at(i * KinModelStateIndex::kSize + KinModelStateIndex::kY);
        control_state_matrix_(KinModelStateIndex::kTheta, i) =
            control_states.at(i * KinModelStateIndex::kSize + KinModelStateIndex::kTheta);
        control_state_matrix_(KinModelStateIndex::kSpeed, i) =
            control_states.at(i * KinModelStateIndex::kSize + KinModelStateIndex::kSpeed);
        control_state_matrix_(KinModelStateIndex::kDelta, i) =
            control_states.at(i * KinModelStateIndex::kSize + KinModelStateIndex::kDelta);
    }

    cur_solution_time_ = TimeUtility::GetCurrentTimeMillsecond();
    return true;
}

void KinMpcController::UpdateStateMatrix(const KinLinModelMatrix& linear_model) {
    kin_osqp_model_.matrix_ad = linear_model.A;
}

void KinMpcController::UpdateInputMatrix(const KinLinModelMatrix& linear_model) {
    kin_osqp_model_.matrix_bd = linear_model.B;
}

void KinMpcController::UpdateLinearizationMatrix(const KinLinModelMatrix& linear_model) {
    kin_osqp_model_.matrix_gd = linear_model.d;
}

const std::vector<KinematicModelState> KinMpcController::UpdateLocalReferenceState(
    const ADCTrajectory* target_glb_trajectory_ptr, const KinematicModelState& current_state) {
    std::vector<KinematicModelState> reference_local_state_seq;
    double resample_delta_time =
        std::max(0.0, TimeUtility::GetCurrentTimesecond() - target_glb_trajectory_ptr->header().timestamp_sec());

    Interpolation1D::DataType t_x_local_interp;
    Interpolation1D::DataType t_y_local_interp;
    Interpolation1D::DataType t_theta_local_interp;
    Interpolation1D::DataType t_speed_interp;

    TrajectoryAnalyzer trajectory_analyzer(target_glb_trajectory_ptr);
    pnc::common::TrajectoryPoint origin_glb_point;
    origin_glb_point.mutable_path_point()->set_x(localization_msg_ptr_->loc_pose().pose().position().x());
    origin_glb_point.mutable_path_point()->set_y(localization_msg_ptr_->loc_pose().pose().position().y());
    auto& orientation = localization_msg_ptr_->loc_pose().pose().orientation();
    EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
    origin_glb_point.mutable_path_point()->set_theta(q.yaw());
    std::vector<pnc::common::TrajectoryPoint> local_trajectory_points =
        trajectory_analyzer.ToVehicleFrame(origin_glb_point);

    for (int32_t i = 0; i < static_cast<int32_t>(local_trajectory_points.size()); i++) {
        t_x_local_interp.emplace_back(std::make_pair(target_glb_trajectory_ptr->trajectory_point(i).relative_time(),
                                                     local_trajectory_points[i].path_point().x()));
        t_y_local_interp.emplace_back(std::make_pair(target_glb_trajectory_ptr->trajectory_point(i).relative_time(),
                                                     local_trajectory_points[i].path_point().y()));
        t_theta_local_interp.emplace_back(std::make_pair(target_glb_trajectory_ptr->trajectory_point(i).relative_time(),
                                                         local_trajectory_points[i].path_point().theta()));
        t_speed_interp.emplace_back(std::make_pair(target_glb_trajectory_ptr->trajectory_point(i).relative_time(),
                                                   target_glb_trajectory_ptr->trajectory_point(i).v()));
    }

    bool fail_to_init_interpolation = false;
    std::unique_ptr<Interpolation1D> tx_interpolation = std::make_unique<Interpolation1D>();
    if (!tx_interpolation->Init(t_x_local_interp)) {
        LOG_ERROR("Fail to init time and x table");
        fail_to_init_interpolation = true;
    }
    std::unique_ptr<Interpolation1D> ty_interpolation = std::make_unique<Interpolation1D>();
    if (!ty_interpolation->Init(t_y_local_interp)) {
        LOG_ERROR("Fail to init time and y table");
        fail_to_init_interpolation = true;
    }
    std::unique_ptr<Interpolation1D> ttheta_interpolation = std::make_unique<Interpolation1D>();
    if (!ttheta_interpolation->Init(t_theta_local_interp)) {
        LOG_ERROR("Fail to init time and theta table");
        fail_to_init_interpolation = true;
    }
    std::unique_ptr<Interpolation1D> tspeed_interpolation = std::make_unique<Interpolation1D>();
    if (!tspeed_interpolation->Init(t_speed_interp)) {
        LOG_ERROR("Fail to init time and speed table");
        fail_to_init_interpolation = true;
    }

    // resample the target local states
    if (fail_to_init_interpolation) {
        return reference_local_state_seq;
    }

    for (auto time : time_grid_) {
        KinematicModelState reference_local_state;
        double revise_time = resample_delta_time + time;
        reference_local_state.setZero();
        reference_local_state.X = tx_interpolation->LinearInterpolate(revise_time);
        reference_local_state.Y = ty_interpolation->LinearInterpolate(revise_time);
        reference_local_state.theta = ttheta_interpolation->LinearInterpolate(revise_time);
        reference_local_state.speed = tspeed_interpolation->LinearInterpolate(revise_time);
        if (kin_mpc_conf_.enable_self_speed_target()) {
            reference_local_state.speed = kin_mpc_conf_.self_speed_target_mps();
        }
        if (is_current_backward_) {
            reference_local_state.speed = -1.0 * reference_local_state.speed;
        }
        reference_local_state.delta = 0.0;
        reference_local_state_seq.push_back(reference_local_state);
    }
    return reference_local_state_seq;
}

void KinMpcController::TimeGridGenerate(double step_gain) {
    double last_grid = 0.0;
    if (kin_mpc_conf_.enable_augmented_timegrid()) {
        for (int32_t i = 0; i < pred_horizon_ + 1; i++) {
            double step = i * step_gain;
            double grid = last_grid + step;
            time_grid_.push_back(grid);
            last_grid = grid;
        }
    } else {
        for (int32_t i = 0; i < pred_horizon_ + 1; i++) {
            time_grid_.push_back(i * step_gain);
        }
    }
}

// Convert
Eigen::MatrixXd KinMpcController::StateToMatrix(const KinematicModelState& mpc_state) {
    Eigen::MatrixXd state_matrix(KinModelStateIndex::kSize, 1);
    state_matrix(KinModelStateIndex::kX) = mpc_state.X;
    state_matrix(KinModelStateIndex::kY) = mpc_state.Y;
    state_matrix(KinModelStateIndex::kTheta) = mpc_state.theta;
    state_matrix(KinModelStateIndex::kSpeed) = mpc_state.speed;
    state_matrix(KinModelStateIndex::kDelta) = mpc_state.delta;

    return state_matrix;
}

Eigen::MatrixXd KinMpcController::InputToMatrix(const KinematicModelInput& mpc_input) {
    Eigen::MatrixXd input_matrix(KinModelInputIndex::kSize, 1);
    input_matrix(KinModelInputIndex::kAcc) = mpc_input.acc;
    input_matrix(KinModelInputIndex::kDeltaRate) = mpc_input.delta_rate;

    return input_matrix;
}

KinematicModelState KinMpcController::MatrixToState(const Eigen::MatrixXd& state_matrix) {
    KinematicModelState mpc_state;
    mpc_state.X = state_matrix(KinModelStateIndex::kX);
    mpc_state.Y = state_matrix(KinModelStateIndex::kY);
    mpc_state.theta = state_matrix(KinModelStateIndex::kTheta);
    mpc_state.speed = state_matrix(KinModelStateIndex::kSpeed);
    mpc_state.delta = state_matrix(KinModelStateIndex::kDelta);

    return mpc_state;
}

KinematicModelInput KinMpcController::MatrixToInput(const Eigen::MatrixXd& input_matrix) {
    KinematicModelInput mpc_input;
    mpc_input.acc = input_matrix(KinModelInputIndex::kAcc);
    mpc_input.delta_rate = input_matrix(KinModelInputIndex::kDeltaRate);

    return mpc_input;
}

void KinMpcController::OutputCommandProcess(const KinematicModelState& current_mpc_state) {
    double steer_angle_target = 0.0;
    double control_desicred_acc = 0.0;
    double steer_angle_target_rate = 0.0;

    // const double single_wheel_steering_max_rate =
    //     vehicle_param_.max_steer_angle_rate() * 180.0 / M_PI / vehicle_param_.steer_ratio() / 180.0 * M_PI;
    const double single_wheel_steering_max = vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio();

    if (std::isnan(control_cmd_matrix_(0, 0)) || std::isnan(control_cmd_matrix_(1, 0))) {
        LOG_ERROR("Kin Solver give NaN;");
        steer_angle_target = 0.0;
        control_desicred_acc = 0.0;
    } else {
        steer_angle_target_rate = (control_state_matrix_(KinModelStateIndex::kDelta, 1) -
                                   control_state_matrix_(KinModelStateIndex::kDelta, 0)) /
                                  ts_;
        if (kin_mpc_conf_.enable_direct_steer()) {
            steer_angle_target =
                Clamp(control_state_matrix_(KinModelStateIndex::kDelta, kin_mpc_conf_.steering_delay_index()),
                      -single_wheel_steering_max, single_wheel_steering_max);
        } else {
            steer_angle_target = Clamp(steer_angle_target_rate * control_period_s_ + current_mpc_state.delta,
                                       -single_wheel_steering_max, single_wheel_steering_max);
        }
    }

    control_desicred_acc = control_cmd_matrix_(0, 0);

    last_steer_rate_cmd_ = steer_angle_target_rate;
    last_steer_cmd_ = steer_angle_target;

    if (is_current_backward_) {
        control_desicred_acc = -1.0 * control_desicred_acc;
    }

    last_acc_cmd_ = control_desicred_acc;

    double throttle_cmd = 0.0;
    double brake_cmd = 0.0;
    if (control_desicred_acc > 0.0) {
        double rough_throttle = ControlMath::GetAccelerationThrottle(
            control_desicred_acc, vehicle_param_.acceleration_at_min(), vehicle_param_.acceleration_per_001_throttle(),
            vehicle_param_.acceleration_throttle_max(), vehicle_param_.acceleration_throttle_min());

        throttle_cmd = std::min(rough_throttle, vehicle_param_.acceleration_throttle_max());
        brake_cmd = 0.0;
    } else {
        double rough_brake = ControlMath::GetDecelerationBrake(
            control_desicred_acc, vehicle_param_.deceleration_at_min(), vehicle_param_.deceleration_per_001_brake(),
            vehicle_param_.deceleration_brake_max(), vehicle_param_.deceleration_brake_min());

        brake_cmd = std::min(rough_brake, vehicle_param_.deceleration_brake_max());
        throttle_cmd = 0.0;
    }

    double steering_wheel_deg_target = steer_angle_target * vehicle_param_.steer_ratio() / M_PI * 180.0;
    command_msg_ptr_->set_steering_target(steering_wheel_deg_target);

    command_msg_ptr_->set_acceleration(control_desicred_acc);
    command_msg_ptr_->set_throttle(throttle_cmd);
    command_msg_ptr_->set_brake(brake_cmd);

    if (FLAGS_enable_control_info_print) {
        LOG_INFO("kin_mpc cmd STR TGT rad: [%f]", steer_angle_target);
        LOG_INFO("kin_mpc cmd ACC TGT: [%f]", control_desicred_acc);
    }
}

bool KinMpcController::SummaryDebugInfo(
    const KinematicModelState& current_mpc_state, const std::vector<KinematicModelState>& reference_states,
    const std::vector<KinematicModelState>& pred_states, const std::vector<KinematicModelInput>& cmd_seqs,
    const std::vector<double>& time_grid, const int32_t to_update_cur_state_ms, const int32_t to_matrix_build_ms,
    const int32_t to_solve_end_ms, const int32_t to_output_ms, const int32_t main_cost_time,
    const ControlModeInfo::ControlScenarioMode& mode, pnc::control::ControlDebug* const control_debug) const {
    pnc::control::KinematicMpcDebug* kinematic_debug = control_debug->mutable_kin_mpc_debug();
    kinematic_debug->set_enable_controller(true);
    kinematic_debug->set_mode(mode);

    kinematic_debug->set_cost_time_ms(main_cost_time);
    kinematic_debug->set_to_update_cur_state_ms(to_update_cur_state_ms);
    kinematic_debug->set_to_matrix_build_ms(to_matrix_build_ms);
    kinematic_debug->set_to_solve_end_ms(to_solve_end_ms);
    kinematic_debug->set_to_output_ms(to_output_ms);

    pnc::common::VehicleState* current_state = kinematic_debug->mutable_current_state();
    current_state->set_x(current_mpc_state.X);
    current_state->set_y(current_mpc_state.Y);
    current_state->set_heading(current_mpc_state.theta);
    current_state->set_linear_velocity(current_mpc_state.speed);

    kinematic_debug->set_acc_cmd(command_msg_ptr_->acceleration());
    kinematic_debug->set_steer_wheel_cmd_deg(command_msg_ptr_->steering_target());

    for (int32_t i = 0; i < kin_mpc_conf_.base_conf().matrix_q().size(); ++i) {
        kinematic_debug->add_final_q(kin_osqp_model_.matrix_q(i, i));
    }
    for (int32_t i = 0; i < kin_mpc_conf_.base_conf().matrix_r().size(); ++i) {
        kinematic_debug->add_final_r(kin_osqp_model_.matrix_r(i, i));
    }

    int32_t ref_index = 0;
    for (const KinematicModelState& ref_state : reference_states) {
        pnc::common::VehicleState* ref_kin_state = kinematic_debug->add_ref_kin_state();
        ref_kin_state->set_index(ref_index);
        ref_kin_state->set_timestamp_sec(time_grid.at(ref_index));
        ref_index++;
        ref_kin_state->set_x(ref_state.X);
        ref_kin_state->set_y(ref_state.Y);
        ref_kin_state->set_heading(ref_state.theta);
        ref_kin_state->set_linear_velocity(ref_state.speed);
        ref_kin_state->set_steering_angle(ref_state.delta);
    }

    int32_t pred_index = 0;
    for (const KinematicModelState& pred_state : pred_states) {
        pnc::common::VehicleState* pred_kin_state = kinematic_debug->add_pred_kin_state();
        pred_kin_state->set_index(pred_index);
        pred_kin_state->set_timestamp_sec(time_grid.at(pred_index));
        pred_index++;
        pred_kin_state->set_x(pred_state.X);
        pred_kin_state->set_y(pred_state.Y);
        pred_kin_state->set_heading(pred_state.theta);
        pred_kin_state->set_linear_velocity(pred_state.speed);
        pred_kin_state->set_steering_angle(pred_state.delta);
        if (pred_index <= pred_horizon_) {
            pred_kin_state->set_linear_acceleration(cmd_seqs[pred_index].acc);
            pred_kin_state->set_steering_angle_rate(cmd_seqs[pred_index].delta_rate);
        }
    }

    return true;
}

bool KinMpcController::ControlMain(const ControlInputStream& control_input_stream,
                                   ControlCommandStream* const control_command_stream) {
    int64_t main_start_time = TimeUtility::GetCurrentTimeMillsecond();

    if (control_command_stream == nullptr) {
        LOG_ERROR("input is nullptr");
        return false;
    }

    if (!controller_initialized_) {
        LOG_ERROR("controller_initialized_ failed");
        return false;
    }
    localization_msg_ptr_ = &control_input_stream.input_stream.localization;
    chassis_msg_ptr_ = &control_input_stream.input_stream.chassis;
    trajectory_msg_ptr_ = &control_input_stream.input_stream.planning_info;
    command_msg_ptr_ = &control_command_stream->control_command;

    if (control_input_stream.vehicle_state.driving_mode() != pnc::chassis::COMPLETE_MANUAL &&
        ControlMath::GearIsReady(localization_msg_ptr_, chassis_msg_ptr_, trajectory_msg_ptr_)) {
        is_current_auto_mode_ = true;
    } else {
        is_current_auto_mode_ = false;
    }

    kin_control_mode_ = RecogniteTrajectoryMode(control_input_stream.input_stream.planning_info);

    bool curr_backward = ControlMath::CheckBackward(chassis_msg_ptr_);

    if (curr_backward != is_current_backward_) {
        is_current_backward_ = curr_backward;
        Reset();
        LOG_INFO("Reset()");
    }

    KinematicModelState current_local_state;
    KinematicModelInput current_cmd;  // TODO(zhaobolin): should from last input
    current_local_state.X = 0.0;
    current_local_state.Y = 0.0;
    current_local_state.theta = 0.0;

    // switch between localization speed or chassis speed
    if (enable_localization_speed_) {
        current_local_state.speed = control_input_stream.vehicle_state.localization_velocity();
    } else {
        current_local_state.speed = control_input_stream.vehicle_state.linear_velocity();
    }

    if (enable_compensatory_acceleration_) {
        current_cmd.acc = control_input_stream.vehicle_state.linear_acceleration_with_compensation();
    } else {
        current_cmd.acc = control_input_stream.vehicle_state.linear_acceleration();
    }

    if (is_current_backward_) {
        current_local_state.speed = -1.0 * current_local_state.speed;
        current_cmd.acc = -1.0 * current_cmd.acc;
    }

    if (kin_mpc_conf_.use_feedback_cmd_data() || control_input_stream.vehicle_state.standstill()) {
        current_cmd.delta_rate = control_input_stream.vehicle_state.steering_angle_rate();
        current_local_state.delta =
            control_input_stream.vehicle_state.steering_angle() / vehicle_param_.steer_ratio() / 180.0 * M_PI;
    } else {
        current_cmd.delta_rate = last_steer_rate_cmd_;
        current_cmd.acc = last_acc_cmd_;
        current_local_state.delta = last_steer_cmd_;
    }  // TODO(bolin):  test longitudinal later

    UpdateCurrentState(current_local_state);
    int32_t to_update_cur_state_ms = static_cast<int32_t>(TimeUtility::GetCurrentTimeMillsecond() - main_start_time);

    // Model Iteration
    KinematicModel kinematic_model(ts_, vehicle_param_.wheel_base());  // a sample-updated model

    KinLinModelMatrix current_dis_linear_model = kinematic_model.getDiscreteLinModel(current_local_state, current_cmd);

    UpdateStateMatrix(current_dis_linear_model);
    UpdateInputMatrix(current_dis_linear_model);
    UpdateLinearizationMatrix(current_dis_linear_model);

    UpdateCostWeightsByMode(kin_control_mode_);
    UpdateCostWeightsBySpeed(std::abs(current_local_state.speed));

    std::vector<KinematicModelState> reference_local_states =
        UpdateLocalReferenceState(trajectory_msg_ptr_, current_local_state);

    if (static_cast<int32_t>(reference_local_states.size()) == 0) {
        LOG_ERROR("No reference states for solver.");
        return false;
    }  // TODO(zhaobolin): should be handled in input check

    reference_local_matrixs_.clear();
    for (int32_t i = 0; i < pred_horizon_ + 1; ++i) {
        reference_local_matrixs_.emplace_back(StateToMatrix(reference_local_states.at(i)));
    }

    int32_t to_matrix_build_ms = static_cast<int32_t>(TimeUtility::GetCurrentTimeMillsecond() - main_start_time);

    if (kin_control_mode_ == ControlModeInfo::TRAJ_PAUSE) {
        LOG_INFO("Kin Contrller in Pause Mode.");
        if (!PauseLogic(current_local_state.speed)) {
            LOG_ERROR("MPC Solved failed in pause mode");
            return false;
        }
    } else {
        if (!Solve()) {
            LOG_ERROR("MPC Solved failed in general mode");
            return false;
        }
    }

    int32_t to_solve_end_ms = static_cast<int32_t>(TimeUtility::GetCurrentTimeMillsecond() - main_start_time);

    std::vector<KinematicModelState> pred_states;
    for (int32_t i = 0; i < pred_horizon_ + 1; ++i) {
        pred_states.emplace_back(MatrixToState(control_state_matrix_.col(i)));
    }
    std::vector<KinematicModelInput> cmd_seqs;
    for (int32_t i = 0; i < pred_horizon_; ++i) {
        cmd_seqs.emplace_back(MatrixToInput(control_cmd_matrix_.col(i)));
    }

    OutputCommandProcess(current_local_state);
    int32_t to_output_ms = static_cast<int32_t>(TimeUtility::GetCurrentTimeMillsecond() - main_start_time);

    int32_t main_total_time = static_cast<int32_t>(TimeUtility::GetCurrentTimeMillsecond() - main_start_time);

    SummaryDebugInfo(current_local_state, reference_local_states, pred_states, cmd_seqs, time_grid_,
                     to_update_cur_state_ms, to_matrix_build_ms, to_solve_end_ms, to_output_ms, main_total_time,
                     kin_control_mode_, &control_command_stream->control_debug);
    LogDebugFile(control_input_stream, control_command_stream->control_debug.kin_mpc_debug());

    return true;
}

bool KinMpcController::Control(const pnc::localization::LocalizationEstimate* const localization,
                               const pnc::chassis::Chassis* const chassis,
                               const pnc::planning::ADCTrajectory* const trajectory,
                               pnc::control::ControlCommand* const cmd) {
    return false;
}

void KinMpcController::LogDebugFile(const ControlInputStream& control_input_stream,
                                    const pnc::control::KinematicMpcDebug& kin_mpc_debug) {
    if (use_debug_csv_file_) {
        double current_loc_time = control_input_stream.vehicle_state.timestamp_sec();
        double pos_glb_x = control_input_stream.vehicle_state.x();
        double pos_glb_y = control_input_stream.vehicle_state.y();
        double pos_glb_phi = control_input_stream.vehicle_state.heading();
        double chassis_spd = control_input_stream.vehicle_state.linear_velocity();
        double yaw_rate = control_input_stream.vehicle_state.angular_velocity();
        double chassis_acc = control_input_stream.vehicle_state.linear_acceleration();

        double compute_time_ms = kin_mpc_debug.cost_time_ms();
        double to_update_cur_state_ms = kin_mpc_debug.to_update_cur_state_ms();
        double to_matrix_build_ms = kin_mpc_debug.to_matrix_build_ms();
        double to_solve_end_ms = kin_mpc_debug.to_solve_end_ms();
        double to_output_ms = kin_mpc_debug.to_output_ms();

        double acc_cmd = kin_mpc_debug.acc_cmd();
        double steer_wheel_cmd_deg = kin_mpc_debug.steer_wheel_cmd_deg();

        if (!(kin_mpc_log_file_ == nullptr && !InitLogFile())) {
            fprintf(kin_mpc_log_file_, "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,",
                    is_current_auto_mode_, current_loc_time, pos_glb_x, pos_glb_y, pos_glb_phi, chassis_spd, yaw_rate,
                    chassis_acc, compute_time_ms, acc_cmd, steer_wheel_cmd_deg, kin_control_mode_,
                    to_update_cur_state_ms, to_matrix_build_ms, to_solve_end_ms, to_output_ms);  // total 16
            // log ref data, 5*21
            for (auto ref_state : kin_mpc_debug.ref_kin_state()) {
                fprintf(kin_mpc_log_file_, "%.4f,%.4f,%.4f,%.4f,%.4f,", ref_state.x(), ref_state.y(),
                        ref_state.heading(), ref_state.linear_velocity(), ref_state.steering_angle());
            }
            // log pred data, 5*21
            for (auto pred_state : kin_mpc_debug.pred_kin_state()) {
                fprintf(kin_mpc_log_file_, "%.4f,%.4f,%.4f,%.4f,%.4f,", pred_state.x(), pred_state.y(),
                        pred_state.heading(), pred_state.linear_velocity(), pred_state.steering_angle());
            }
            // log cmd data, 2*20
            for (auto pred_state : kin_mpc_debug.pred_kin_state()) {
                if (pred_state.has_linear_velocity() && pred_state.has_steering_angle_rate()) {
                    fprintf(kin_mpc_log_file_, "%.4f,%.4f,", pred_state.linear_acceleration(),
                            pred_state.steering_angle_rate());
                }
            }
            fprintf(kin_mpc_log_file_, "\n");
            fflush(kin_mpc_log_file_);
        }
    }
}

bool KinMpcController::InitLogFile() {
    auto now_time = std::chrono::system_clock::now();
    auto now_time_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now_time);
    std::time_t now_c = std::chrono::system_clock::to_time_t(now_time_seconds);
    char name_buffer[80];
    strftime(name_buffer, 80, "steer_log_lqr_%Y%m%d%H%M%S.csv", std::localtime(&now_c));

    std::string file_name = FLAGS_pnc_data_log_path + "/control/" + name_buffer;
    kin_mpc_log_file_ = fopen(file_name.c_str(), "w");
    if (kin_mpc_log_file_ == nullptr) {
        return false;
    }
    return true;
}

ControlModeInfo::ControlScenarioMode KinMpcController::RecogniteTrajectoryMode(
    const ADCTrajectory& target_glb_trajectory) {
    ControlModeInfo::ControlScenarioMode mode = ControlModeInfo::NONE;
    int32_t current_trajectory_point_size = target_glb_trajectory.trajectory_point_size();
    pnc::common::TrajectoryPoint last_point = target_glb_trajectory.trajectory_point(current_trajectory_point_size - 1);

    if (target_glb_trajectory.planning_to_control() == pnc::planning::ADCTrajectory_PlanningToControl_PAUSE) {
        mode = ControlModeInfo::TRAJ_PAUSE;
    }

    return mode;
}

// parallel to Solve()
bool KinMpcController::PauseLogic(double speed) {
    if (control_state_matrix_.cols() != time_grid_.size() || control_state_matrix_.cols() < pred_horizon_) {
        LOG_ERROR("Fail to pass size check in PasueLogic");
        return false;
    }

    // DO NOT regenerate matrix
    Interpolation1D::DataType t_pause_delta_interp;

    for (int32_t i = 0; i < static_cast<int32_t>(time_grid_.size()); i++) {
        t_pause_delta_interp.emplace_back(
            std::make_pair(time_grid_.at(i), control_state_matrix_(KinModelStateIndex::kDelta, i)));
    }
    std::unique_ptr<Interpolation1D> tdelta_interpolation = std::make_unique<Interpolation1D>();
    if (!tdelta_interpolation->Init(t_pause_delta_interp)) {
        LOG_ERROR("Fail to init time and delta table for pause");
        return false;
    }
    double resample_delta_time = std::max(0.0, TimeUtility::GetCurrentTimesecond() - cur_solution_time_);
    std::vector<double> resample_delta;
    for (auto time : time_grid_) {
        resample_delta.emplace_back(tdelta_interpolation->LinearInterpolate(resample_delta_time + time));
    }

    double desired_acc = kin_mpc_conf_.acc_for_pause_control();
    if (std::fabs(speed) < standstill_speed_) {
        desired_acc = kin_mpc_conf_.acc_for_standstill();
    }

    int32_t backward_revise = -1;
    if (is_current_backward_) {
        backward_revise = 1;
    }
    for (int32_t i = 0; i < pred_horizon_; ++i) {
        control_cmd_matrix_(KinModelInputIndex::kAcc, i) = backward_revise * std::fabs(desired_acc);
    }
    for (int32_t i = 0; i < pred_horizon_ + 1; ++i) {
        control_state_matrix_(KinModelStateIndex::kDelta, i) = resample_delta.at(i);
    }
    cur_solution_time_ = TimeUtility::GetCurrentTimeMillsecond();

    return true;
}

}  // namespace control
}  // namespace jiduauto
