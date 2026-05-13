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

#include "control/src/controller/dyn_mpc_controller.h"

#include <algorithm>
#include <cstdint>
#include <limits>
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

namespace jiduauto {
namespace control {
using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::math::Clamp;
using jiduauto::pnc::util::TimeUtility;
using pnc::chassis::Chassis;
using pnc::control::ControlCommand;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

DynMpcController::DynMpcController() { LOG_INFO("%s used. ", Name().c_str()); }

DynMpcController::~DynMpcController() { Stop(); }

bool DynMpcController::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("%s begin init.", Name().c_str());
    use_debug_csv_file_ = control_conf.enable_csv_debug();
    control_sample_time_ = control_conf.control_period();
    dyn_mpc_conf_ = control_conf.dyn_mpc_controller_conf();
    vehicle_param_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();
    ts_ = dyn_mpc_conf_.model_sample_time();
    pred_horizon_ = dyn_mpc_conf_.pred_horizon();

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
        Eigen::MatrixXd state_matrix(ModelStateIndex::kSize, 1);
        reference_local_matrixs_.push_back(state_matrix);
    }

    GenerateTimeGrid(dyn_mpc_conf_.model_sample_time());

    controller_initialized_ = true;

    return true;
}

bool DynMpcController::Reset() {
    pre_steer_angle_ = 0.0;
    pre_acc_ = 0.0;

    return true;
}

std::string DynMpcController::Name() const { return "DynMpcController"; }

void DynMpcController::Stop() {
    if (dyn_mpc_log_file_ != nullptr) {
        fclose(dyn_mpc_log_file_);
        dyn_mpc_log_file_ = nullptr;
    }
    log_index_ = 0;
    localization_msg_ptr_ = nullptr;
    chassis_msg_ptr_ = nullptr;
    trajectory_msg_ptr_ = nullptr;
}

bool DynMpcController::MatrixInit() {
    // Matrix init operations.
    // forward
    dyn_osqp_model_.matrix_ad = Eigen::MatrixXd::Zero(ModelStateIndex::kSize, ModelStateIndex::kSize);
    dyn_osqp_model_.matrix_bd = Eigen::MatrixXd::Zero(ModelStateIndex::kSize, ModelInputIndex::kSize);

    dyn_osqp_model_.matrix_state = Eigen::MatrixXd::Zero(ModelStateIndex::kSize, 1);
    dyn_osqp_model_.matrix_gd = Eigen::MatrixXd::Zero(ModelStateIndex::kSize, 1);
    dyn_osqp_model_.matrix_r = Eigen::MatrixXd::Identity(ModelInputIndex::kSize, ModelInputIndex::kSize);
    dyn_osqp_model_.matrix_q = Eigen::MatrixXd::Identity(ModelStateIndex::kSize, ModelStateIndex::kSize);

    // conf init
    // sanity check
    int32_t q_param_size = dyn_mpc_conf_.matrix_q_size();
    if (q_param_size != ModelStateIndex::kSize) {
        LOG_INFO("MPC controller error: matrix_r size: [%d], in parameter file not equal to basic_state_size_: [%d]",
                 q_param_size, ModelStateIndex::kSize);
        return false;
    }

    for (int32_t i = 0; i < q_param_size; ++i) {
        dyn_osqp_model_.matrix_q(i, i) = dyn_mpc_conf_.matrix_q(i);
    }

    int32_t r_param_size = dyn_mpc_conf_.matrix_r_size();
    if (r_param_size != ModelInputIndex::kSize) {
        LOG_ERROR("MPC controller error: matrix_r size: [%f]", r_param_size,
                  " in parameter file not equal to basic_control_size_: [%f]", ModelInputIndex::kSize);
        return false;
    }

    for (int32_t i = 0; i < r_param_size; ++i) {
        dyn_osqp_model_.matrix_r(i, i) = dyn_mpc_conf_.matrix_r(i);
    }

    return true;
}

bool DynMpcController::LoadMPCGainScheduler() {
    const auto& acc_gain_scheduler = dyn_mpc_conf_.acc_gain_scheduler();
    const auto& steer_gain_scheduler = dyn_mpc_conf_.steer_gain_scheduler();

    const auto& x_scheduler = dyn_mpc_conf_.x_scheduler();
    const auto& y_scheduler = dyn_mpc_conf_.y_scheduler();
    const auto& theta_scheduler = dyn_mpc_conf_.theta_scheduler();
    const auto& vx_scheduler = dyn_mpc_conf_.vx_scheduler();
    const auto& vy_scheduler = dyn_mpc_conf_.vy_scheduler();
    const auto& yaw_rate_scheduler = dyn_mpc_conf_.yaw_rate_scheduler();
    const auto& acc_scheduler = dyn_mpc_conf_.acc_scheduler();
    const auto& delta_scheduler = dyn_mpc_conf_.delta_scheduler();

    Interpolation1D::DataType xy1, xy2, xy3, xy4, xy5, xy6, xy7, xy8, xy9, xy10;
    for (const auto& scheduler : acc_gain_scheduler.scheduler()) {
        xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : steer_gain_scheduler.scheduler()) {
        xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : x_scheduler.scheduler()) {
        xy3.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : y_scheduler.scheduler()) {
        xy4.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : theta_scheduler.scheduler()) {
        xy5.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : vx_scheduler.scheduler()) {
        xy6.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : vy_scheduler.scheduler()) {
        xy7.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : yaw_rate_scheduler.scheduler()) {
        xy8.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : acc_scheduler.scheduler()) {
        xy9.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }
    for (const auto& scheduler : delta_scheduler.scheduler()) {
        xy10.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
    }

    if (!acc_control_interpolation_->Init(xy1)) {
        LOG_ERROR("Fail to load acc gain scheduler for MPC controller");
        return false;
    }

    if (!steer_control_interpolation_->Init(xy2)) {
        LOG_ERROR("Fail to load steer gain scheduler for MPC controller");
        return false;
    }
    if (!x_interpolation_->Init(xy3)) {
        LOG_ERROR("Fail to load acc gain scheduler for MPC controller");
        return false;
    }

    if (!y_interpolation_->Init(xy4)) {
        LOG_ERROR("Fail to load steer gain scheduler for MPC controller");
        return false;
    }
    if (!theta_interpolation_->Init(xy5)) {
        LOG_ERROR("Fail to load acc gain scheduler for MPC controller");
        return false;
    }

    if (!vx_interpolation_->Init(xy6)) {
        LOG_ERROR("Fail to load steer gain scheduler for MPC controller");
        return false;
    }
    if (!vy_interpolation_->Init(xy7)) {
        LOG_ERROR("Fail to load acc gain scheduler for MPC controller");
        return false;
    }

    if (!yaw_rate_interpolation_->Init(xy8)) {
        LOG_ERROR("Fail to load steer gain scheduler for MPC controller");
        return false;
    }
    if (!acc_interpolation_->Init(xy9)) {
        LOG_ERROR("Fail to load acc gain scheduler for MPC controller");
        return false;
    }

    if (!delta_interpolation_->Init(xy10)) {
        LOG_ERROR("Fail to load steer gain scheduler for MPC controller");
        return false;
    }

    return true;
}

void DynMpcController::UpdateCurrentState(const DynamicModelState& current_state) {
    dyn_osqp_model_.matrix_state(0, 0) = current_state.X;
    dyn_osqp_model_.matrix_state(1, 0) = current_state.Y;
    dyn_osqp_model_.matrix_state(2, 0) = current_state.theta;
    dyn_osqp_model_.matrix_state(3, 0) = current_state.vx;
    dyn_osqp_model_.matrix_state(4, 0) = current_state.vy;
    dyn_osqp_model_.matrix_state(5, 0) = current_state.yaw_rate;
    dyn_osqp_model_.matrix_state(6, 0) = current_state.acc;
    dyn_osqp_model_.matrix_state(7, 0) = current_state.delta;
}

bool DynMpcController::Solve() {
    control_cmd_matrix_ = Eigen::MatrixXd::Zero(ModelInputIndex::kSize, pred_horizon_);
    control_state_matrix_ = Eigen::MatrixXd::Zero(ModelStateIndex::kSize, pred_horizon_ + 1);

    const std::vector<DynamicModelInput> input_bounds = GenerateInputBound();
    const std::vector<DynamicModelState> state_bounds = GenerateStateBound();

    const Eigen::MatrixXd lower_state_bound = StateToMatrix(state_bounds[0]);
    const Eigen::MatrixXd upper_state_bound = StateToMatrix(state_bounds[1]);
    const Eigen::MatrixXd lower_input_bound = InputToMatrix(input_bounds[0]);
    const Eigen::MatrixXd upper_input_bound = InputToMatrix(input_bounds[1]);

    // one gain cmd, with two state cmd
    std::vector<double> control_cmds(ModelInputIndex::kSize * pred_horizon_, 0);
    std::vector<double> control_states(ModelStateIndex::kSize * (pred_horizon_ + 1), 0);

    MpcOSQPSolver mpc_dyn_solver(dyn_osqp_model_.matrix_ad, dyn_osqp_model_.matrix_bd, dyn_osqp_model_.matrix_q,
                                 dyn_osqp_model_.matrix_r, dyn_osqp_model_.matrix_gd, dyn_osqp_model_.matrix_state,
                                 lower_input_bound, upper_input_bound, lower_state_bound, upper_state_bound,
                                 reference_local_matrixs_, dyn_mpc_conf_.max_iteration(), pred_horizon_,
                                 dyn_mpc_conf_.eps());

    if (!mpc_dyn_solver.Execute(&control_cmds, &control_states)) {
        LOG_ERROR("MPC OSQP solver failed");
        return false;
    }

    for (int32_t i = 0; i < pred_horizon_; ++i) {
        control_cmd_matrix_(ModelInputIndex::kAccRate, i) =
            control_cmds.at(i * ModelInputIndex::kSize + ModelInputIndex::kAccRate);
        control_cmd_matrix_(ModelInputIndex::kDeltaRate, i) =
            control_cmds.at(i * ModelInputIndex::kSize + ModelInputIndex::kDeltaRate);
    }
    for (int32_t i = 0; i < (pred_horizon_ + 1); ++i) {
        control_state_matrix_(ModelStateIndex::kX, i) =
            control_states.at(i * ModelStateIndex::kSize + ModelStateIndex::kX);
        control_state_matrix_(ModelStateIndex::kY, i) =
            control_states.at(i * ModelStateIndex::kSize + ModelStateIndex::kY);
        control_state_matrix_(ModelStateIndex::kTheta, i) =
            control_states.at(i * ModelStateIndex::kSize + ModelStateIndex::kTheta);
        control_state_matrix_(ModelStateIndex::kVx, i) =
            control_states.at(i * ModelStateIndex::kSize + ModelStateIndex::kVx);
        control_state_matrix_(ModelStateIndex::kVy, i) =
            control_states.at(i * ModelStateIndex::kSize + ModelStateIndex::kVy);
        control_state_matrix_(ModelStateIndex::kYawRate, i) =
            control_states.at(i * ModelStateIndex::kSize + ModelStateIndex::kYawRate);
        control_state_matrix_(ModelStateIndex::kAcc, i) =
            control_states.at(i * ModelStateIndex::kSize + ModelStateIndex::kAcc);
        control_state_matrix_(ModelStateIndex::kDelta, i) =
            control_states.at(i * ModelStateIndex::kSize + ModelStateIndex::kDelta);
    }

    return true;
}

void DynMpcController::UpdateStateMatrix(const LinModelMatrix& linear_dynamic_model) {
    dyn_osqp_model_.matrix_ad = linear_dynamic_model.A;
}

void DynMpcController::UpdateInputMatrix(const LinModelMatrix& linear_dynamic_model) {
    dyn_osqp_model_.matrix_bd = linear_dynamic_model.B;
}

void DynMpcController::UpdateLinearizationMatrix(const LinModelMatrix& linear_dynamic_model) {
    dyn_osqp_model_.matrix_gd = linear_dynamic_model.g;
}

const std::vector<DynamicModelState> DynMpcController::UpdateLocalReferenceState(
    const ADCTrajectory* target_glb_trajectory_ptr, const DynamicModelState& target_state,
    const DynamicModelState& current_state) {
    std::vector<DynamicModelState> reference_local_state_seq;

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

    if (fail_to_init_interpolation) {
        return reference_local_state_seq;
    }

    for (auto time : time_grid_) {
        DynamicModelState reference_local_state;
        reference_local_state.setZero();
        reference_local_state.X = tx_interpolation->LinearInterExtrapolate(time);
        reference_local_state.Y = ty_interpolation->LinearInterExtrapolate(time);
        reference_local_state.theta = ttheta_interpolation->LinearInterExtrapolate(time);
        reference_local_state.vx = tspeed_interpolation->LinearInterExtrapolate(time);
        // TODO(zhaobolin): could add other required states
        reference_local_state_seq.push_back(reference_local_state);
    }
    return reference_local_state_seq;
}

// no trajectory mode
const std::vector<DynamicModelState> DynMpcController::UpdateLocalReferenceState(
    const DynamicModelState& target_state, const DynamicModelState& current_state) {
    std::vector<DynamicModelState> reference_single_local_state_seq;
    for (int32_t i = 0; i < static_cast<int32_t>(time_grid_.size()); ++i) {
        reference_single_local_state_seq.push_back(target_state);
    }
    return reference_single_local_state_seq;
}

void DynMpcController::GenerateTimeGrid(double step_gain) {
    double last_grid = 0.0;
    if (dyn_mpc_conf_.enable_augmented_timegrid()) {
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
Eigen::MatrixXd DynMpcController::StateToMatrix(const DynamicModelState& mpc_state) {
    Eigen::MatrixXd state_matrix(ModelStateIndex::kSize, 1);
    state_matrix(ModelStateIndex::kX) = mpc_state.X;
    state_matrix(ModelStateIndex::kY) = mpc_state.Y;
    state_matrix(ModelStateIndex::kTheta) = mpc_state.theta;
    state_matrix(ModelStateIndex::kVx) = mpc_state.vx;
    state_matrix(ModelStateIndex::kVy) = mpc_state.vy;
    state_matrix(ModelStateIndex::kYawRate) = mpc_state.yaw_rate;
    state_matrix(ModelStateIndex::kAcc) = mpc_state.acc;
    state_matrix(ModelStateIndex::kDelta) = mpc_state.delta;

    return state_matrix;
}

Eigen::MatrixXd DynMpcController::InputToMatrix(const DynamicModelInput& mpc_input) {
    Eigen::MatrixXd input_matrix(ModelInputIndex::kSize, 1);
    input_matrix(ModelInputIndex::kAccRate) = mpc_input.d_acc;
    input_matrix(ModelInputIndex::kDeltaRate) = mpc_input.d_delta;

    return input_matrix;
}

DynamicModelState DynMpcController::MatrixToState(const Eigen::MatrixXd& state_matrix) {
    DynamicModelState mpc_state;
    mpc_state.X = state_matrix(ModelStateIndex::kX);
    mpc_state.Y = state_matrix(ModelStateIndex::kY);
    mpc_state.theta = state_matrix(ModelStateIndex::kTheta);
    mpc_state.vx = state_matrix(ModelStateIndex::kVx);
    mpc_state.vy = state_matrix(ModelStateIndex::kVy);
    mpc_state.yaw_rate = state_matrix(ModelStateIndex::kYawRate);
    mpc_state.acc = state_matrix(ModelStateIndex::kAcc);
    mpc_state.delta = state_matrix(ModelStateIndex::kDelta);

    return mpc_state;
}

DynamicModelInput DynMpcController::MatrixToInput(const Eigen::MatrixXd& input_matrix) {
    DynamicModelInput mpc_input;
    mpc_input.d_acc = input_matrix(ModelInputIndex::kAccRate);
    mpc_input.d_delta = input_matrix(ModelInputIndex::kDeltaRate);

    return mpc_input;
}

const std::vector<DynamicModelInput> DynMpcController::GenerateInputBound() const {
    std::vector<DynamicModelInput> input_bounds;

    double min_accel_dot = -10.0;  // m/s^3  // TODO(bolin): fill those config in the conf
    double max_accel_dot = 10.0;   // m/s^3
    double max_angle_dot = 2 * 3.14 / vehicle_param_.steer_ratio();
    // 2*3.14 rad/s for steering wheel speed

    double min_delta_accel = min_accel_dot * 1.0;  // TODO(bolin): check this
    double max_delta_accel = max_accel_dot * 1.0;
    double max_delta_angle = max_angle_dot * 1.0;

    DynamicModelInput lower_input_bound;
    lower_input_bound.d_acc = min_delta_accel;
    lower_input_bound.d_delta = -max_delta_angle;
    input_bounds.push_back(lower_input_bound);

    DynamicModelInput upper_input_bound;
    upper_input_bound.d_acc = max_delta_accel;
    upper_input_bound.d_delta = max_delta_angle;
    input_bounds.push_back(upper_input_bound);

    return input_bounds;
}

const std::vector<DynamicModelState> DynMpcController::GenerateStateBound() const {
    std::vector<DynamicModelState> state_bounds;
    const double max = std::numeric_limits<double>::max();

    DynamicModelState lower_state_bound;
    lower_state_bound.X = -1.0 * max;
    lower_state_bound.Y = -1.0 * max;
    lower_state_bound.theta = -1.0 * max;
    lower_state_bound.vx = -1.0 * max;
    lower_state_bound.vy = -1.0 * max;
    lower_state_bound.yaw_rate = -1.0 * max;
    lower_state_bound.acc = -6.0;  // TODO(bolin): fill those config in the conf
    lower_state_bound.delta = -0.5;
    state_bounds.push_back(lower_state_bound);

    DynamicModelState upper_state_bound;
    upper_state_bound.X = max;
    upper_state_bound.Y = max;
    upper_state_bound.theta = max;
    upper_state_bound.vx = max;
    upper_state_bound.vy = max;
    upper_state_bound.yaw_rate = max;
    upper_state_bound.acc = 3.0;  // TODO(bolin): fill those config in the conf
    upper_state_bound.delta = 0.5;
    state_bounds.push_back(upper_state_bound);

    return state_bounds;
}

void DynMpcController::OutputCommandProcess(const DynamicModelState& current_mpc_state) {
    double steer_angle_target;
    double control_desicred_acc;
    double delta_steer_angle_target;
    double delta_acc;

    if (std::isnan(control_cmd_matrix_(0, 0)) || std::isnan(control_cmd_matrix_(1, 0))) {
        steer_angle_target = current_mpc_state.delta;  // current angle
        delta_steer_angle_target = pre_delta_steer_angle_;
        control_desicred_acc = current_mpc_state.acc;  // current acc
        delta_acc = last_measured_acc_;
        LOG_WARN("NaN in cmd, keep last cmd;");
    } else {
        if (dyn_mpc_conf_.use_rate_as_gain()) {
            delta_steer_angle_target = control_cmd_matrix_(ModelInputIndex::kDeltaRate, 0) / ts_ * control_sample_time_;
            steer_angle_target = last_measured_delta_ + delta_steer_angle_target;
            LOG_INFO("last_measured_delta_ is %f", last_measured_delta_);
            steer_angle_target = Clamp(steer_angle_target, -0.5, 0.5);  // TODO(bolin): fill those config in the conf

            delta_acc = Clamp(control_cmd_matrix_(ModelInputIndex::kAccRate, 0), -10.0, 10.0) / ts_ *
                        control_sample_time_;  // TODO(bolin): fill those config in the conf
            control_desicred_acc = last_measured_acc_ + delta_acc;
            control_desicred_acc = Clamp(control_desicred_acc, -6.0, 3.0);
        } else {
            steer_angle_target = control_state_matrix_(ModelStateIndex::kDelta, 1);
            control_desicred_acc = control_state_matrix_(ModelStateIndex::kAcc, 1);
        }
    }

    LOG_INFO("control_cmd_matrix_(0, 0) %f", control_cmd_matrix_(0, 0));
    LOG_INFO("control_cmd_matrix_(1, 0) %f", control_cmd_matrix_(1, 0));
    LOG_INFO("control_state_matrix_(ModelStateIndex::kAcc 1) %f", control_state_matrix_(ModelStateIndex::kAcc, 1));
    LOG_INFO("control_state_matrix_(ModelStateIndex::kDelta, 1) %f", control_state_matrix_(ModelStateIndex::kDelta, 1));

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
    LOG_INFO("steer_angle_target rad %f", steer_angle_target);

    command_msg_ptr_->set_acceleration(control_desicred_acc);
    command_msg_ptr_->set_throttle(throttle_cmd);
    command_msg_ptr_->set_brake(brake_cmd);

    LOG_INFO("dyn_mpc cmd STR TGT rad: [%f]", steer_angle_target);
    LOG_INFO("dyn_mpc cmd ACC TGT: [%f]", control_desicred_acc);
    LOG_INFO("dyn_mpc cmd THR TGT: [%f]", throttle_cmd);
    LOG_INFO("dyn_mpc cmd BRK TGT: [%f]", brake_cmd);

    pre_steer_angle_ = steer_angle_target;
    pre_acc_ = control_desicred_acc;

    pre_delta_steer_angle_ = delta_steer_angle_target;
    pre_delta_acc_ = delta_acc;
}

bool DynMpcController::Control(const pnc::localization::LocalizationEstimate* const localization,
                               const pnc::chassis::Chassis* const chassis,
                               const pnc::planning::ADCTrajectory* const trajectory,
                               pnc::control::ControlCommand* const cmd) {
    return false;
}

bool DynMpcController::ControlMain(const ControlInputStream& control_input_stream,
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

    if (chassis_msg_ptr_->driving_mode() != pnc::chassis::COMPLETE_MANUAL &&
        ControlMath::GearIsReady(localization_msg_ptr_, chassis_msg_ptr_, trajectory_msg_ptr_)) {
        is_current_auto_mode_ = true;
    } else {
        is_current_auto_mode_ = false;
    }

    bool curr_backward = ControlMath::CheckBackward(chassis_msg_ptr_);

    if (curr_backward != is_current_backward_) {
        is_current_backward_ = curr_backward;
        Reset();
        LOG_INFO("Reset()");
    }

    DynamicModelState current_local_state;
    current_local_state.X = 0;
    current_local_state.Y = 0;
    current_local_state.theta = 0;

    // switch between localization speed or chassis speed
    if (FLAGS_enable_using_localization_velocity) {
        // TODO(zhaobolin): (review)
        // should control and planning use the same speed resource?
        const auto& speed3d = localization_msg_ptr_->loc_pose().pose().linear_velocity();
        current_local_state.vx = std::hypot(speed3d.x(), speed3d.y());
    } else {
        LOG_ERROR("chassis spd %f", chassis_msg_ptr_->speed_mps());
        current_local_state.vx = chassis_msg_ptr_->speed_mps();
    }
    // TODO(zhaobolin): move to conf
    current_local_state.vx = std::max(current_local_state.vx, dyn_mpc_conf_.protect_low_speed());

    // TODO(zhaobolin): use get_lateral_spd instead
    current_local_state.vy = 0.0;

    current_local_state.yaw_rate = localization_msg_ptr_->loc_pose().pose().angular_velocity().z();

    current_local_state.acc = chassis_msg_ptr_->chassis_lon_acc();

    current_local_state.delta = chassis_msg_ptr_->steering_percentage() / 100.0 * vehicle_param_.max_steer_angle() /
                                vehicle_param_.steer_ratio();  // command

    UpdateCurrentState(current_local_state);
    DynamicModelInput current_cmd;  // TODO(zhaobolin): should from last input
    current_cmd.d_acc = current_local_state.acc - last_measured_acc_;
    current_cmd.d_delta = current_local_state.delta - last_measured_delta_;
    last_measured_delta_ = current_local_state.delta;
    last_measured_acc_ = current_local_state.acc;

    // Dynamic Model Iteration
    Param model_param;                             // should get from init process, temporary init here
    DynamicModel dynamic_model(ts_, model_param);  // a sample-updated model

    LOG_INFO("start getLinModel ");
    LinModelMatrix current_linear_model = dynamic_model.getLinModel(current_local_state, current_cmd);

    // std::cout << "current_linear_model Ad " << current_linear_model.A << std::endl
    // std::cout << "current_linear_model Bd " << current_linear_model.B << std::endl
    // std::cout << "current_linear_model Gd " << current_linear_model.g << std::endl

    UpdateStateMatrix(current_linear_model);
    UpdateInputMatrix(current_linear_model);
    UpdateLinearizationMatrix(current_linear_model);

    DynamicModelState target_drift_state;  // TODO(zhaobolin): fill in the static dirft state
    target_drift_state.setZero();
    // one point target (no cost on zero term)
    target_drift_state.vx = dyn_mpc_conf_.target_drift_state().linear_velocity();
    target_drift_state.vy = dyn_mpc_conf_.target_drift_state().linear_lateral_velocity();
    target_drift_state.yaw_rate = dyn_mpc_conf_.target_drift_state().yaw_rate();

    std::vector<DynamicModelState> reference_local_states;
    if (dyn_mpc_conf_.enable_single_point_mode() == true) {
        LOG_INFO("In single point drift mode;");
        reference_local_states = UpdateLocalReferenceState(target_drift_state, current_local_state);
    } else {
        LOG_INFO("In trajectory following mode;");
        reference_local_states =
            UpdateLocalReferenceState(trajectory_msg_ptr_, target_drift_state, current_local_state);
    }
    LOG_INFO("reference states length %d", reference_local_states.size());

    if (reference_local_states.size() != pred_horizon_ + 1) {
        return false;
    }  // TODO(zhaobolin): should be handled in input check

    reference_local_matrixs_.clear();
    for (int32_t i = 0; i < pred_horizon_ + 1; ++i) {
        reference_local_matrixs_.emplace_back(StateToMatrix(reference_local_states.at(i)));
    }

    // TODO(zhaobolin): A B matrix with any inf should jump
    if (!Solve()) {
        LOG_ERROR("MPC Solved failed.");
        return false;
    }

    std::vector<DynamicModelState> pred_states;
    for (int32_t i = 0; i < pred_horizon_ + 1; ++i) {
        pred_states.emplace_back(MatrixToState(control_state_matrix_.col(i)));
    }
    std::vector<DynamicModelInput> cmd_seqs;
    for (int32_t i = 0; i < pred_horizon_; ++i) {
        cmd_seqs.emplace_back(MatrixToInput(control_cmd_matrix_.col(i)));
    }

    OutputCommandProcess(current_local_state);

    int32_t main_total_time = static_cast<int32_t>(TimeUtility::GetCurrentTimeMillsecond() - main_start_time);

    SummaryDebugInfo(current_local_state, current_cmd, reference_local_states, pred_states, cmd_seqs, time_grid_,
                     main_total_time, &control_command_stream->control_debug);
    LogDebugFile(control_input_stream, control_command_stream->control_debug.dynamic_mpc_debug());

    LOG_INFO("Finish MPC DYN ControlMain;");
    return true;
}

const double DynMpcController::GetLateralSpeed(const pnc::localization::LocalizationEstimate& localization) {
    double glb_vx = localization.loc_pose().pose().linear_velocity().x();
    double glb_vy = localization.loc_pose().pose().linear_velocity().y();
    auto& orientation = localization.loc_pose().pose().orientation();
    EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
    double heading = q.yaw();
    double vy_vrf = (-glb_vx * std::sin(heading) + glb_vy * std::cos(heading));
    if (std::fabs(vy_vrf) < 0.01) {
        vy_vrf = 0.0;
    }
    LOG_INFO("lateral speed is %f", vy_vrf);
    return vy_vrf;
}

bool DynMpcController::SummaryDebugInfo(const DynamicModelState& current_mpc_state,
                                        const DynamicModelInput& current_mpc_input,
                                        const std::vector<DynamicModelState>& reference_states,
                                        const std::vector<DynamicModelState>& pred_states,
                                        const std::vector<DynamicModelInput>& cmd_seqs,
                                        const std::vector<double>& time_grid, const int32_t main_cost_time,
                                        pnc::control::ControlDebug* const control_debug) const {
    auto* dynamic_debug = control_debug->mutable_dynamic_mpc_debug();
    dynamic_debug->set_enable_controller(true);
    dynamic_debug->set_cost_time_ms(main_cost_time);
    dynamic_debug->set_enable_single_state_mode(dyn_mpc_conf_.enable_single_point_mode());
    dynamic_debug->set_use_rate_as_gain(dyn_mpc_conf_.use_rate_as_gain());
    for (int32_t i = 0; i < dyn_mpc_conf_.matrix_q().size(); ++i) {
        dynamic_debug->add_final_q(dyn_osqp_model_.matrix_q(i, i));
    }
    for (int32_t i = 0; i < dyn_mpc_conf_.matrix_r().size(); ++i) {
        dynamic_debug->add_final_r(dyn_osqp_model_.matrix_r(i, i));
    }

    pnc::common::VehicleState* current_state = dynamic_debug->mutable_current_state();
    current_state->set_x(current_mpc_state.X);
    current_state->set_y(current_mpc_state.Y);
    current_state->set_heading(current_mpc_state.theta);
    current_state->set_linear_velocity(current_mpc_state.vx);
    current_state->set_linear_lateral_velocity(current_mpc_state.vy);
    current_state->set_yaw_rate(current_mpc_state.yaw_rate);
    current_state->set_linear_acceleration(current_mpc_state.acc);
    current_state->set_steering_angle(current_mpc_state.delta);
    current_state->set_acc_gain(current_mpc_input.d_acc);
    current_state->set_steering_angle_rate(current_mpc_input.d_delta);

    int32_t ref_index = 0;
    for (const DynamicModelState& state : reference_states) {
        pnc::common::VehicleState* dyn_state = dynamic_debug->add_ref_dyn_state();
        dyn_state->set_index(ref_index);
        dyn_state->set_timestamp_sec(time_grid.at(ref_index));
        ref_index++;
        dyn_state->set_x(state.X);
        dyn_state->set_y(state.Y);
        dyn_state->set_heading(state.theta);
        dyn_state->set_linear_velocity(state.vx);
        dyn_state->set_linear_lateral_velocity(state.vy);
        dyn_state->set_yaw_rate(state.yaw_rate);
        dyn_state->set_linear_acceleration(state.acc);
        dyn_state->set_steering_angle(state.delta);
    }
    int32_t pred_index = 0;
    for (const DynamicModelState& pred_state : pred_states) {
        pnc::common::VehicleState* pred_dyn_state = dynamic_debug->add_pred_dyn_state();
        pred_dyn_state->set_index(pred_index);
        pred_dyn_state->set_timestamp_sec(time_grid.at(pred_index));
        pred_index++;
        pred_dyn_state->set_x(pred_state.X);
        pred_dyn_state->set_y(pred_state.Y);
        pred_dyn_state->set_heading(pred_state.theta);
        pred_dyn_state->set_linear_velocity(pred_state.vx);
        pred_dyn_state->set_linear_lateral_velocity(pred_state.vy);
        pred_dyn_state->set_yaw_rate(pred_state.yaw_rate);
        pred_dyn_state->set_linear_acceleration(pred_state.acc);
        pred_dyn_state->set_steering_angle(pred_state.delta);
        if (pred_index <= pred_horizon_) {
            pred_dyn_state->set_acc_gain(cmd_seqs[pred_index].d_acc);
            pred_dyn_state->set_steering_angle_rate(cmd_seqs[pred_index].d_delta);
        }
    }
    for (double time : time_grid) {
        dynamic_debug->add_time_grid(time);
    }

    dynamic_debug->set_acc_cmd(command_msg_ptr_->acceleration());
    dynamic_debug->set_steer_wheel_cmd_deg(command_msg_ptr_->steering_target());
    dynamic_debug->set_steer_cmd_rad(pre_steer_angle_);
    dynamic_debug->set_acc_gain_rate(pre_delta_acc_ / ts_);
    dynamic_debug->set_steer_cmd_gain_rate(pre_delta_steer_angle_ / ts_);

    return true;
}

bool DynMpcController::InitLogFile() {
    time_t rawtime;
    char name_buffer[80];
    time(&rawtime);
    struct tm now_time;
    localtime_r(&rawtime, &now_time);
    strftime(name_buffer, 80, "dyn_mpc_log_%F_%H%M%S.csv", &now_time);
    std::string file_name = FLAGS_pnc_data_log_path + "/control/" + name_buffer;
    dyn_mpc_log_file_ = fopen(file_name.c_str(), "w");
    if (dyn_mpc_log_file_ == nullptr) {
        return false;
    }
    return true;
}

void DynMpcController::LogDebugFile(const ControlInputStream& control_input_stream,
                                    const pnc::control::DynamicMpcDebug& dyn_mpc_debug) {
    if (use_debug_csv_file_) {
        double current_loc_time = control_input_stream.vehicle_state.timestamp_sec();
        double pos_glb_x = control_input_stream.vehicle_state.x();
        double pos_glb_y = control_input_stream.vehicle_state.y();
        double pos_glb_phi = control_input_stream.vehicle_state.heading();
        double chassis_spd = control_input_stream.vehicle_state.linear_velocity();
        double yaw_rate = control_input_stream.vehicle_state.angular_velocity();
        double chassis_acc = control_input_stream.vehicle_state.linear_acceleration();

        int compute_time_ms = dyn_mpc_debug.cost_time_ms();
        // double to_update_cur_state_ms = dyn_mpc_debug.to_update_cur_state_ms();
        // double to_matrix_build_ms = dyn_mpc_debug.to_matrix_build_ms();
        // double to_solve_end_ms = dyn_mpc_debug.to_solve_end_ms();
        // double to_output_ms = dyn_mpc_debug.to_output_ms();

        double acc_cmd = dyn_mpc_debug.acc_cmd();
        double steer_wheel_cmd_deg = dyn_mpc_debug.steer_wheel_cmd_deg();

        if (!(dyn_mpc_log_file_ == nullptr && !InitLogFile())) {
            fprintf(dyn_mpc_log_file_, "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,", is_current_auto_mode_,
                    current_loc_time, pos_glb_x, pos_glb_y, pos_glb_phi, chassis_spd, yaw_rate, chassis_acc,
                    compute_time_ms, acc_cmd, steer_wheel_cmd_deg);
            // total 11

            fprintf(dyn_mpc_log_file_, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,",
                    dyn_mpc_debug.current_state().x(), dyn_mpc_debug.current_state().y(),
                    dyn_mpc_debug.current_state().heading(), dyn_mpc_debug.current_state().linear_velocity(),
                    dyn_mpc_debug.current_state().linear_lateral_velocity(), dyn_mpc_debug.current_state().yaw_rate(),
                    dyn_mpc_debug.current_state().linear_acceleration(), dyn_mpc_debug.current_state().steering_angle(),
                    dyn_mpc_debug.current_state().acc_gain(), dyn_mpc_debug.current_state().steering_angle_rate());
            // total 10

            for (auto ref_state : dyn_mpc_debug.ref_dyn_state()) {
                fprintf(dyn_mpc_log_file_, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,", ref_state.x(), ref_state.y(),
                        ref_state.heading(), ref_state.linear_velocity(), ref_state.linear_lateral_velocity(),
                        ref_state.yaw_rate(), ref_state.linear_acceleration(), ref_state.steering_angle());
            }

            for (auto pred_state : dyn_mpc_debug.pred_dyn_state()) {
                fprintf(dyn_mpc_log_file_, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,", pred_state.x(), pred_state.y(),
                        pred_state.heading(), pred_state.linear_velocity(), pred_state.linear_lateral_velocity(),
                        pred_state.yaw_rate(), pred_state.linear_acceleration(), pred_state.steering_angle());
            }

            for (auto pred_state : dyn_mpc_debug.pred_dyn_state()) {
                if (pred_state.has_acc_gain() && pred_state.has_steering_angle_rate()) {
                    fprintf(dyn_mpc_log_file_, "%.4f,%.4f,", pred_state.acc_gain(), pred_state.steering_angle_rate());
                }
            }
            fprintf(dyn_mpc_log_file_, "\n");
            fflush(dyn_mpc_log_file_);
        }
    }
}

}  // namespace control
}  // namespace jiduauto
