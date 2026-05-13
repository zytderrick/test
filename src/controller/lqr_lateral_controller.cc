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

#include "control/src/controller/lqr_lateral_controller.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "control/src/common/control_gflags.h"
#include "control/src/math/control_math.h"
#include "control/src/math/trajectory_analyzer.h"
#include "pnc_common/src/common/pnc_gflag.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::math::Clamp;
using pnc::chassis::Chassis;
using pnc::control::ControlCommand;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

using Mtx = Eigen::MatrixXd;

LqrLateralController::LqrLateralController() { LOG_INFO("%s used. ", Name().c_str()); }

LqrLateralController::~LqrLateralController() { Stop(); }

bool LqrLateralController::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("%s begin init.", Name().c_str());
    use_debug_csv_file_ = control_conf.enable_csv_debug();
    lqr_conf_ = control_conf.lqr_controller_conf();
    ts_ = control_conf.control_period();
    lat_paras_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();

    if (!MatrixInit()) {
        LOG_ERROR("MatrixInit failed");
        return false;
    }

    // digital filter
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    ControlMath::GetLpfCoefficient(ts_, lqr_conf_.cutoff_freq(), &den, &num);
    steer_angle_filter_.SetCoefficients(den, num);
    lateral_error_filter_.SetWindowSize(lqr_conf_.mean_filter_window_size());
    lateral_rate_filter_.SetWindowSize(lqr_conf_.mean_filter_window_size());
    heading_error_filter_.SetWindowSize(lqr_conf_.mean_filter_window_size());
    heading_rate_filter_.SetWindowSize(lqr_conf_.mean_filter_window_size());
    curvature_filter_.SetWindowSize(lqr_conf_.mean_filter_window_size());

    // lateral error and heading error, already init...
    // lqr output, already init...

    controller_initialized_ = true;

    return true;
}

bool LqrLateralController::Reset() {
    previous_heading_error_ = 0.0;
    previous_lateral_error_ = 0.0;
    steer_angle_filter_.Reset();
    lateral_error_filter_.Reset();
    lateral_rate_filter_.Reset();
    heading_error_filter_.Reset();
    heading_rate_filter_.Reset();
    curvature_filter_.Reset();
    return true;
}

std::string LqrLateralController::Name() const { return "LqrLateralController"; }

void LqrLateralController::Stop() {
    if (steer_log_file_ != nullptr) {
        fclose(steer_log_file_);
        steer_log_file_ = nullptr;
    }
    log_index_ = 0;
    localization_msg_ = nullptr;
    chassis_msg_ = nullptr;
    trajectory_msg_ = nullptr;
}

bool LqrLateralController::MatrixInit() {
    const auto& dyn_paras = lat_paras_.vehicle_dynamics_paras();
    const auto& stiffness = lat_paras_.tire_cornering_stiffness();

    // Matrix init operations.
    // forward
    /*[0.0, 1.0, 0.0, 0.0;
       0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
       (l_r * c_r - l_f * c_f) / m / v;
       0.0, 0.0, 0.0, 1.0;
       0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
       (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]*/
    lqr_matrix_.matrix_a = Mtx::Zero(STATE_SIZE, STATE_SIZE);
    lqr_matrix_.matrix_ad = Mtx::Zero(STATE_SIZE, STATE_SIZE);
    lqr_matrix_.matrix_adc = Mtx::Zero(STATE_SIZE, STATE_SIZE);
    lqr_matrix_.matrix_a(0, 1) = 1.0;
    lqr_matrix_.matrix_a(1, 2) = (stiffness.cf() + stiffness.cr()) / dyn_paras.mass();
    lqr_matrix_.matrix_a(2, 3) = 1.0;
    lqr_matrix_.matrix_a(3, 2) = (dyn_paras.lf() * stiffness.cf() - dyn_paras.lr() * stiffness.cr()) / dyn_paras.iz();

    lqr_matrix_.matrix_a_coeff = Mtx::Zero(STATE_SIZE, STATE_SIZE);
    lqr_matrix_.matrix_a_coeff(1, 1) = -(stiffness.cf() + stiffness.cr()) / dyn_paras.mass();
    lqr_matrix_.matrix_a_coeff(1, 3) =
        (dyn_paras.lr() * stiffness.cr() - dyn_paras.lf() * stiffness.cf()) / dyn_paras.mass();

    lqr_matrix_.matrix_a_coeff(3, 1) =
        (dyn_paras.lr() * stiffness.cr() - dyn_paras.lf() * stiffness.cf()) / dyn_paras.iz();
    lqr_matrix_.matrix_a_coeff(3, 3) =
        -1.0 * (dyn_paras.lf() * dyn_paras.lf() * stiffness.cf() + dyn_paras.lr() * dyn_paras.lr() * stiffness.cr()) /
        dyn_paras.iz();

    /* b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T */
    lqr_matrix_.matrix_b = Mtx::Zero(STATE_SIZE, 1);
    lqr_matrix_.matrix_bd = Mtx::Zero(STATE_SIZE, 1);
    lqr_matrix_.matrix_bdc = Mtx::Zero(STATE_SIZE, 1);
    lqr_matrix_.matrix_b(1, 0) = stiffness.cf() / dyn_paras.mass();
    lqr_matrix_.matrix_b(3, 0) = dyn_paras.lf() * stiffness.cf() / dyn_paras.iz();
    lqr_matrix_.matrix_bd = lqr_matrix_.matrix_b * ts_;

    lqr_matrix_.matrix_state = Mtx::Zero(STATE_SIZE, 1);
    lqr_matrix_.matrix_k = Mtx::Zero(1, STATE_SIZE);
    lqr_matrix_.matrix_r = Mtx::Identity(1, 1);
    lqr_matrix_.matrix_q = Mtx::Zero(STATE_SIZE, STATE_SIZE);

    // backward
    // we use |v| as v;
    lqr_matrix_.backward_matrix_a = lqr_matrix_.matrix_a;
    lqr_matrix_.backward_matrix_ad = lqr_matrix_.matrix_ad;
    lqr_matrix_.backward_matrix_adc = lqr_matrix_.matrix_adc;
    lqr_matrix_.backward_matrix_a_coeff = lqr_matrix_.matrix_a_coeff;
    lqr_matrix_.backward_matrix_a(1, 2) = -lqr_matrix_.backward_matrix_a(1, 2);
    lqr_matrix_.backward_matrix_a(3, 2) = -lqr_matrix_.backward_matrix_a(3, 2);

    lqr_matrix_.backward_matrix_b = lqr_matrix_.matrix_b;
    lqr_matrix_.backward_matrix_b(1, 0) = -lqr_matrix_.backward_matrix_b(1, 0);
    lqr_matrix_.backward_matrix_b(3, 0) = -lqr_matrix_.backward_matrix_b(3, 0);
    lqr_matrix_.backward_matrix_bd = lqr_matrix_.backward_matrix_b * ts_;
    lqr_matrix_.backward_matrix_bdc = Mtx::Zero(STATE_SIZE, 1);

    // conf init
    // sanity check
    for (int32_t i = 0; i < lqr_conf_.backward_paras_size(); ++i) {
        if (STATE_SIZE != lqr_conf_.backward_paras(i).matrix_q_size()) {
            LOG_ERROR(
                "STATE_SIZE[%d] != "
                "lqr_conf_.backward_paras(%d).matrix_q_size(%d)",
                STATE_SIZE, i, lqr_conf_.backward_paras(i).matrix_q_size());
            return false;
        }
        if (lqr_conf_.backward_paras(i).matrix_r_size() != 1) {
            LOG_ERROR("lqr_conf_.backward_paras(%d).matrix_r_size(%d) != 1", i,
                      lqr_conf_.backward_paras(i).matrix_r_size());
            return false;
        }
    }

    if (STATE_SIZE != lqr_conf_.parking_paras().matrix_q_size()) {
        LOG_ERROR(
            "STATE_SIZE[%d] != "
            "lqr_conf_.parking_paras().matrix_q_size(%d)",
            STATE_SIZE, lqr_conf_.parking_paras().matrix_q_size());
        return false;
    }
    if (lqr_conf_.parking_paras().matrix_r_size() != 1) {
        LOG_ERROR("lqr_conf_.parking_paras().matrix_r_size(%d) != 1", lqr_conf_.parking_paras().matrix_r_size());
        return false;
    }

    if (STATE_SIZE != lqr_conf_.parking_backward_paras().matrix_q_size()) {
        LOG_ERROR(
            "STATE_SIZE[%d] != "
            "lqr_conf_.parking_backward_paras().matrix_q_size(%d)",
            STATE_SIZE, lqr_conf_.parking_backward_paras().matrix_q_size());
        return false;
    }
    if (lqr_conf_.parking_backward_paras().matrix_r_size() != 1) {
        LOG_ERROR("lqr_conf_.parking_backward_paras().matrix_r_size(%d) != 1",
                  lqr_conf_.parking_backward_paras().matrix_r_size());
        return false;
    }

    for (int32_t i = 0; i < lqr_conf_.forward_paras_size(); ++i) {
        if (STATE_SIZE != lqr_conf_.forward_paras(i).matrix_q_size()) {
            LOG_ERROR(
                "STATE_SIZE[%d] != "
                "lqr_conf_.forward_paras(%d).matrix_q_size(%d)",
                STATE_SIZE, i, lqr_conf_.forward_paras(i).matrix_q_size());
            return false;
        }
        if (lqr_conf_.forward_paras(i).matrix_r_size() != 1) {
            LOG_ERROR("lqr_conf_.forward_paras(%d).matrix_r_size(%d) != 1", i,
                      lqr_conf_.forward_paras(i).matrix_r_size());
            return false;
        }
    }

    // init matrix_p
    // Note: STATE_SIZE == lqr_conf_.forward_paras(0).matrix_q_size()
    lqr_matrix_.matrix_p = lqr_matrix_.matrix_q;
    for (int32_t i = 0; i < lqr_conf_.forward_paras(0).matrix_q_size(); ++i) {
        lqr_matrix_.matrix_p(i, i) = lqr_conf_.forward_paras(0).matrix_q(i);
    }

    return true;
}

// state = [lateral error, rate; heading error, rate,
// preview lateral1, preview lateral2, ...]
bool LqrLateralController::StateUpdate() {
    double local_x{localization_msg_->loc_pose().pose().position().x()},
        local_y{localization_msg_->loc_pose().pose().position().y()};

    int32_t target_index = 0;
    record_x_ = 0.0;
    record_y_ = 0.0;
    record_head_ = 0.0;
    TrajectoryAnalyzer analyzer;
    if (!analyzer.GetAbsoluteTimeLateralAndHeadingError(
            localization_msg_, trajectory_msg_, local_x, local_y, current_paras_.lateral_preview_time(),
            current_paras_.min_preview_dis(), lat_paras_.heading_offset(), &current_index_, &current_lateral_,
            &current_heading_, &current_ref_heading_, &current_heading_error_, &inner_lateral_error_,
            &inner_heading_error_, &record_x_, &record_y_, &record_head_, &target_index)) {
        LOG_ERROR("GetAbsoluteTimeLateralAndHeadingError failed");
        return false;
    }

    LOG_INFO("pt [x,y]: [%.4f, %.4f], target index: %d, draft lateral error: %.4f", local_x, local_y, current_index_,
             current_lateral_);

    current_lateral_error_ = current_lateral_;
    LOG_INFO("lateral error after first compensated: %.4f", current_lateral_error_);

    current_lateral_error_ = lateral_error_filter_.Update(current_lateral_error_);
    LOG_INFO("lateral error after filter: %.4f", current_lateral_error_);

    // 0.4 / 0.02, preview time
    int32_t pre_index = current_index_ + current_paras_.feedforward_preview_index();
    if (std::abs(current_speed_) < 2.1) {
        // TODO(zhaobolin): add trajectory_msg_->low_speed_control() from planning
        // low speed control, want to use large steer
        pre_index = target_index;
    }
    if (pre_index < 0 || pre_index >= trajectory_msg_->trajectory_point_size() - 1) {
        LOG_ERROR("pre_index error: %d", pre_index);
        return false;
    }
    target_index_ = pre_index;
    current_curvature_ = trajectory_msg_->trajectory_point(pre_index).path_point().kappa();
    LOG_INFO("curvature before filter: %.4f", current_curvature_);
    current_curvature_ = curvature_filter_.Update(current_curvature_);
    LOG_INFO("curvature after filter: %.4f", current_curvature_);

    LOG_INFO("current_heading_error_ before filter: %.4f", current_heading_error_);
    current_heading_error_ = heading_error_filter_.Update(current_heading_error_);
    LOG_INFO("current_heading_error_ after filter: %.4f", current_heading_error_);

    // use forward method to calc lat_rate and head_rate
    double math_lateral_error_rate = std::abs(current_speed_) * std::sin(current_heading_error_);
    if (is_current_backward_) {
        math_lateral_error_rate *= -1;
    }
    double angular_v = localization_msg_->loc_pose().pose().angular_velocity().z();
    if (is_current_backward_) {
        angular_v *= -1;
    }
    double math_heading_error_rate = angular_v - trajectory_msg_->trajectory_point(pre_index).path_point().kappa() *
                                                     trajectory_msg_->trajectory_point(pre_index).v();
    if (is_current_backward_) {
        math_heading_error_rate = angular_v + trajectory_msg_->trajectory_point(pre_index).path_point().kappa() *
                                                  trajectory_msg_->trajectory_point(pre_index).v();
    }

    lateral_error_rate_ = math_lateral_error_rate;
    heading_error_rate_ = math_heading_error_rate;

    LOG_INFO("lateral_error_rate_ before filter: %.4f", lateral_error_rate_);
    lateral_error_rate_ = lateral_rate_filter_.Update(lateral_error_rate_);
    LOG_INFO("lateral_error_rate_ after filter: %.4f", lateral_error_rate_);

    LOG_INFO("heading_error_rate_ before filter: %.4f", heading_error_rate_);
    heading_error_rate_ = heading_rate_filter_.Update(heading_error_rate_);
    LOG_INFO("heading_error_rate_ after filter: %.4f", heading_error_rate_);

    // Prepare for next iteration.
    previous_heading_error_ = current_heading_error_;
    previous_lateral_error_ = current_lateral_error_;

    // State matrix update;
    // First four elements are fixed;
    lqr_matrix_.matrix_state(0, 0) = current_lateral_error_;
    lqr_matrix_.matrix_state(1, 0) = lateral_error_rate_;
    lqr_matrix_.matrix_state(2, 0) = current_heading_error_;
    lqr_matrix_.matrix_state(3, 0) = heading_error_rate_;

    // debug info, use LOG_WARN.
    LOG_WARN(
        "Analysis, curr_x: %.4f, curr_y: %.4f,curr_head: %.4f, expect_x: %.4f, "
        "expect_y: %.4f,expect_head: %.4f, lateral_error: %.4f, head_error: %.4f",
        local_x, local_y, current_heading_, record_x_, record_y_, record_head_, inner_lateral_error_,
        inner_heading_error_);
    return true;
}

/**
 * @brief Update A based on v.
 */
void LqrLateralController::MatrixUpdate() {
    double v_ = std::max(current_speed_, lat_paras_.min_speed_protection());
    lqr_matrix_.matrix_a(1, 1) = lqr_matrix_.matrix_a_coeff(1, 1) / v_;
    lqr_matrix_.matrix_a(1, 3) = lqr_matrix_.matrix_a_coeff(1, 3) / v_;
    lqr_matrix_.matrix_a(3, 1) = lqr_matrix_.matrix_a_coeff(3, 1) / v_;
    lqr_matrix_.matrix_a(3, 3) = lqr_matrix_.matrix_a_coeff(3, 3) / v_;
    ControlMath::C2D(lqr_matrix_.matrix_a, lqr_matrix_.matrix_b, ts_, &lqr_matrix_.matrix_ad, &lqr_matrix_.matrix_bd);
    lqr_matrix_.backward_matrix_a(1, 1) = lqr_matrix_.backward_matrix_a_coeff(1, 1) / v_;
    lqr_matrix_.backward_matrix_a(1, 3) = lqr_matrix_.backward_matrix_a_coeff(1, 3) / v_;
    lqr_matrix_.backward_matrix_a(3, 1) = lqr_matrix_.backward_matrix_a_coeff(3, 1) / v_;
    lqr_matrix_.backward_matrix_a(3, 3) = lqr_matrix_.backward_matrix_a_coeff(3, 3) / v_;
    ControlMath::C2D(lqr_matrix_.backward_matrix_a, lqr_matrix_.backward_matrix_b, ts_, &lqr_matrix_.backward_matrix_ad,
                     &lqr_matrix_.backward_matrix_bd);
}

/**
 * @brief used for preview matrix. Reserved.
 */
void LqrLateralController::MatrixCompound() {
    // Initialize preview matrix
    lqr_matrix_.matrix_adc.block(0, 0, STATE_SIZE, STATE_SIZE) = lqr_matrix_.matrix_ad;
    lqr_matrix_.matrix_bdc.block(0, 0, STATE_SIZE, 1) = lqr_matrix_.matrix_bd;
    lqr_matrix_.backward_matrix_adc.block(0, 0, STATE_SIZE, STATE_SIZE) = lqr_matrix_.backward_matrix_ad;
    lqr_matrix_.backward_matrix_bdc.block(0, 0, STATE_SIZE, 1) = lqr_matrix_.backward_matrix_bd;
}

void LqrLateralController::SteerComponentCalc() {
    const auto& dyn_paras = lat_paras_.vehicle_dynamics_paras();
    const auto& stiffness = lat_paras_.tire_cornering_stiffness();

    // feedback = - K * state
    // Convert vehicle steer angle from rad to degree and then to steer degree
    // then to 100% ratio
    double gain = 180.0 / M_PI * lat_paras_.steer_ratio();

    steer_angle_feedbackterm_ = -(lqr_matrix_.matrix_k * lqr_matrix_.matrix_state)(0, 0) * gain;

    steer_angle_lateral_contribution_ = -lqr_matrix_.matrix_k(0, 0) * lqr_matrix_.matrix_state(0, 0) * gain;

    steer_angle_lateral_rate_contribution_ = -lqr_matrix_.matrix_k(0, 1) * lqr_matrix_.matrix_state(1, 0) * gain;

    steer_angle_heading_contribution_ = -lqr_matrix_.matrix_k(0, 2) * lqr_matrix_.matrix_state(2, 0) * gain;

    steer_angle_heading_rate_contribution_ = -lqr_matrix_.matrix_k(0, 3) * lqr_matrix_.matrix_state(3, 0) * gain;

    bool bflag = false;
    if (is_current_backward_) {
        // make sure you konw it before modify here
        if (lqr_matrix_.matrix_state(0, 0) > 0.01) {
            if (steer_angle_lateral_contribution_ > 0.01) {
                steer_angle_lateral_contribution_ *= -1;
                bflag = true;
            }
        } else if (lqr_matrix_.matrix_state(0, 0) < -0.01) {
            if (steer_angle_lateral_contribution_ < -0.01) {
                steer_angle_lateral_contribution_ *= -1;
                bflag = true;
            }
        }
    }

    if (bflag) {
        steer_angle_feedbackterm_ = steer_angle_lateral_contribution_ + steer_angle_lateral_rate_contribution_ +
                                    steer_angle_heading_contribution_ + steer_angle_heading_rate_contribution_;
    }

    double kv_ = 0.0;
    // feedforward
    if (is_current_backward_ && lqr_conf_.use_backward_model()) {
        // backward
        kv_ = -dyn_paras.lr() * dyn_paras.mass() / 2 / stiffness.cf() / lat_paras_.wheel_base() +
              dyn_paras.lf() * dyn_paras.mass() / 2 / stiffness.cr() / lat_paras_.wheel_base();
        if (std::abs(current_curvature_) < 0.002) {
            // TODO(chi.zhang): (review) magic num , move to proto?
            steer_angle_feedforwardterm_ = 0.0;
        } else {
            steer_angle_feedforwardterm_ = lat_paras_.wheel_base() * current_curvature_ * gain;
        }

        steer_angle_feedforwardterm_ = Clamp(steer_angle_feedforwardterm_, -lat_paras_.max_steer_angle() * 180.0 / M_PI,
                                             lat_paras_.max_steer_angle() * 180.0 / M_PI);
    } else {
        // forward
        kv_ = dyn_paras.lr() * dyn_paras.mass() / 2 / stiffness.cf() / lat_paras_.wheel_base() -
              dyn_paras.lf() * dyn_paras.mass() / 2 / stiffness.cr() / lat_paras_.wheel_base();
        double v_ = std::max(current_speed_, lat_paras_.min_speed_protection());

        // TODO(chi.zhang): (review) magic num , move to proto?
        if (std::abs(current_curvature_) < 0.002) {
            steer_angle_feedforwardterm_ = 0.0;
        } else {
            // then change it from rad to %
            steer_angle_feedforwardterm_ =
                (lat_paras_.wheel_base() * current_curvature_ + kv_ * v_ * v_ * current_curvature_ -
                 (lqr_conf_.enable_ff_angle_error()) * lqr_matrix_.matrix_k(0, 2) *
                     (dyn_paras.lr() * current_curvature_ - dyn_paras.lf() * dyn_paras.mass() * v_ * v_ *
                                                                current_curvature_ / 2 / stiffness.cr() /
                                                                lat_paras_.wheel_base())) *
                gain;
        }

        steer_angle_feedforwardterm_ = Clamp(steer_angle_feedforwardterm_, -lat_paras_.max_steer_angle() * 180.0 / M_PI,
                                             lat_paras_.max_steer_angle() * 180.0 / M_PI);
    }

    LOG_INFO("feedback: %.4f, feed_forward: %.4f,", steer_angle_feedbackterm_, steer_angle_feedforwardterm_);
    LOG_INFO("lat: %.4f, lat rate: %.4f, head: %.4f, head rate: %.4f", steer_angle_lateral_contribution_,
             steer_angle_lateral_rate_contribution_, steer_angle_heading_contribution_,
             steer_angle_heading_rate_contribution_);
}

bool LqrLateralController::GetRearPos(double* const x, double* const y) {
    if (x == nullptr || y == nullptr) {
        LOG_ERROR("input is nullptr");
        return false;
    }
    *x = localization_msg_->loc_pose().pose().position().x();
    *y = localization_msg_->loc_pose().pose().position().y();

    const auto& dyn_paras = lat_paras_.vehicle_dynamics_paras();
    double distance_lh = dyn_paras.lr() + lat_paras_.imu_offset();

    auto& orientation = localization_msg_->loc_pose().pose().orientation();
    EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());

    double curr_head = q.yaw() + lat_paras_.heading_offset();  //+ M_PI * 0.5
    *x = *x - distance_lh * std::cos(curr_head);
    *y = *y - distance_lh * std::sin(curr_head);

    return true;
}

bool LqrLateralController::CheckOverStopPoint() {
    int32_t current_index{0};
    double current_station = 0.0;
    double lateral = 0.0;
    double path_remain = 0.0;
    TrajectoryAnalyzer::GetSL(trajectory_msg_, localization_msg_->loc_pose().pose().position().x(),
                              localization_msg_->loc_pose().pose().position().y(), &current_station, &lateral,
                              &current_index);

    path_remain =
        TrajectoryAnalyzer::GetPathRemain(trajectory_msg_, current_station, lat_paras_.speed_deadzone(), false);
    if (path_remain < 0.0) {
        return true;
    } else {
        return false;
    }
}

bool LqrLateralController::SanityCheck() {
    // TODO(zhaobolin): add low speed control

    if (FLAGS_enable_lateral_curvature_check && std::abs(current_curvature_) > FLAGS_low_speed_curvature_threshold) {
        LOG_INFO("Check curvature error, current: %.4f", current_curvature_);
        return false;
    }
    // wheel_base/2
    if (FLAGS_enable_lateral_error_check && std::abs(current_lateral_) > FLAGS_low_speed_lateral_error_threshold) {
        LOG_INFO("Check lateral error, current: %.4f", current_lateral_);
        return false;
    }
    // M_PI / 3
    if (FLAGS_enable_heading_error_check &&
        std::abs(current_heading_error_) > FLAGS_low_speed_heading_error_threshold) {
        LOG_INFO("Check heading error, current: %.4f", current_heading_error_);
        return false;
    }

    return true;
    for (int32_t i = 0; i < lqr_conf_.forward_paras(0).matrix_q_size(); ++i) {
        lqr_matrix_.matrix_p(i, i) = lqr_conf_.forward_paras(0).matrix_q(i);
    }
}

/**
 * @brief Given speed, calculate Q and R.
 * @param this->current_paras_
 * @param this->lqr_matrix_.matrix_q_
 * @param this->lqr_matrix_.matrix_r_
 */
void LqrLateralController::GetParasFromScheduler(const double current_speed) {
    current_paras_ = lqr_conf_.forward_paras(0);

    if (is_current_backward_) {
        GainScheduler(current_speed, lqr_conf_.backward_paras());
    } else {
        GainScheduler(current_speed, lqr_conf_.forward_paras());
    }
    if (lqr_conf_.tuning_mode()) {
        current_paras_ = lqr_conf_.forward_paras(0);
    }
    // update paras
    for (int32_t i = 0; i < current_paras_.matrix_q_size(); ++i) {
        lqr_matrix_.matrix_q(i, i) = current_paras_.matrix_q(i);
    }
    lqr_matrix_.matrix_r(0, 0) = current_paras_.matrix_r(0);

    // debug, record paras
    for (int32_t i = 0; i < current_paras_.matrix_q_size(); ++i) {
        LOG_DEBUG("q[%d]: %.4f", i, current_paras_.matrix_q(i));
    }
    LOG_DEBUG("r[0]: %.4f", current_paras_.matrix_r(0));
    LOG_DEBUG("feedforward_ratio: %.4f", current_paras_.feedforward_ratio());
    LOG_DEBUG("feedback_ratio: %.4f", current_paras_.feedback_ratio());
    LOG_DEBUG("steer_delta_limit: %.4f", current_paras_.steer_delta_limit());
    LOG_DEBUG("lateral_preview_time: %.4f", current_paras_.lateral_preview_time());
    LOG_DEBUG("min_preview_dis: %.4f", current_paras_.min_preview_dis());
    LOG_DEBUG("feedforward_preview_index: %d", current_paras_.feedforward_preview_index());
}

/**
 * @brief Given speed, and pnc::control::LqrTable, calculate current_paras_.
 * Using linear interpolation.
 * @param this->current_paras_
 */
void LqrLateralController::GainScheduler(const double current_speed,
                                         const google::protobuf::RepeatedPtrField<pnc::control::LqrTable>& paras) {
    int32_t size = paras.size();
    if (current_speed <= paras[0].vel()) {
        current_paras_ = paras[0];
        return;
    } else if (current_speed >= paras[size - 1].vel()) {
        current_paras_ = paras[size - 1];
        return;
    }

    for (int32_t i = 1; i < size; ++i) {
        if (current_speed >= paras[i - 1].vel() && current_speed < paras[i].vel()) {
            auto& curr = paras[i - 1];
            auto& next = paras[i];

            double gain = (current_speed - curr.vel()) / (next.vel() - curr.vel());

            // q
            for (int32_t j = 0; j < current_paras_.matrix_q_size(); ++j) {
                double t = curr.matrix_q(j) + (next.matrix_q(j) - curr.matrix_q(j)) * gain;
                current_paras_.set_matrix_q(j, t);
            }
            // r
            double t = curr.matrix_r(0) + (next.matrix_r(0) - curr.matrix_r(0)) * gain;
            current_paras_.set_matrix_r(0, t);
            // feedforward_ratio
            t = curr.feedforward_ratio() + (next.feedforward_ratio() - curr.feedforward_ratio()) * gain;
            current_paras_.set_feedforward_ratio(t);
            // feedback_ratio
            t = curr.feedback_ratio() + (next.feedback_ratio() - curr.feedback_ratio()) * gain;
            current_paras_.set_feedback_ratio(t);
            // steer_delta_limit
            t = curr.steer_delta_limit() + (next.steer_delta_limit() - curr.steer_delta_limit()) * gain;
            current_paras_.set_steer_delta_limit(t);
            // lateral_preview_time
            t = curr.lateral_preview_time() + (next.lateral_preview_time() - curr.lateral_preview_time()) * gain;
            current_paras_.set_lateral_preview_time(t);
            // min_preview_dis
            t = curr.min_preview_dis() + (next.min_preview_dis() - curr.min_preview_dis()) * gain;
            current_paras_.set_min_preview_dis(t);
            // feedforward_preview_index
            int32_t tmp =
                curr.feedforward_preview_index() +
                static_cast<int32_t>((next.feedforward_preview_index() - curr.feedforward_preview_index()) * gain);
            current_paras_.set_feedforward_preview_index(tmp);
            return;
        }
    }

    return;
}

void LqrLateralController::LogDebugInfo() {
    if (use_debug_csv_file_) {
        if (!(steer_log_file_ == nullptr && !InitLogFile())) {
            fprintf(steer_log_file_,
                    "%d, %.4f, %.4f, %.4f, "
                    "%.4f, %.4f, %d, "
                    "%.4f, %.4f, %.4f, "
                    "%.4f, %.4f, %.4f, "
                    "%.4f, %.4f, "
                    "%.4f, %.4f, "
                    "%.4f, %.4f, "
                    "%.4f, %.4f, "
                    "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %d, %.4f,\n",
                    is_current_auto_mode_, localization_msg_->header().timestamp_sec(),
                    localization_msg_->loc_pose().pose().position().x(),
                    localization_msg_->loc_pose().pose().position().y(), current_heading_, current_speed_,
                    current_index_, current_ref_heading_, current_lateral_, lateral_error_rate_, current_heading_error_,
                    heading_error_rate_, current_curvature_, steer_angle_,
                    chassis_msg_->steering_percentage() / 100.0 * lat_paras_.max_steer_angle() * 180.0 / M_PI,
                    steer_angle_feedforwardterm_, steer_angle_feedbackterm_, steer_angle_lateral_contribution_,
                    steer_angle_lateral_rate_contribution_, steer_angle_heading_contribution_,
                    steer_angle_heading_rate_contribution_, inner_lateral_error_, inner_heading_error_,
                    trajectory_msg_->trajectory_point(current_index_).path_point().x(),
                    trajectory_msg_->trajectory_point(current_index_).path_point().y(), record_x_, record_y_,
                    record_head_, num_iterations_, chassis_msg_->steering_percentage());
            fflush(steer_log_file_);
        }
    }

    if (log_index_ >= 10) {
        LOG_INFO(
            "Steer_Detail, auto_mode: %d, time: %.4f, x: %.4f, y: %.4f, heading: "
            "%.4f, vel: %.4f, curr_index: %d, "
            "ref_heading: %.4f, lateral_error: %.4f, lateral_error_rate: %.4f,"
            "heading_error: %.4f, heading_error_rate: %.4f,  curvature: %.4f,"
            "send_steer_angle: %.4f, chassis_steer_angle: %.4f,"
            "feedforwardterm: %.4f, feedbackterm: %.4f,"
            "lateral_contribution: %.4f, lateral_rate_contribution: %.4f, "
            "heading_contribution: %.4f, heading_rate_contribution: %.4f, "
            "lateral_evaluation: %.4f, heading_evaluation: %.4f,"
            "record_x: %.4f, record_y: %.4f, record_head: %.4f, num_iterations: %d,"
            "chassis_steer_percentage: %.4f",
            is_current_auto_mode_, localization_msg_->header().timestamp_sec(),
            localization_msg_->loc_pose().pose().position().x(), localization_msg_->loc_pose().pose().position().y(),
            current_heading_, current_speed_, current_index_, current_ref_heading_, current_lateral_,
            lateral_error_rate_, current_heading_error_, heading_error_rate_, current_curvature_, steer_angle_,
            chassis_msg_->steering_percentage() / 100.0 * lat_paras_.max_steer_angle() * 180.0 / M_PI,
            steer_angle_feedforwardterm_, steer_angle_feedbackterm_, steer_angle_lateral_contribution_,
            steer_angle_lateral_rate_contribution_, steer_angle_heading_contribution_,
            steer_angle_heading_rate_contribution_, inner_lateral_error_, inner_heading_error_, record_x_, record_y_,
            record_head_, num_iterations_, chassis_msg_->steering_percentage());
        log_index_ = 0;
    }
    ++log_index_;
}

bool LqrLateralController::InitLogFile() {
    auto now_time = std::chrono::system_clock::now();
    auto now_time_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now_time);
    std::time_t now_c = std::chrono::system_clock::to_time_t(now_time_seconds);
    char name_buffer[80];
    strftime(name_buffer, 80, "steer_log_lqr_%Y%m%d%H%M%S.csv", std::localtime(&now_c));

    std::string file_name = FLAGS_pnc_data_log_path + "/control/" + name_buffer;
    steer_log_file_ = fopen(file_name.c_str(), "w");
    if (steer_log_file_ == nullptr) {
        return false;
    }
    fprintf(steer_log_file_,
            "auto_mode,time,x,y,heading,vel,curr_index,"
            "ref_heading,lateral_error,lateral_error_rate,"
            "heading_error,heading_error_rate,curvature,"
            "send_steer_angle,chassis_steer_angle,"
            "feedforwardterm,feedbackterm,"
            "lateral_contribution,lateral_rate_contribution,"
            "heading_contribution,"
            "heading_rate_contribution,lateral_evaluation,"
            "heading_evaluation,"
            "expected_x,expected_y,record_x,record_y,record_head,num_iterations,chassis_steer_percentage,\n");
    return true;
}

bool LqrLateralController::ControlMain(const ControlInputStream& control_input_stream,
                                       ControlCommandStream* const control_command_stream) {
    if (control_command_stream == nullptr) {
        LOG_ERROR("input is nullptr");
        return false;
    }

    if (!controller_initialized_) {
        LOG_ERROR("controller_initialized_ failed");
        return false;
    }
    localization_msg_ = &control_input_stream.input_stream.localization;
    chassis_msg_ = &control_input_stream.input_stream.chassis;
    trajectory_msg_ = &control_input_stream.input_stream.planning_info;

    if (chassis_msg_->driving_mode() != pnc::chassis::COMPLETE_MANUAL &&
        ControlMath::GearIsReady(localization_msg_, chassis_msg_, trajectory_msg_)) {
        is_current_auto_mode_ = true;
    } else {
        is_current_auto_mode_ = false;
    }

    bool curr_backward = ControlMath::CheckBackward(chassis_msg_);
    if (curr_backward != is_current_backward_) {
        is_current_backward_ = curr_backward;
        Reset();
        LOG_INFO("Reset()");
    }

    // switch between localization speed or chassis speed
    if (FLAGS_enable_using_localization_velocity) {
        const auto& speed3d = control_input_stream.input_stream.localization.loc_pose().pose().linear_velocity();
        current_speed_ = std::hypot(speed3d.x(), speed3d.y());
    } else {
        current_speed_ = control_input_stream.input_stream.chassis.speed_mps();
    }

    GetParasFromScheduler(current_speed_);

    // [lateral error, rate; heading error, rate]
    if (!StateUpdate()) {
        LOG_ERROR("StateUpdate failed");
        return false;
    }

    // matrix A
    MatrixUpdate();

    // Compound discrete matrix with raod preview model
    MatrixCompound();
    if (!lqr_conf_.enable_lqr_warm_start()) {
        if (is_current_backward_) {
            ControlMath::LqrSolver(lqr_matrix_.backward_matrix_adc, lqr_matrix_.backward_matrix_bdc,
                                   lqr_matrix_.matrix_q, lqr_matrix_.matrix_r, lqr_conf_.eps(),
                                   lqr_conf_.max_iteration(), &lqr_matrix_.matrix_k, &num_iterations_);
        } else {
            ControlMath::LqrSolver(lqr_matrix_.matrix_adc, lqr_matrix_.matrix_bdc, lqr_matrix_.matrix_q,
                                   lqr_matrix_.matrix_r, lqr_conf_.eps(), lqr_conf_.max_iteration(),
                                   &lqr_matrix_.matrix_k, &num_iterations_);
        }
    } else {
        if (is_current_backward_) {
            ControlMath::LqrSolverWarmStart(lqr_matrix_.backward_matrix_adc, lqr_matrix_.backward_matrix_bdc,
                                            lqr_matrix_.matrix_q, lqr_matrix_.matrix_r, lqr_conf_.eps(),
                                            lqr_conf_.max_iteration(), &lqr_matrix_.matrix_k, &num_iterations_,
                                            &lqr_matrix_.matrix_p);
        } else {
            ControlMath::LqrSolverWarmStart(lqr_matrix_.matrix_adc, lqr_matrix_.matrix_bdc, lqr_matrix_.matrix_q,
                                            lqr_matrix_.matrix_r, lqr_conf_.eps(), lqr_conf_.max_iteration(),
                                            &lqr_matrix_.matrix_k, &num_iterations_, &lqr_matrix_.matrix_p);
        }
    }

    SteerComponentCalc();

    steer_angle_ = steer_angle_feedbackterm_ * current_paras_.feedback_ratio() +
                   steer_angle_feedforwardterm_ * current_paras_.feedforward_ratio();

    // TODO(chi.zhang): Add leadlag. reserved
    // ...
    // TODO(chi.zhang): Add steer_limit based on lateral acceleration and central
    // acceleration
    // reserved.
    // TODO(chi.zhang): Add mrac. reserved
    // ...

    steer_angle_ =
        Clamp(steer_angle_, -lat_paras_.max_steer_angle() * 180.0 / M_PI, lat_paras_.max_steer_angle() * 180.0 / M_PI);
    steer_angle_ = steer_angle_filter_.Filter(steer_angle_);

    // steer protection
    bool over_stop_point = CheckOverStopPoint();
    LOG_INFO("over stop point: %d", over_stop_point);
    if (FLAGS_enable_lowspeed_steer_protect &&
        ((current_speed_ < lat_paras_.lock_steer_speed() &&
          control_input_stream.input_stream.chassis.has_gear_location() &&
          control_input_stream.input_stream.chassis.gear_location() == pnc::chassis::GEAR_DRIVE) ||
         over_stop_point)) {
        steer_angle_ = pre_steer_angle_;
        LOG_INFO("use pre_steer_angle");
    }

    steer_angle_ =
        Clamp(steer_angle_, -lat_paras_.max_steer_angle() * 180.0 / M_PI, lat_paras_.max_steer_angle() * 180.0 / M_PI);

    // This is used to check whether to stop vehicle to decrease steer delta
    FLAGS_big_steer_diff_stop_flag = false;
    if (!FLAGS_enable_lowspeed_steer_protect && FLAGS_enable_big_steer_diff_stop_function) {
        double curr_steer = control_input_stream.input_stream.chassis.steering_percentage();
        if (std::abs(curr_steer - steer_angle_) > FLAGS_big_steer_diff_threshold) {
            FLAGS_big_steer_diff_stop_flag = true;
            LOG_INFO("curr_steer: %.4f, target_steer: %.4f differs a lot, stop vehicle", curr_steer, steer_angle_);
        }
    }

    // add ramp filter, in case of big steer rate--> central acc, central jerk
    if (steer_angle_ > pre_steer_angle_ + current_paras_.steer_delta_limit()) {
        steer_angle_ = pre_steer_angle_ + current_paras_.steer_delta_limit();
    } else if (steer_angle_ < pre_steer_angle_ - current_paras_.steer_delta_limit()) {
        steer_angle_ = pre_steer_angle_ - current_paras_.steer_delta_limit();
    }

    // TODO(chi.zhang):
    steer_angle_ =
        Clamp(steer_angle_, -lat_paras_.max_steer_angle() * 180.0 / M_PI, lat_paras_.max_steer_angle() * 180.0 / M_PI);
    LOG_INFO("after ramp filter steer angle: %.4f", steer_angle_);

    pre_steer_angle_ = steer_angle_;
    control_command_stream->control_command.set_steering_target(steer_angle_);
    control_command_stream->control_command.set_steering_rate(lat_paras_.max_steer_angle_rate() * 180.0 / M_PI);
    // set control context
    control_command_stream->control_command.mutable_debug()->mutable_urban_lat_debug()->set_heading_error(
        current_heading_error_);
    control_command_stream->control_command.mutable_debug()->mutable_urban_lat_debug()->set_lateral_error(
        current_lateral_error_);
    control_command_stream->control_command.mutable_debug()->mutable_urban_lat_debug()->set_steer_control(steer_angle_);

    LogDebugInfo();
    SummaryDebugInfo(&control_command_stream->control_debug);

    // for simulation
    if (FLAGS_enable_lateral_sanity_check && !SanityCheck()) {
        LOG_ERROR("Lateral SanityCheck error");
        return false;
    }

    return true;
}

bool LqrLateralController::SummaryDebugInfo(pnc::control::ControlDebug* const control_debug) const {
    auto* lqr_debug = control_debug->mutable_lqr_controller_debug();
    const auto& dyn_paras = lat_paras_.vehicle_dynamics_paras();
    const auto& stiffness = lat_paras_.tire_cornering_stiffness();
    lqr_debug->set_mass(dyn_paras.mass());
    lqr_debug->set_lr(dyn_paras.lr());
    lqr_debug->set_lf(dyn_paras.lf());
    lqr_debug->set_cr(stiffness.cr());
    lqr_debug->set_cf(stiffness.cf());
    lqr_debug->set_enable_controller(true);
    lqr_debug->set_cur_index(current_index_);
    lqr_debug->set_target_index(target_index_);

    lqr_debug->set_cur_velocity(current_speed_);
    lqr_debug->set_cur_ref_kappa(current_curvature_);
    lqr_debug->set_steer_wheel_deg(steer_angle_);
    lqr_debug->set_steer_wheel_fb_pecentage(chassis_msg_->steering_percentage());
    lqr_debug->set_steer_angle_ff(steer_angle_feedforwardterm_);
    lqr_debug->set_steer_angle_fb(steer_angle_feedbackterm_);
    lqr_debug->set_steer_cmd_lat_part(steer_angle_lateral_contribution_);
    lqr_debug->set_steer_cmd_lat_rate_part(steer_angle_lateral_rate_contribution_);
    lqr_debug->set_steer_cmd_hdg_part(steer_angle_heading_contribution_);
    lqr_debug->set_steer_cmd_hdg_rate_part(steer_angle_heading_rate_contribution_);

    lqr_debug->set_matrix_lat_err(current_lateral_error_);
    lqr_debug->set_matrix_lat_err_rate(lateral_error_rate_);
    lqr_debug->set_matrix_hdg_err(current_heading_error_);
    lqr_debug->set_matrix_hdg_err_rate(heading_error_rate_);

    for (int32_t i = 0; i < current_paras_.matrix_q_size(); ++i) {
        lqr_debug->add_cur_q(current_paras_.matrix_q(i));
    }
    lqr_debug->set_cur_r(current_paras_.matrix_r(0));  // case only 1
    for (int32_t i = 0; i < STATE_SIZE; ++i) {
        lqr_debug->add_cur_k(lqr_matrix_.matrix_k(0, i));
    }
    lqr_debug->set_num_iterations(num_iterations_);

    return true;
}

}  // namespace control
}  // namespace jiduauto
