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

#include "control/src/math/control_math.h"

#include <Eigen/LU>

#include "control/src/common/control_gflags.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::EulerAnglesZXYf;
using pnc::chassis::Chassis;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

// use bilinear approximation
void ControlMath::C2D(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b, const double ts,
                      Eigen::MatrixXd* matrix_ad, Eigen::MatrixXd* matrix_bd) {
    if (matrix_ad == nullptr || matrix_bd == nullptr) {
        LOG_ERROR("Matrix Ad or Bd is nullptr");
        return;
    }
    if (matrix_a.rows() != matrix_a.cols()) {
        LOG_ERROR("Matrix A dimension mismatch.");
        return;
    }
    if (matrix_b.rows() != matrix_a.rows() || matrix_b.cols() != 1) {
        LOG_ERROR("Matrix B dimension mismatch.");
        return;
    }
    Eigen::MatrixXd matrix_i = Eigen::MatrixXd::Identity(matrix_a.cols(), matrix_a.cols());
    Eigen::MatrixXd matrix_i_minus_at_inverse = (matrix_i - ts * 0.5 * matrix_a).inverse();
    *matrix_ad = matrix_i_minus_at_inverse * (matrix_i + ts * 0.5 * matrix_a);
    *matrix_bd = matrix_i_minus_at_inverse * matrix_b * ts;
}

void ControlMath::LqrSolver(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b,
                            const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r, const double eps,
                            const int32_t max_num_iteration, Eigen::MatrixXd* const matrix_k,
                            int32_t* const num_iterations_ptr) {
    *num_iterations_ptr = 0;
    if (matrix_k == nullptr) {
        LOG_ERROR("matrix_k is nullptr");
        return;
    }
    if (matrix_a.rows() != matrix_a.cols() || matrix_b.rows() != matrix_a.rows() ||
        matrix_q.rows() != matrix_q.cols() || matrix_q.rows() != matrix_a.rows() ||
        matrix_r.rows() != matrix_r.cols() || matrix_r.rows() != matrix_b.cols()) {
        LOG_ERROR("One or more matrices have incompatible dimensions. Aborting.");
        return;
    }

    // Pre-compute
    Eigen::MatrixXd matrix_a_tran = matrix_a.transpose();
    Eigen::MatrixXd matrix_b_tran = matrix_b.transpose();

    // Caculate Matrix Difference Riccati Equation, initialize P and Q
    Eigen::MatrixXd matrix_p = matrix_q;

    // int num_iterations = 0;
    Eigen::MatrixXd matrix_p_old = matrix_p;

    double diff = 0;
    while ((*num_iterations_ptr)++ < max_num_iteration) {
        matrix_p = matrix_a_tran * matrix_p * matrix_a -
                   matrix_a_tran * matrix_p * matrix_b * (matrix_r + matrix_b_tran * matrix_p * matrix_b).inverse() *
                       matrix_b_tran * matrix_p * matrix_a +
                   matrix_q;
        // update delta
        Eigen::MatrixXd delta = matrix_p - matrix_p_old;
        Eigen::ArrayXXd delta_abs = delta.array().abs();
        diff = delta_abs.maxCoeff();
        if (diff < eps) {
            LOG_INFO(
                "Number of iterations until convergence:, %d,"
                "max difference:, %f \n",
                *num_iterations_ptr, diff);
            break;
        }
        matrix_p_old = matrix_p;
    }

    *matrix_k = (matrix_r + matrix_b_tran * matrix_p * matrix_b).inverse() * matrix_b_tran * matrix_p * matrix_a;
}

void ControlMath::LqrSolverWarmStart(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b,
                                     const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r, const double eps,
                                     const int32_t max_num_iteration, Eigen::MatrixXd* const matrix_k,
                                     int32_t* const num_iterations_ptr, Eigen::MatrixXd* const matrix_p_ptr) {
    *num_iterations_ptr = 0;
    if (matrix_k == nullptr) {
        LOG_ERROR("matrix_k is nullptr");
        return;
    }
    if (matrix_a.rows() != matrix_a.cols() || matrix_b.rows() != matrix_a.rows() ||
        matrix_q.rows() != matrix_q.cols() || matrix_q.rows() != matrix_a.rows() ||
        matrix_r.rows() != matrix_r.cols() || matrix_r.rows() != matrix_b.cols()) {
        LOG_ERROR("One or more matrices have incompatible dimensions. Aborting.");
        return;
    }

    // Pre-compute
    Eigen::MatrixXd matrix_a_tran = matrix_a.transpose();
    Eigen::MatrixXd matrix_b_tran = matrix_b.transpose();

    // Caculate Matrix Difference Riccati Equation, initialize P and Q
    // Eigen::MatrixXd matrix_p = matrix_q;
    Eigen::MatrixXd matrix_p = *matrix_p_ptr;

    // int num_iterations = 0;
    Eigen::MatrixXd matrix_p_old = matrix_p;

    double diff = 0;
    while ((*num_iterations_ptr)++ < max_num_iteration) {
        matrix_p = matrix_a_tran * matrix_p * matrix_a -
                   matrix_a_tran * matrix_p * matrix_b * (matrix_r + matrix_b_tran * matrix_p * matrix_b).inverse() *
                       matrix_b_tran * matrix_p * matrix_a +
                   matrix_q;
        // update delta
        Eigen::MatrixXd delta = matrix_p - matrix_p_old;
        Eigen::ArrayXXd delta_abs = delta.array().abs();
        diff = delta_abs.maxCoeff();
        if (diff < eps) {
            LOG_INFO(
                "Number of iterations until convergence:, %d,"
                "max difference:, %f \n",
                *num_iterations_ptr, diff);
            break;
        }
        matrix_p_old = matrix_p;
    }

    *matrix_k = (matrix_r + matrix_b_tran * matrix_p * matrix_b).inverse() * matrix_b_tran * matrix_p * matrix_a;
    *matrix_p_ptr = matrix_p;
}

void ControlMath::GetLpfCoefficient(const double ts, const double cutoff_freq, std::vector<double>* const den,
                                    std::vector<double>* const num) {
    // This is coefficient for two pole, second order low pass butterworth filter,
    // No pre-warp
    if (den == nullptr || num == nullptr) {
        if (den == nullptr) {
            LOG_ERROR("den is nullptr");
        } else if (num == nullptr) {
            LOG_ERROR("num is nullptr");
        }
        return;
    }
    if ((den->size() != 3) || (num->size() != 3)) {
        LOG_ERROR("denumerator or numerator size wrong!");
        return;
    }
    double wa = 2 * M_PI * cutoff_freq;  // Analog frequency in rad/s
    double alpha = wa * ts / 2;          // tan(Wd/2), Wd is discrete frequency
    double alpha_sqr = alpha * alpha;
    double tmp_term = sqrt(2) * alpha + alpha_sqr;
    double gain = alpha_sqr / (1 + tmp_term);
    (*den)[2] = (1 - sqrt(2) * alpha + alpha_sqr) / (1 + tmp_term);
    (*den)[1] = 2 * (alpha_sqr - 1) / (1 + tmp_term);
    (*den)[0] = 1;

    (*num)[0] = gain;
    (*num)[1] = 2 * gain;
    (*num)[2] = gain;
    return;
}

void ControlMath::Smooth(const double pre_val, const double zone_width, double* const val) {
    if (val == nullptr) {
        LOG_ERROR("val is nullptr");
        return;
    }
    if (std::abs(*val - pre_val) > zone_width) {
        if (*val > pre_val)
            *val = pre_val + zone_width;
        else if (*val < pre_val)
            *val = pre_val - zone_width;
    }
    return;
}

double ControlMath::GoldenSectionSearch(const std::function<double(double)>& func, const double lower_bound,
                                        const double upper_bound, const double tol) {
    static constexpr double gr = 1.618033989;  // (sqrt(5) + 1) / 2

    double a = lower_bound;
    double b = upper_bound;

    double t = (b - a) / gr;
    double c = b - t;
    double d = a + t;

    while (std::abs(c - d) > tol) {
        if (func(c) < func(d)) {
            b = d;
        } else {
            a = c;
        }
        t = (b - a) / gr;
        c = b - t;
        d = a + t;
    }
    return (a + b) * 0.5;
}

bool ControlMath::IsEnableHillAssist(const Chassis* const chassis, const ADCTrajectory* const trajectory,
                                     const double pitch_angle) {
    if (!FLAGS_enable_hill_assist_function) {
        return false;
    }
    if (chassis == nullptr || trajectory == nullptr) {
        return false;
    }

    if (chassis->gear_location() != pnc::chassis::GEAR_PARKING) {
        return false;
    }
    if (!(trajectory->gear() == pnc::chassis::GEAR_DRIVE || trajectory->gear() == pnc::chassis::GEAR_REVERSE)) {
        return false;
    }

    if (chassis->speed_mps() >= 0.3) {  // TODO(anyone): (review) magic num , move to proto?
        return false;
    }

    if (std::abs(pitch_angle) < 0.04) {  // TODO(anyone): (review) magic num , move to proto?
        return false;
    }

    return true;
}

bool ControlMath::GearIsReady(const LocalizationEstimate* const localization, const Chassis* const chassis,
                              const ADCTrajectory* const trajectory) {
    if (localization == nullptr || chassis == nullptr || trajectory == nullptr) {
        return false;
    }
    auto& orientation = localization->loc_pose().pose().orientation();
    EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());

    bool enable_hill_assist = IsEnableHillAssist(chassis, trajectory, q.pitch());

    if (((chassis->gear_location() == pnc::chassis::GEAR_DRIVE ||
          (enable_hill_assist && chassis->gear_location() == pnc::chassis::GEAR_PARKING)) &&
         trajectory->gear() == pnc::chassis::GEAR_DRIVE) ||
        ((chassis->gear_location() == pnc::chassis::GEAR_REVERSE ||
          (enable_hill_assist && chassis->gear_location() == pnc::chassis::GEAR_PARKING)) &&
         trajectory->gear() == pnc::chassis::GEAR_REVERSE)) {
        return true;
    } else {
        return false;
    }
}

bool ControlMath::CheckBackward(const Chassis* const chassis) {
    if (chassis == nullptr) {
        LOG_ERROR("chassis is nullptr");
        return false;
    }

    if (chassis->gear_location() == pnc::chassis::GEAR_REVERSE) {
        if (FLAGS_enable_control_info_print) {
            LOG_INFO("Is Backward!");
        }
        return true;
    } else {
        return false;
    }
}

bool ControlMath::ErrorSpeedLimit(const double lateral_err, const double head_err, double* speed_limit) {
    // TODO(anyone): (review) magic num , move to proto?
    if (std::abs(lateral_err) >= 0.4 || std::abs(head_err) >= 0.35) {
        *speed_limit = 1.0;
    } else if (std::abs(lateral_err) >= 0.25 || std::abs(head_err) >= 0.17) {
        *speed_limit = 2.0;  // TODO(anyone): (review) magic num , move to proto?
    }
    return true;
}

double ControlMath::GetAccelerationThrottle(const double acc_required, const double acceleration_at_min,
                                            const double acceleration_per_001_throttle,
                                            const double acceleration_throttle_max,
                                            const double acceleration_throttle_min) {
    if (acc_required < 0) {
        return GetAccelerationThrottle(-acc_required, acceleration_at_min, acceleration_per_001_throttle,
                                       acceleration_throttle_max, acceleration_throttle_min);
    }
    if (acc_required < acceleration_at_min) {
        if (FLAGS_enable_control_info_print) {
            LOG_INFO("acceleration_too_small");
        }
        return 0.0;
    } else {
        double acc_percent = (acc_required - acceleration_at_min) / acceleration_per_001_throttle;
        if (acc_percent > acceleration_throttle_max) {
            acc_percent = acceleration_throttle_max;
        }
        return acc_percent;
    }
}

double ControlMath::GetDecelerationBrake(const double dec_required, const double deceleration_at_min,
                                         const double deceleration_per_001_brake, const double deceleration_brake_max,
                                         const double deceleration_brake_min) {
    if (dec_required < 0) {
        return GetDecelerationBrake(-dec_required, deceleration_at_min, deceleration_per_001_brake,
                                    deceleration_brake_max, deceleration_brake_min);
    }
    if (dec_required < deceleration_at_min) {
        if (FLAGS_enable_control_info_print) {
            LOG_INFO("deceleration_too_small");
        }
        return 0.0;
    } else {
        double brake_percent = (dec_required - deceleration_at_min) / deceleration_per_001_brake;
        if (brake_percent > deceleration_brake_max) {
            brake_percent = deceleration_brake_max;
        }
        return brake_percent;
    }
}

bool ControlMath::RequestDoorOpenProtection(const Chassis& chassis) {
    if (chassis.has_dash_board() && chassis.dash_board().has_is_total_door_lock() &&
        chassis.dash_board().is_total_door_lock() == false) {
        LOG_WARN("Door is Open");
        return true;
    }
    return false;
}

bool ControlMath::RequestVehicleSlipProtection(const LocalizationEstimate& localization, const Chassis& chassis) {
    // reserved
    return false;
}

}  // namespace control
}  // namespace jiduauto
