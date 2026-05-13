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

#pragma once

#include <Eigen/Core>
#include <functional>
#include <vector>

#include "pnc_localization_estimate.pb.h"
#include "pnc_planning.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {
/**
 * @brief provide math functions.
 */

class ControlMath {
public:
    ControlMath() = delete;
    ~ControlMath() = delete;

    static double DotProduct(const double x1, const double y1, const double x2, const double y2) {
        return x1 * x2 + y1 * y2;
    }

    static double CrossProduct(const double x1, const double y1, const double x2, const double y2) {
        return x1 * y2 - x2 * y1;
    }

    static int16_t Sign(const double value) { return (value > 0) ? 1 : ((value < 0) ? -1 : 0); }

    /**
     * @brief Continuous matrix to Discrete matrix.
     * Computes bilinear transformation of the continuous to discrete form
     * for state space representation.
     */
    static void C2D(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b, const double ts,
                    Eigen::MatrixXd* matrix_ad, Eigen::MatrixXd* matrix_bd);

    /**
     * @brief solver of lqr problem.
     */
    static void LqrSolver(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b,
                          const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r, const double eps,
                          const int32_t max_num_iteration, Eigen::MatrixXd* const matrix_k,
                          int32_t* const num_iterations_ptr);

    static void LqrSolverWarmStart(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b,
                                   const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r, const double eps,
                                   const int32_t max_num_iteration, Eigen::MatrixXd* const matrix_k,
                                   int32_t* const num_iterations_ptr, Eigen::MatrixXd* const matrix_p_ptr);

    /**
     * @brief coefficient of low pass filter.
     */
    static void GetLpfCoefficient(const double ts, const double cutoff_freq, std::vector<double>* const den,
                                  std::vector<double>* const num);

    /**
     * @brief smooth using ramp method.
     */
    static void Smooth(const double pre_val, const double zone_width, double* const val);

    /**
     * @param func The target single-variable function to minimize.
     * @param lower_bound The lower bound of the interval.
     * @param upper_bound The upper bound of the interval.
     * @param tol The tolerance of error.
     * @return The value that minimize the function fun.
     */
    static double GoldenSectionSearch(const std::function<double(double)>& func, const double lower_bound,
                                      const double upper_bound, const double tol = 1e-6);

    /**
     * @brief hiss assist function. Given throttle on GEAR_PARKING.
     */
    static bool IsEnableHillAssist(const pnc::chassis::Chassis* const chassis,
                                   const pnc::planning::ADCTrajectory* const trajectory, const double pitch_angle);

    /**
     * @brief check whether gear is ready.
     */
    static bool GearIsReady(const pnc::localization::LocalizationEstimate* const localization,
                            const pnc::chassis::Chassis* const chassis,
                            const pnc::planning::ADCTrajectory* const trajectory);

    /**
     * @brief check whether gear is reverse.
     */
    static bool CheckBackward(const pnc::chassis::Chassis* const chassis);

    /**
     * @brief Given lateral error and heading error, get speed_limit.
     * TODO: chi.zhang. Optimize it.
     */
    static bool ErrorSpeedLimit(const double lateral_err, const double head_err, double* speed_limit);

    /**
     * @brief Given 'a', calculate throttle using linear approximation.
     */
    static double GetAccelerationThrottle(const double acc_required, const double acceleration_at_min,
                                          const double acceleration_per_001_throttle,
                                          const double acceleration_throttle_max,
                                          const double acceleration_throttle_min);

    /**
     * @brief Given 'a', calculate brake using linear approximation.
     */
    static double GetDecelerationBrake(const double dec_required, const double deceleration_at_min,
                                       const double deceleration_per_001_brake, const double deceleration_brake_max,
                                       const double deceleration_brake_min);

    /**
     * @brief Safety check, whether door is open.
     * @return true-door opened; false-door closed.
     */
    static bool RequestDoorOpenProtection(const pnc::chassis::Chassis& chassis);

    /**
     * @brief Safety check, whether vehicle slipped.
     * @return true-vehicle slipped; otherwise false.
     * TODO: chi.zhang. develop it.
     */
    static bool RequestVehicleSlipProtection(const pnc::localization::LocalizationEstimate& localization,
                                             const pnc::chassis::Chassis& chassis);

private:
};
}  // namespace control
}  // namespace jiduauto
