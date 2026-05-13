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

#include <gtest/gtest.h>

#include "control/src/common/control_gflags.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::EulerAnglesZXYf;
using pnc::chassis::Chassis;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

double LinearFunc(double x) { return 2.0 * x; }

double SquareFunc(double x) { return x * x; }

double CubicFunc(double x) { return (x - 1.0) * (x - 2.0) * (x - 3.0); }

double SinFunc(double x) { return std::sin(x); }

TEST(ControlMathTest, C2DTest) {
    const int states = 4;
    Eigen::MatrixXd matrix_a = Eigen::MatrixXd::Zero(states, states);
    Eigen::MatrixXd matrix_ad = Eigen::MatrixXd::Zero(states, states);
    Eigen::MatrixXd matrix_b = Eigen::MatrixXd::Zero(states, 1);
    Eigen::MatrixXd matrix_bd = Eigen::MatrixXd::Zero(states, 1);

    Eigen::MatrixXd matrix_I = Eigen::MatrixXd::Identity(states, states);
    double ts = 0.01;
    ControlMath::C2D(matrix_a, matrix_b, ts, &matrix_ad, &matrix_bd);

    for (int i = 0; i < states; i++) {
        EXPECT_EQ(matrix_bd(i, 0), matrix_b(i, 0));
        for (int j = 0; j < states; j++) {
            EXPECT_EQ(matrix_ad(i, j), matrix_I(i, j));
        }
    }
}

TEST(ControlMathTest, LqrFullRankMatrix) {
    // TO-DO reverse
}

TEST(ControlMathTest, SmoothTest) {
    double pre_val = 20.0;
    double val = 40.0;

    double zone_width = 10;
    ControlMath::Smooth(pre_val, zone_width, &val);
    EXPECT_NEAR(val, 30, 1e-6);

    zone_width = 25;
    val = 40.0;
    ControlMath::Smooth(pre_val, zone_width, &val);
    EXPECT_NEAR(val, 40, 1e-6);

    val = -10;
    ControlMath::Smooth(pre_val, zone_width, &val);
    EXPECT_NEAR(val, -5, 1e-6);
}

TEST(SearchTest, GoldenSectionSearch) {
    double linear_argmin = ControlMath::GoldenSectionSearch(LinearFunc, 0.0, 1.0, 1e-6);
    EXPECT_NEAR(linear_argmin, 0.0, 1e-5);
    double square_argmin_1 = ControlMath::GoldenSectionSearch(SquareFunc, -1.0, 2.0, 1e-6);
    EXPECT_NEAR(square_argmin_1, 0.0, 1e-5);
    double square_argmin_2 = ControlMath::GoldenSectionSearch(SquareFunc, 1.0, 2.0, 1e-6);
    EXPECT_NEAR(square_argmin_2, 1.0, 1e-5);
    double cubic_argmin_1 = ControlMath::GoldenSectionSearch(CubicFunc, 0.0, 1.5, 1e-6);
    EXPECT_NEAR(cubic_argmin_1, 0.0, 1e-5);
    double cubic_argmin_2 = ControlMath::GoldenSectionSearch(CubicFunc, 1.0, 1.8, 1e-6);
    EXPECT_NEAR(cubic_argmin_2, 1.0, 1e-5);
    double cubic_argmin_3 = ControlMath::GoldenSectionSearch(CubicFunc, 2.0, 3.0, 1e-6);
    EXPECT_NEAR(cubic_argmin_3, 2.0 + 1.0 / std::sqrt(3.0), 1e-5);
    double sin_argmin = ControlMath::GoldenSectionSearch(SinFunc, 0.0, 2 * M_PI, 1e-6);
    EXPECT_NEAR(sin_argmin, 1.5 * M_PI, 1e-5);
}

TEST(ControlMath, ErrorSpeedLimitTest) {
    double lateral_err;
    double head_err;
    double speed_limit = 0.0;

    lateral_err = 0.5;
    head_err = 0.2;
    ControlMath::ErrorSpeedLimit(lateral_err, head_err, &speed_limit);
    EXPECT_NEAR(speed_limit, 1.0, 1e-6);
    lateral_err = 0.3;
    head_err = 0.4;
    ControlMath::ErrorSpeedLimit(lateral_err, head_err, &speed_limit);
    EXPECT_NEAR(speed_limit, 1.0, 1e-6);
    lateral_err = 0.3;
    head_err = 0.3;
    ControlMath::ErrorSpeedLimit(lateral_err, head_err, &speed_limit);
    EXPECT_NEAR(speed_limit, 2.0, 1e-6);
}

TEST(ControlMath, GetAccelerationThrottleTest) {
    double acc_required = 0.0;
    double acceleration_at_min = 0.0;
    double acceleration_per_001_throttle = 0.0;
    double acceleration_throttle_max = 0.0;
    double acceleration_throttle_min = 0.0;
    double result = 0.0;

    acc_required = 5.0;
    acceleration_at_min = 10.0;

    result = ControlMath::GetAccelerationThrottle(acc_required, acceleration_at_min, acceleration_per_001_throttle,
                                                  acceleration_throttle_max, acceleration_throttle_min);
    EXPECT_NEAR(result, 0.0, 1e-6);

    acc_required = 10.0;
    acceleration_at_min = 5.0;
    acceleration_per_001_throttle = 10.0;
    acceleration_throttle_min = 0.0;
    acceleration_throttle_max = 0.7;
    result = ControlMath::GetAccelerationThrottle(acc_required, acceleration_at_min, acceleration_per_001_throttle,
                                                  acceleration_throttle_max, acceleration_throttle_min);
    EXPECT_NEAR(result, 0.5, 1e-6);
    acceleration_per_001_throttle = 5.0;
    result = ControlMath::GetAccelerationThrottle(acc_required, acceleration_at_min, acceleration_per_001_throttle,
                                                  acceleration_throttle_max, acceleration_throttle_min);
    EXPECT_NEAR(result, 0.7, 1e-6);

    acc_required = -10.0;
    result = ControlMath::GetAccelerationThrottle(acc_required, acceleration_at_min, acceleration_per_001_throttle,
                                                  acceleration_throttle_max, acceleration_throttle_min);
    EXPECT_NEAR(result, 0.7, 1e-6);
}

TEST(ControlMath, GetDecelerationBrakeTest) {
    double dec_required = 0.0;
    double deceleration_at_min = 0.0;
    double deceleration_per_001_brake = 0.0;
    double deceleration_brake_max = 0.0;
    double deceleration_brake_min = 0.0;
    double result = 0.0;

    dec_required = 5.0;
    deceleration_at_min = 10.0;

    result = ControlMath::GetAccelerationThrottle(dec_required, deceleration_at_min, deceleration_per_001_brake,
                                                  deceleration_brake_max, deceleration_brake_min);
    EXPECT_NEAR(result, 0.0, 1e-6);

    dec_required = 10.0;
    deceleration_at_min = 5.0;
    deceleration_per_001_brake = 10.0;
    deceleration_brake_min = 0.0;
    deceleration_brake_max = 0.7;
    result = ControlMath::GetDecelerationBrake(dec_required, deceleration_at_min, deceleration_per_001_brake,
                                               deceleration_brake_max, deceleration_brake_min);
    EXPECT_NEAR(result, 0.5, 1e-6);
    deceleration_per_001_brake = 5.0;
    result = ControlMath::GetDecelerationBrake(dec_required, deceleration_at_min, deceleration_per_001_brake,
                                               deceleration_brake_max, deceleration_brake_min);
    EXPECT_NEAR(result, 0.7, 1e-6);

    dec_required = -10.0;
    result = ControlMath::GetDecelerationBrake(dec_required, deceleration_at_min, deceleration_per_001_brake,
                                               deceleration_brake_max, deceleration_brake_min);
    EXPECT_NEAR(result, 0.7, 1e-6);
}

}  // namespace control
}  // namespace jiduauto
