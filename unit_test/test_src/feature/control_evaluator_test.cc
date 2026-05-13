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

// Copyright 2022 JiDuAuto LLC
// Author: Bolin ZHAO, Jidu PnC

#include "control/src/feature/control_evaluator.h"

#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/common/file_util.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/pnc_utility.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"
#include "pnc_control_cmd.pb.h"
#include "pnc_control_config.pb.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::math::NormalizeAngle;
using jiduauto::pnc::util::TimeUtility;
using pnc::chassis::Chassis;
using pnc::control::ControlCommand;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

class ControlEvaluatorTest : public ::testing::Test {
public:
    virtual void SetUp() {
        CHECK(FileUtil::ReadPnCPath());
        std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));
        InitVehicleState();
        InitPlanning();
    }

    void InitVehicleState(bool random = false);
    void InitPlanning();

protected:
    pnc::control::ControlConfig control_conf_;
    pnc::common::VehicleState vehicle_state_;
    pnc::planning::ADCTrajectory planning_msg_;
    pnc::control::ControlCommand cmd_msg_;

    double curr_x_{0.2};
};

void ControlEvaluatorTest::InitVehicleState(bool random) {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    vehicle_state_.set_timestamp_sec(curr_time);

    double t_x = curr_x_;
    double t_y = 0.01 * std::pow(t_x, 3) + 0.1 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
    if (random) {
        unsigned int seed = time(NULL);
        t_x += (rand_r(&seed) % 20 - 10.0) / 10.0;
        t_y += (rand_r(&seed) % 20 - 10.0) / 10.0;
    }
    vehicle_state_.set_x(t_x);
    vehicle_state_.set_y(t_y);
    vehicle_state_.set_z(0.1);
    curr_x_ += 0.2;

    t_y = 0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2;  // yaw
    t_y = NormalizeAngle(t_y);                        // -M_PI_2
    vehicle_state_.set_heading(t_y);

    vehicle_state_.set_linear_velocity(1.4);
    vehicle_state_.set_steering_percentage(0.1);
}

void ControlEvaluatorTest::InitPlanning() {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    planning_msg_.mutable_header()->set_timestamp_sec(curr_time);
    planning_msg_.mutable_header()->set_module_name("planning");
    planning_msg_.mutable_header()->set_sequence_num(1);
    // v = 1.2*sin(t) + 1.4
    planning_msg_.set_total_path_time(8.0);
    planning_msg_.mutable_estop()->set_is_estop(false);
    planning_msg_.set_is_replan(false);
    planning_msg_.set_gear(pnc::chassis::GEAR_DRIVE);
    double t_x = vehicle_state_.x() - 1.0;
    double accu_s = 0.0;
    double init_v = 1.4;
    for (double t = 0.0; t <= 8.0; t += 0.02) {
        double v = 1.2 * std::sin(t) + init_v;
        double a = 1.2 * std::cos(t);
        auto* point = planning_msg_.add_trajectory_point();
        t_x += v * 0.02;
        point->mutable_path_point()->set_x(t_x);
        double t_y = 0.01 * std::pow(t_x, 3) + 0.1 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
        point->mutable_path_point()->set_y(t_y);
        point->set_v(v);
        point->set_a(a);
        // curv
        t_y = (0.06 * t_x + 0.2) / std::pow(1.0 + std::pow(0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2, 2), 1.5);
        point->mutable_path_point()->set_kappa(t_y);
        point->set_relative_time(t);
        t_y = 0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2;
        t_y = NormalizeAngle(t_y);
        point->mutable_path_point()->set_theta(t_y);
        point->mutable_path_point()->set_s(accu_s);
        accu_s += v * 0.02;
    }
}

TEST_F(ControlEvaluatorTest, Test1) {
    // reserved
    ControlEvaluator evaluator;
    EvaluationInfo eva_info = evaluator.UpdateEvaluationDebug(planning_msg_, vehicle_state_);

    EXPECT_NEAR(eva_info.lon_debug.station_error(), -0.9329, 0.01);  // ref - self
    EXPECT_NEAR(eva_info.lon_debug.speed_error(), 0.0239, 0.01);
    EXPECT_NEAR(eva_info.lon_debug.station_target(), 0.0278, 0.01);
    EXPECT_NEAR(eva_info.lon_debug.speed_target(), 1.4239, 0.01);  // init speed

    EXPECT_NEAR(eva_info.lat_debug.lateral_error_by_s(), 0.0, 0.01);
    EXPECT_NEAR(eva_info.lat_debug.heading_error_by_s(), 0.0, 0.01);
}

}  // namespace control
}  // namespace jiduauto
