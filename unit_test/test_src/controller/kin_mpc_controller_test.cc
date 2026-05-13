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

// Copyright(C) 2022, JiDu Auto. All rights reserved.
// Author: Bolin ZHAO (bolin.zhao@jiduauto.com)

#include "control/src/controller/kin_mpc_controller.h"

#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include <utility>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/common/file_util.h"
#include "control/src/feature/virtual_sensors.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/pnc_utility.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"
#include "pnc_control_config.pb.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::math::NormalizeAngle;
using jiduauto::pnc::util::TimeUtility;

class KinMpcControllerTest : public ::testing::Test {
public:
    virtual void SetUp() {
        std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));
        InitLocalization();
        InitChassis();
        InitPlanning();
        InitControlInputStream();

        virtual_sensors_.Init(control_conf_);
        virtual_sensors_.Update(control_input_stream_.input_stream, &control_command_stream_.control_debug);
        control_input_stream_.vehicle_state = virtual_sensors_.GetSenseView();
    }

    void InitLocalization(bool random = false);
    void InitChassis();
    void InitPlanning();
    void InitControlInputStream();

protected:
    pnc::control::ControlConfig control_conf_;
    pnc::localization::LocalizationEstimate localization_msg_;
    pnc::chassis::Chassis chassis_msg_;
    pnc::planning::ADCTrajectory planning_msg_;
    pnc::control::ControlCommand cmd_msg_;
    ControlInputStream control_input_stream_;
    ControlCommandStream control_command_stream_;
    VirtualSensors virtual_sensors_;

    double curr_x_{0.2};
};

void KinMpcControllerTest::InitLocalization(bool random) {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    localization_msg_.mutable_header()->set_timestamp_sec(curr_time);
    localization_msg_.mutable_header()->set_module_name("localization");
    localization_msg_.mutable_header()->set_sequence_num(1);

    double t_x = curr_x_;
    double t_y = 0.01 * std::pow(t_x, 3) + 0.1 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
    if (random) {
        unsigned int seed = time(NULL);
        t_x += (rand_r(&seed) % 20 - 10.0) / 10.0;
        t_y += (rand_r(&seed) % 20 - 10.0) / 10.0;
    }
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_x(t_x);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_y(t_y);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_z(0.1);
    curr_x_ += 0.2;

    double t_r = 0.0;  // roll
    double t_p = 0.0;  // pitch
    // t_y = 0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2; // yaw
    t_y = 0.01;  // yaw
    t_y = NormalizeAngle(t_y - M_PI_2);
    EulerAnglesZXYf q(t_r, t_p, t_y);
    Eigen::Quaternion<float> qua = q.ToQuaternion();
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qx(qua.x());
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qy(qua.y());
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qz(qua.z());
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qw(qua.w());

    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_x(0.5);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_y(0.5);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_z(0.1);

    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_z(0.0);

    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_x(0.0);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_y(0.0);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_z(0.0);

    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_x(0.0);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_y(0.0);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_z(0.0);
}

void KinMpcControllerTest::InitChassis() {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    chassis_msg_.mutable_header()->set_timestamp_sec(curr_time);
    chassis_msg_.mutable_header()->set_module_name("chassis");
    chassis_msg_.mutable_header()->set_sequence_num(1);
    chassis_msg_.set_engine_started(true);
    chassis_msg_.set_speed_mps(1.2);
    chassis_msg_.set_odometer_m(100.2);
    chassis_msg_.set_throttle_percentage(5.2);
    chassis_msg_.set_brake_percentage(0.0);
    chassis_msg_.set_steering_percentage(0.1);
    chassis_msg_.set_parking_brake(false);
    chassis_msg_.set_steering_angle_spd(0.0);
    chassis_msg_.set_steering_torque_nm(0.0);
    chassis_msg_.set_chassis_lon_acc(0.0);
    chassis_msg_.set_driving_mode(pnc::chassis::COMPLETE_AUTO_DRIVE);
    chassis_msg_.set_gear_location(pnc::chassis::GEAR_DRIVE);
    chassis_msg_.set_error_code(pnc::chassis::ErrorCode::NO_ERROR);
}

void KinMpcControllerTest::InitPlanning() {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    planning_msg_.mutable_header()->set_timestamp_sec(curr_time);
    planning_msg_.mutable_header()->set_module_name("planning");
    planning_msg_.mutable_header()->set_sequence_num(1);
    // v = 1.2*sin(t) + 1.2
    planning_msg_.set_total_path_time(8.0);
    planning_msg_.mutable_estop()->set_is_estop(false);
    planning_msg_.set_is_replan(false);
    planning_msg_.set_gear(pnc::chassis::GEAR_DRIVE);
    double t_x = localization_msg_.loc_pose().pose().position().x() - 0.05;
    double accu_s = 0.0;
    for (double t = 0.0; t <= 8.0; t += 0.02) {
        double v = 1.2 * std::sin(t) + 1.2;
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

void KinMpcControllerTest::InitControlInputStream() {
    control_input_stream_.input_stream.localization = localization_msg_;
    control_input_stream_.input_stream.planning_info = planning_msg_;
    control_input_stream_.input_stream.chassis = chassis_msg_;
}

// test with perfect position
TEST_F(KinMpcControllerTest, Test1) {
    // reserved
    KinMpcController kin_mpc_controller;

    kin_mpc_controller.Init(control_conf_);
    bool bflag = kin_mpc_controller.ControlMain(control_input_stream_, &control_command_stream_);
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    EXPECT_NEAR(control_command_stream_.control_command.steering_target(), 8.0, 0.1);
    EXPECT_GT(control_command_stream_.control_command.acceleration(), -4.0);
    EXPECT_NEAR(control_command_stream_.control_command.throttle(), 0.0, 0.1);
    EXPECT_GT(control_command_stream_.control_command.brake(), 0.0);
}

// test with point mode
TEST_F(KinMpcControllerTest, Test2) {
    // reserved
    KinMpcController kin_mpc_controller;

    kin_mpc_controller.Init(control_conf_);
    bool bflag = kin_mpc_controller.ControlMain(control_input_stream_, &control_command_stream_);
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    EXPECT_NEAR(control_command_stream_.control_command.steering_target(), 8.0, 0.1);
    EXPECT_GT(control_command_stream_.control_command.acceleration(), -4.0);
    EXPECT_NEAR(control_command_stream_.control_command.throttle(), 0.0, 0.1);
    EXPECT_GT(control_command_stream_.control_command.brake(), 0.0);
}

// // test with random position
// TEST_F(KinMpcControllerTest, Test2) {
//   // reserved
//   KinMpcController kin_mpc_controller;
//   kin_mpc_controller.Init(control_conf_);
//   InitLocalization(true);
//   chassis_msg_.set_speed_mps(5.8);
//   bool bflag = kin_mpc_controller.Control(&localization_msg_, &chassis_msg_,
//                                           &planning_msg_, &cmd_msg_);
//   EXPECT_TRUE(bflag);
//   if (!bflag)
//     return;
// }

// // test with perfect position, loop
// // expect donot core dump
// TEST_F(KinMpcControllerTest, Test3) {
//   // reserved
//   KinMpcController kin_mpc_controller;
//   kin_mpc_controller.Init(control_conf_);
//   chassis_msg_.set_speed_mps(0.8);
//   curr_x_ = 0.2;
//   for (int i = 0; i < 50; ++i) {
//     InitLocalization();
//     bool bflag = kin_mpc_controller.Control(&localization_msg_,
//     &chassis_msg_,
//                                             &planning_msg_, &cmd_msg_);
//     EXPECT_TRUE(bflag);
//     if (!bflag)
//       return;
//   }
// }

// // test with random position, loop
// // expect donot core dump
// TEST_F(KinMpcControllerTest, Test4) {
//   // reserved
//   KinMpcController kin_mpc_controller;
//   kin_mpc_controller.Init(control_conf_);
//   curr_x_ = 0.2;
//   for (int i = 0; i < 50; ++i) {
//     InitLocalization(true);
//     bool bflag = kin_mpc_controller.Control(&localization_msg_,
//     &chassis_msg_,
//                                             &planning_msg_, &cmd_msg_);
//     EXPECT_TRUE(bflag);
//     if (!bflag)
//       return;
//   }
// }

// // test log_csv_file, open it manually
// TEST_F(KinMpcControllerTest, Test5) {
//   // reserved
//   KinMpcController kin_mpc_controller;
//   control_conf_.set_enable_csv_debug(false);
//   kin_mpc_controller.Init(control_conf_);
//   curr_x_ = 0.2;
//   for (int i = 0; i < 200; ++i) {
//     InitLocalization(true);
//     kin_mpc_controller.Control(&localization_msg_, &chassis_msg_,
//                                &planning_msg_, &cmd_msg_);
//   }
// }

}  // namespace control
}  // namespace jiduauto
