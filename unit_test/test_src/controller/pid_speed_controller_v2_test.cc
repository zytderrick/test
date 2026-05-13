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

#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "control/src/common/control_gflags.h"
// #include "control/src/common/control_utility.h"
#include "control/src/common/file_util.h"
#include "control/src/feature/control_message_manager.h"
#include "control/src/feature/virtual_sensors.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/pnc_utility.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::math::NormalizeAngle;
using jiduauto::pnc::util::TimeUtility;

class PidSpeedControllerV2Test : public ::testing::Test {
public:
    virtual void SetUp() {
        CHECK(FileUtil::ReadPnCPath());
        std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));
        InitLocalization();
        InitChassis();
        InitPlanning();
        InitControlInputStream();
    }

    void InitLocalization(bool random = false);
    void InitChassis(bool update = false);
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

void PidSpeedControllerV2Test::InitLocalization(bool random) {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    localization_msg_.mutable_loc_pose()->set_measurement_time(curr_time);
    localization_msg_.mutable_header()->set_timestamp_sec(curr_time);
    localization_msg_.mutable_header()->set_module_name("localization");
    localization_msg_.mutable_header()->set_sequence_num(1);

    double t_x = curr_x_;
    double t_y = 0.01 * std::pow(t_x, 3) + 0.1 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
    if (random) {
        unsigned int seed = time(NULL);
        t_x += (rand_r(&seed) % 20 - 10.0) / 100.0;
        t_y += (rand_r(&seed) % 20 - 10.0) / 100.0;
    }
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_x(t_x);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_y(t_y);
    localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_z(0.1);

    double t_r = 0.0;                                 // roll
    double t_p = 0.0;                                 // pitch
    t_y = 0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2;  // yaw
    t_y = NormalizeAngle(t_y);                        //- M_PI_2
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

    // copy local pose
    localization_msg_.mutable_loc_pose()->mutable_local_pose()->CopyFrom(localization_msg_.loc_pose().pose());
}

void PidSpeedControllerV2Test::InitChassis(bool update) {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    chassis_msg_.mutable_header()->set_timestamp_sec(curr_time);
    chassis_msg_.mutable_header()->set_module_name("chassis");
    chassis_msg_.mutable_header()->set_sequence_num(1);
    chassis_msg_.set_engine_started(true);
    chassis_msg_.set_speed_mps(0.8);
    chassis_msg_.set_odometer_m(100.2);
    chassis_msg_.set_throttle_percentage(5.2);
    chassis_msg_.set_brake_percentage(0.0);
    chassis_msg_.set_steering_percentage(0.1);
    chassis_msg_.set_steering_angle_spd(100.0);
    chassis_msg_.set_steering_torque_nm(1.5);
    chassis_msg_.set_chassis_lon_acc(0.0);
    chassis_msg_.set_parking_brake(false);
    chassis_msg_.set_driving_mode(pnc::chassis::COMPLETE_AUTO_DRIVE);
    chassis_msg_.set_gear_location(pnc::chassis::GEAR_DRIVE);
    chassis_msg_.set_error_code(pnc::chassis::ErrorCode::NO_ERROR);

    if (update) {
        double acc_cmd = control_command_stream_.control_debug.pid_speed_debug().lon_context().accel_cmd();
        double throttle_cmd = control_command_stream_.control_debug.pid_speed_debug().lon_context().throttle_cmd();
        double brake_cmd = control_command_stream_.control_debug.pid_speed_debug().lon_context().brake_cmd();
        double curr_speed = control_command_stream_.control_debug.pid_speed_debug().lon_context().speed_target();
        chassis_msg_.set_speed_mps(curr_speed);
        chassis_msg_.set_chassis_lon_acc(acc_cmd);
        chassis_msg_.set_throttle_percentage(throttle_cmd);
        chassis_msg_.set_brake_percentage(brake_cmd);
    }
}

void PidSpeedControllerV2Test::InitPlanning() {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    planning_msg_.mutable_header()->set_timestamp_sec(curr_time);
    planning_msg_.mutable_header()->set_module_name("planning");
    planning_msg_.mutable_header()->set_sequence_num(1);
    // v = 1.2*sin(t) + 1.4
    planning_msg_.set_total_path_time(8.0);
    // planning_msg_.set_estop(false);
    planning_msg_.set_is_replan(false);
    planning_msg_.set_gear(pnc::chassis::GEAR_DRIVE);
    double t_x = localization_msg_.loc_pose().pose().position().x() - 1.0;
    double accu_s = 0.0;
    for (double t = 0.0; t <= 8.0; t += 0.02) {
        double v = 0.5 * std::cos(t) + 3.6;
        double a = -0.5 * std ::sin(t) + 1.2;
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

void PidSpeedControllerV2Test::InitControlInputStream() {
    control_input_stream_.input_stream.localization = localization_msg_;
    control_input_stream_.input_stream.chassis = chassis_msg_;
    control_input_stream_.input_stream.planning_info = planning_msg_;
    virtual_sensors_.Init(control_conf_);
    virtual_sensors_.Update(control_input_stream_.input_stream, &control_command_stream_.control_debug);
    control_input_stream_.vehicle_state = virtual_sensors_.GetSenseView();
}

TEST_F(PidSpeedControllerV2Test, Test1) {
    PidSpeedControllerV2 pid_controller;

    bool bflag = pid_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    if (!bflag) return;

    bflag = pid_controller.ControlMain(control_input_stream_, &control_command_stream_);
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    EXPECT_GT(control_command_stream_.control_command.acceleration(), 1e-5);
    EXPECT_GT(control_command_stream_.control_command.throttle(), 1.0);
    EXPECT_NEAR(control_command_stream_.control_command.brake(), 0.0, 0.1);
    EXPECT_GT(std::fabs(control_command_stream_.control_debug.pid_speed_debug().lon_context().speed_error()), 1e-1);
    EXPECT_GT(std::fabs(control_command_stream_.control_debug.pid_speed_debug().lon_context().station_error()), 1e-1);
}

TEST_F(PidSpeedControllerV2Test, Test2) {
    PidSpeedControllerV2 pid_controller;

    bool bflag = pid_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    if (!bflag) return;

    InitLocalization(false);
    chassis_msg_.set_speed_mps(1.8);
    InitControlInputStream();

    bflag = pid_controller.ControlMain(control_input_stream_, &control_command_stream_);
    EXPECT_TRUE(bflag == true);
    if (!bflag) return;

    EXPECT_GT(control_command_stream_.control_command.acceleration(), 1e-5);
    EXPECT_GT(control_command_stream_.control_command.throttle(), 1.0);
    EXPECT_NEAR(control_command_stream_.control_command.brake(), 0.0, 0.1);
    EXPECT_GT(std::fabs(control_command_stream_.control_debug.pid_speed_debug().lon_context().speed_error()), 1e-1);
    EXPECT_GT(std::fabs(control_command_stream_.control_debug.pid_speed_debug().lon_context().station_error()), 0.0);
}

TEST_F(PidSpeedControllerV2Test, Test3) {
    PidSpeedControllerV2 pid_controller;

    bool bflag = pid_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    if (!bflag) return;

    chassis_msg_.set_speed_mps(0.8);
    for (int i = 0; i < 50; ++i) {
        InitLocalization();
        InitControlInputStream();
        bflag = pid_controller.ControlMain(control_input_stream_, &control_command_stream_);
        EXPECT_TRUE(bflag == true);
        if (!bflag) return;

        EXPECT_GT(control_command_stream_.control_command.acceleration(), 1e-5);
        EXPECT_GT(control_command_stream_.control_command.throttle(), 1.0);
        EXPECT_NEAR(control_command_stream_.control_command.brake(), 0.0, 0.1);
        EXPECT_GT(std::fabs(control_command_stream_.control_debug.pid_speed_debug().lon_context().speed_error()), 1e-1);
        EXPECT_GT(std::fabs(control_command_stream_.control_debug.pid_speed_debug().lon_context().station_error()),
                  1e-1);
    }
}

TEST_F(PidSpeedControllerV2Test, Test4) {
    PidSpeedControllerV2 pid_controller;

    bool bflag = pid_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    if (!bflag) return;

    for (int i = 0; i < 50; ++i) {
        InitLocalization(false);
        chassis_msg_.set_speed_mps(1.8);
        if (i % 5 == 0) {
            InitPlanning();
        }
        InitControlInputStream();
        bflag = pid_controller.ControlMain(control_input_stream_, &control_command_stream_);
        EXPECT_TRUE(bflag == true);
        if (!bflag)
            return;
        else
            InitChassis(bflag);

        EXPECT_GT(std::fabs(control_command_stream_.control_command.acceleration()), 1e-5);
        EXPECT_GT(std::fabs(control_command_stream_.control_debug.pid_speed_debug().lon_context().speed_error()), 1e-1);
        EXPECT_GT(std::fabs(control_command_stream_.control_debug.pid_speed_debug().lon_context().station_error()),
                  0.0);
    }
}

}  // namespace control
}  // namespace jiduauto
