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

#include "control/src/controller/chassis_test_controller.h"

#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/common/file_util.h"
#include "control/src/math/control_math.h"
#include "pnc_chassis_test_config.pb.h"
#include "pnc_common/src/utility/pnc_utility.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"
#include "pnc_common_vehicle_param.pb.h"
#include "pnc_control_config.pb.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::util::TimeUtility;
using pnc::chassis::Chassis;
using pnc::control::ControlCommand;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

class ChassisTestControllerTest : public ::testing::Test {
public:
    virtual void SetUp() {
        CHECK(FileUtil::ReadPnCPath());
        std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));
        control_conf_file = FLAGS_control_conf_path + FLAGS_control_chassis_test_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file,
                                                  control_conf_.mutable_chassis_test_controller_conf()))
            << "ReadPbFromTextFile failed: " << control_conf_file;
        vehicle_param_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();
        InitChassis();
    }

    void InitChassis();
    void InitControlInputStream();
    void UpdateChassis();
    void UpdateControlInputStream();

protected:
    pnc::control::ControlConfig control_conf_;
    pnc::common::VehicleParam vehicle_param_;
    LocalizationEstimate localization_msg_;
    ADCTrajectory planning_msg_;
    Chassis chassis_msg_;
    ControlCommand cmd_msg_;
    ControlInputStream control_input_stream_;
    ControlCommandStream control_command_stream_;
    double prev_time_{0.0};
};

void ChassisTestControllerTest::InitChassis() {
    double curr_time = TimeUtility::GetCurrentTimesecond();
    chassis_msg_.mutable_header()->set_timestamp_sec(curr_time);
    chassis_msg_.mutable_header()->set_module_name("chassis");
    chassis_msg_.mutable_header()->set_sequence_num(1);
    chassis_msg_.set_engine_started(true);
    chassis_msg_.set_speed_mps(0.0);
    chassis_msg_.set_odometer_m(0.0);
    chassis_msg_.set_throttle_percentage(0.0);
    chassis_msg_.set_brake_percentage(0.0);
    chassis_msg_.set_steering_percentage(0.0);
    chassis_msg_.set_parking_brake(false);
    chassis_msg_.set_steering_angle_spd(0.0);
    chassis_msg_.set_steering_torque_nm(0.0);
    chassis_msg_.set_chassis_lon_acc(0.0);
    chassis_msg_.set_driving_mode(pnc::chassis::COMPLETE_MANUAL);
    chassis_msg_.set_gear_location(pnc::chassis::GEAR_PARKING);
    chassis_msg_.set_error_code(pnc::chassis::ErrorCode::NO_ERROR);
    prev_time_ = curr_time;
}

void ChassisTestControllerTest::InitControlInputStream() { control_input_stream_.input_stream.chassis = chassis_msg_; }

void ChassisTestControllerTest::UpdateChassis() {
    static int num = 1;
    double curr_time = TimeUtility::GetCurrentTimesecond();
    chassis_msg_.mutable_header()->set_timestamp_sec(curr_time);
    chassis_msg_.mutable_header()->set_module_name("chassis");
    chassis_msg_.mutable_header()->set_sequence_num(++num);
    chassis_msg_.set_speed_mps(
        std::max(0., chassis_msg_.speed_mps() + cmd_msg_.acceleration() * (curr_time - prev_time_)));
    chassis_msg_.set_steering_percentage(100 * cmd_msg_.steering_target() / vehicle_param_.max_steer_angle() * 180.0 /
                                         M_PI);
    chassis_msg_.set_driving_mode(cmd_msg_.driving_mode());
    chassis_msg_.set_gear_location(cmd_msg_.gear_location());
    prev_time_ = curr_time;
}

void ChassisTestControllerTest::UpdateControlInputStream() {
    control_input_stream_.input_stream.chassis = chassis_msg_;
}

TEST_F(ChassisTestControllerTest, Test1) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LON_STANDSTILL_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.Control(&localization_msg_, &chassis_msg_, &planning_msg_, &cmd_msg_);
        UpdateChassis();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test2) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LON_IDLESPEED_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.Control(&localization_msg_, &chassis_msg_, &planning_msg_, &cmd_msg_);
        UpdateChassis();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test3) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(pnc::control::ChassisTestConf::LON_STEP_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.Control(&localization_msg_, &chassis_msg_, &planning_msg_, &cmd_msg_);
        UpdateChassis();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test4) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LON_SINUSOID_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.Control(&localization_msg_, &chassis_msg_, &planning_msg_, &cmd_msg_);
        UpdateChassis();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test5) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LAT_STANDSTILL_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.Control(&localization_msg_, &chassis_msg_, &planning_msg_, &cmd_msg_);
        UpdateChassis();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test6) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(pnc::control::ChassisTestConf::LAT_STEP_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.Control(&localization_msg_, &chassis_msg_, &planning_msg_, &cmd_msg_);
        UpdateChassis();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test7) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LAT_SINUSOID_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.Control(&localization_msg_, &chassis_msg_, &planning_msg_, &cmd_msg_);
        UpdateChassis();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test2_1) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LON_STANDSTILL_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.ControlMain(control_input_stream_, &control_command_stream_);
        UpdateChassis();
        UpdateControlInputStream();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test2_2) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LON_IDLESPEED_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.ControlMain(control_input_stream_, &control_command_stream_);
        UpdateChassis();
        UpdateControlInputStream();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test2_3) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(pnc::control::ChassisTestConf::LON_STEP_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.ControlMain(control_input_stream_, &control_command_stream_);
        UpdateChassis();
        UpdateControlInputStream();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test2_4) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LON_SINUSOID_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.ControlMain(control_input_stream_, &control_command_stream_);
        UpdateChassis();
        UpdateControlInputStream();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test2_5) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LAT_STANDSTILL_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.ControlMain(control_input_stream_, &control_command_stream_);
        UpdateChassis();
        UpdateControlInputStream();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test2_6) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(pnc::control::ChassisTestConf::LAT_STEP_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.ControlMain(control_input_stream_, &control_command_stream_);
        UpdateChassis();
        UpdateControlInputStream();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

TEST_F(ChassisTestControllerTest, Test2_7) {
    control_conf_.mutable_chassis_test_controller_conf()->set_enable_test(
        pnc::control::ChassisTestConf::LAT_SINUSOID_TEST);
    ChassisTestController chassis_test_controller;
    bool bflag = chassis_test_controller.Init(control_conf_);
    EXPECT_TRUE(bflag);
    for (int i = 0; i < 100; ++i) {
        bflag = chassis_test_controller.ControlMain(control_input_stream_, &control_command_stream_);
        UpdateChassis();
        UpdateControlInputStream();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        usleep(200);
    }
}

}  // namespace control
}  // namespace jiduauto
