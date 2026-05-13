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

#include "control/src/control_regulator_v2.h"

#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/common/file_util.h"
#include "control/src/math/control_math.h"
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
using pnc::chassis::Chassis;

class ControlRegulatorV2Test : public ::testing::Test {
public:
    virtual void SetUp() {
        CHECK(FileUtil::ReadPnCPath());
        std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));

        control_conf_file = FLAGS_control_conf_path + FLAGS_control_chassis_test_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file,
                                                  control_conf_.mutable_chassis_test_controller_conf()))
            << "ReadPbFromTextFile failed: " << control_conf_file;

        InitLocalization();
        InitChassis();
        InitPlanning();
    }

    void InitLocalization(bool random = false);
    void InitChassis();
    void InitPlanning();

protected:
    pnc::control::ControlConfig control_conf_;
    pnc::localization::LocalizationEstimate localization_msg_;
    pnc::chassis::Chassis chassis_msg_;
    pnc::planning::ADCTrajectory planning_msg_;
    pnc::control::ControlCommand cmd_msg_;

    double curr_x_{0.2};
};

void ControlRegulatorV2Test::InitLocalization(bool random) {
    double curr_time = 0.0;
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

void ControlRegulatorV2Test::InitChassis() {
    double curr_time = 0.0;
    chassis_msg_.mutable_header()->set_timestamp_sec(curr_time);
    chassis_msg_.mutable_header()->set_module_name("chassis");
    chassis_msg_.mutable_header()->set_sequence_num(1);
    chassis_msg_.set_engine_started(true);
    chassis_msg_.set_speed_mps(1.2);
    chassis_msg_.set_odometer_m(100.2);
    chassis_msg_.set_throttle_percentage(5.2);
    chassis_msg_.set_brake_percentage(0.0);
    chassis_msg_.set_steering_percentage(0.1);
    chassis_msg_.set_steering_angle_spd(0.0);
    chassis_msg_.set_steering_torque_nm(0.0);
    chassis_msg_.set_chassis_lon_acc(0.0);
    chassis_msg_.set_parking_brake(false);
    chassis_msg_.set_driving_mode(pnc::chassis::COMPLETE_AUTO_DRIVE);
    chassis_msg_.set_gear_location(pnc::chassis::GEAR_DRIVE);
    chassis_msg_.set_error_code(pnc::chassis::ErrorCode::NO_ERROR);
}

void ControlRegulatorV2Test::InitPlanning() {
    double curr_time = 0.0;
    planning_msg_.mutable_header()->set_timestamp_sec(curr_time);
    planning_msg_.mutable_header()->set_module_name("planning");
    planning_msg_.mutable_header()->set_sequence_num(1);
    // v = 1.2*sin(t) + 1.4
    planning_msg_.set_total_path_time(8.0);
    planning_msg_.mutable_estop()->set_is_estop(false);
    planning_msg_.set_is_replan(false);
    planning_msg_.set_gear(pnc::chassis::GEAR_DRIVE);
    double t_x = localization_msg_.loc_pose().pose().position().x() - 1.0;
    double accu_s = 0.0;
    for (double t = 0.0; t <= 8.0; t += 0.02) {
        double v = 1.2 * std::sin(t) + 1.4;
        double a = 1.2 * std::cos(t);
        auto* point = planning_msg_.add_trajectory_point();
        t_x += v * 0.02;
        point->mutable_path_point()->set_x(t_x);
        // Note(tan.ju): To pass function CurvatureCheckPassed, change 0.01 to 0.001, change 0.1 to 0.05
        double t_y = 0.001 * std::pow(t_x, 3) + 0.05 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
        // double t_y = 0.01 * std::pow(t_x, 3) + 0.1 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
        point->mutable_path_point()->set_y(t_y);
        point->set_v(v);
        point->set_a(a);
        // curv
        // Note(tan.ju): To pass function CurvatureCheckPassed, change 0.06 to 0.006, change 0.2 to 0.1
        t_y =
            std::abs(0.006 * t_x + 0.1) / std::pow(1.0 + std::pow(0.003 * std::pow(t_x, 2) + 0.1 * t_x - 0.2, 2), 1.5);
        // t_y = (0.06 * t_x + 0.2) / std::pow(1.0 + std::pow(0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2, 2), 1.5);
        point->mutable_path_point()->set_kappa(t_y);
        point->set_relative_time(t);
        t_y = 0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2;
        t_y = NormalizeAngle(t_y);
        point->mutable_path_point()->set_theta(t_y);
        point->mutable_path_point()->set_s(accu_s);
        accu_s += v * 0.02;
    }
}

// test with perfect position
TEST_F(ControlRegulatorV2Test, Test1) {
    // reserved
    ControlRegulatorV2 controller_regulator(control_conf_);
    bool bflag = controller_regulator.Init();
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    std::shared_ptr<pnc::chassis::Chassis> chassis_ptr = std::make_shared<pnc::chassis::Chassis>(chassis_msg_);
    std::shared_ptr<pnc::localization::LocalizationEstimate> localization_ptr =
        std::make_shared<pnc::localization::LocalizationEstimate>(localization_msg_);
    std::shared_ptr<pnc::planning::ADCTrajectory> planning_ptr =
        std::make_shared<pnc::planning::ADCTrajectory>(planning_msg_);

    controller_regulator.SaveChassisMessage(chassis_ptr);
    controller_regulator.SaveLocalizationMessage(localization_ptr);
    controller_regulator.SavePlanningMessage(planning_ptr);

    bflag = controller_regulator.IterationRoutine();
    EXPECT_TRUE(bflag);
    if (!bflag) return;

    auto cmd_ptr = controller_regulator.GetControlCommand();
    EXPECT_TRUE(cmd_ptr != nullptr);
    EXPECT_GT(cmd_ptr->header().timestamp_sec(), 0.1);
    EXPECT_LE(cmd_ptr->steering_target(), 0.0);
    EXPECT_NEAR(cmd_ptr->debug().urban_lat_debug().lateral_error(), 0.0, 0.3);
    EXPECT_NEAR(cmd_ptr->debug().urban_lat_debug().heading_error(), 0.0, 0.2);
    EXPECT_GE(cmd_ptr->acceleration(), -3.0);
    EXPECT_NEAR(cmd_ptr->throttle(), 0.0, 0.01);
    EXPECT_LT(cmd_ptr->brake(), 0.0);
    EXPECT_GE(cmd_ptr->debug().urban_lon_debug().speed_error(), 0.0);
    EXPECT_NEAR(cmd_ptr->debug().urban_lon_debug().station_error(), -0.9329, 1.0);
}

// test with random position
TEST_F(ControlRegulatorV2Test, Test2) {
    InitLocalization(false);
    ControlRegulatorV2 controller_regulator(control_conf_);
    bool bflag = controller_regulator.Init();
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    std::shared_ptr<pnc::chassis::Chassis> chassis_ptr = std::make_shared<pnc::chassis::Chassis>(chassis_msg_);
    std::shared_ptr<pnc::localization::LocalizationEstimate> localization_ptr =
        std::make_shared<pnc::localization::LocalizationEstimate>(localization_msg_);
    std::shared_ptr<pnc::planning::ADCTrajectory> planning_ptr =
        std::make_shared<pnc::planning::ADCTrajectory>(planning_msg_);

    controller_regulator.SaveChassisMessage(chassis_ptr);
    controller_regulator.SaveLocalizationMessage(localization_ptr);
    controller_regulator.SavePlanningMessage(planning_ptr);

    bflag = controller_regulator.IterationRoutine();
    auto cmd_ptr = controller_regulator.GetControlCommand();
    EXPECT_TRUE(cmd_ptr != nullptr);
}

// test with perfect position, loop
// expect donot core dump
TEST_F(ControlRegulatorV2Test, Test3) {
    ControlRegulatorV2 controller_regulator(control_conf_);
    bool bflag = controller_regulator.Init();
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    curr_x_ = 0.2;
    for (int i = 0; i < 50; ++i) {
        InitLocalization();
        std::shared_ptr<pnc::chassis::Chassis> chassis_ptr = std::make_shared<pnc::chassis::Chassis>(chassis_msg_);
        std::shared_ptr<pnc::localization::LocalizationEstimate> localization_ptr =
            std::make_shared<pnc::localization::LocalizationEstimate>(localization_msg_);
        std::shared_ptr<pnc::planning::ADCTrajectory> planning_ptr =
            std::make_shared<pnc::planning::ADCTrajectory>(planning_msg_);

        controller_regulator.SaveChassisMessage(chassis_ptr);
        controller_regulator.SaveLocalizationMessage(localization_ptr);
        controller_regulator.SavePlanningMessage(planning_ptr);

        bflag = controller_regulator.IterationRoutine();
        EXPECT_TRUE(bflag);
        if (!bflag) return;
        auto cmd_ptr = controller_regulator.GetControlCommand();
        EXPECT_TRUE(cmd_ptr != nullptr);
    }
}

// test with random position, loop
// expect donot core dump
TEST_F(ControlRegulatorV2Test, Test4) {
    ControlRegulatorV2 controller_regulator(control_conf_);
    bool bflag = controller_regulator.Init();
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    curr_x_ = 0.2;
    for (int i = 0; i < 50; ++i) {
        InitLocalization(false);
        std::shared_ptr<pnc::chassis::Chassis> chassis_ptr = std::make_shared<pnc::chassis::Chassis>(chassis_msg_);
        std::shared_ptr<pnc::localization::LocalizationEstimate> localization_ptr =
            std::make_shared<pnc::localization::LocalizationEstimate>(localization_msg_);
        std::shared_ptr<pnc::planning::ADCTrajectory> planning_ptr =
            std::make_shared<pnc::planning::ADCTrajectory>(planning_msg_);

        controller_regulator.SaveChassisMessage(chassis_ptr);
        controller_regulator.SaveLocalizationMessage(localization_ptr);
        controller_regulator.SavePlanningMessage(planning_ptr);

        bflag = controller_regulator.IterationRoutine();
        auto cmd_ptr = controller_regulator.GetControlCommand();
        EXPECT_TRUE(cmd_ptr != nullptr);
    }
}

// test log_csv_file, open it manually
TEST_F(ControlRegulatorV2Test, Test5) {
    control_conf_.set_enable_csv_debug(false);
    ControlRegulatorV2 controller_regulator(control_conf_);
    bool bflag = controller_regulator.Init();
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    curr_x_ = 0.2;
    for (int i = 0; i < 200; ++i) {
        InitLocalization(false);
        std::shared_ptr<pnc::chassis::Chassis> chassis_ptr = std::make_shared<pnc::chassis::Chassis>(chassis_msg_);
        std::shared_ptr<pnc::localization::LocalizationEstimate> localization_ptr =
            std::make_shared<pnc::localization::LocalizationEstimate>(localization_msg_);
        std::shared_ptr<pnc::planning::ADCTrajectory> planning_ptr =
            std::make_shared<pnc::planning::ADCTrajectory>(planning_msg_);

        controller_regulator.SaveChassisMessage(chassis_ptr);
        controller_regulator.SaveLocalizationMessage(localization_ptr);
        controller_regulator.SavePlanningMessage(planning_ptr);

        bflag = controller_regulator.IterationRoutine();
        auto cmd_ptr = controller_regulator.GetControlCommand();
        EXPECT_TRUE(cmd_ptr != nullptr);
    }
}

// test chassis_test_mode
TEST_F(ControlRegulatorV2Test, Test6) {
    auto bak_lat = control_conf_.controller_config().lat_init();
    control_conf_.mutable_controller_config()->set_lat_init(pnc::control::ControlConfig::CHASSIS_TEST_CONTROLLER);
    ControlRegulatorV2 controller_regulator(control_conf_);
    bool bflag = controller_regulator.Init();
    EXPECT_TRUE(bflag);
    if (!bflag) return;
    curr_x_ = 0.2;
    for (int i = 0; i < 50; ++i) {
        std::shared_ptr<pnc::chassis::Chassis> chassis_ptr = std::make_shared<pnc::chassis::Chassis>(chassis_msg_);

        controller_regulator.SaveChassisMessage(chassis_ptr);

        bflag = controller_regulator.IterationRoutine();
        auto cmd_ptr = controller_regulator.GetControlCommand();
        EXPECT_TRUE(cmd_ptr != nullptr);
    }
    control_conf_.mutable_controller_config()->set_lat_init(bak_lat);
}

}  // namespace control
}  // namespace jiduauto
