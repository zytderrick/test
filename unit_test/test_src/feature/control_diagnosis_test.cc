// /**
//  * Copyright @ 2021 - 2023 JIDU AUTO CO.,LTD.
//  * All Rights Reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are NOT permitted except as agreed by
//  * JIDU AUTO CO.,LTD.
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  */

// // Copyright 2022 JiDuAuto LLC
// // Author: Bolin ZHAO, Jidu PnC

// // #include "control/src/feature/control_diagnosis.h"

// #include <gtest/gtest.h>
// #include <stdlib.h>

// #include <fstream>
// #include <iostream>
// #include <istream>
// #include <sstream>
// #include <streambuf>
// #include <string>
// #include <utility>
// #include <vector>

// #include "control/src/common/control_gflags.h"
// #include "control/src/common/control_utility.h"
// #include "control/src/common/file_util.h"
// #include "pnc_common/src/vehicle/vehicle_config.h"
// #include "pnc_control_cmd.pb.h"
// #include "pnc_control_config.pb.h"
// #include "pnc_common/src/math/math_utility/math_utils.h"
// #include "pnc_common/src/math/transform/euler_angles_zxy.h"
// #include "pnc_debug_control.pb.h"
// #include "pnc_common/src/utility/pnc_utility.h"
// #include "pnc_common/src/utility/time_utility.h"

// namespace jiduauto {
// namespace control {

// using ChassisConstPtr = std::shared_ptr<const pnc::chassis::Chassis>;
// using LocalizationConstPtr = std::shared_ptr<const pnc::localization::LocalizationEstimate>;
// using TrajectoryConstPtr = std::shared_ptr<const pnc::planning::ADCTrajectory>;
// using ControlCommandConstPtr = std::shared_ptr<const pnc::control::ControlCommand>;
// using jiduauto::pnc::EulerAnglesZXYf;
// using jiduauto::pnc::math::NormalizeAngle;
// using jiduauto::pnc::util::TimeUtility;

// struct SteerLogKeyInfo {
//     double cur_time;
//     double pose_x;
//     double pose_y;
//     double heading;
//     double vel;
//     double steer_feedback_deg;
// };  // mainly used for testing CheckSteerAngleOffsetPassed

// class ControlDiagnosisTest : public ::testing::Test {
// public:
//     virtual void SetUp() {
//         CHECK(FileUtil::ReadPnCPath());
//         std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
//         CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));
//         diag_conf_ = control_conf_.diag_conf();
//
//         controldiag_.Init(control_conf_);
//         veh_paras_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();
//     }

//     void InitLocalization(bool random = false);
//     void InitChassis();
//     void InitPlanning();
//     void InitControlCommand(pnc::control::ControlCommand* const control_command_ptr, double steering_target,
//                             double throttle, double brake, double acceleration);
//     bool ReadAnchorPoints(const std::string& file_name, std::vector<SteerLogKeyInfo>* anchor_points_ptr);
//     std::vector<std::string> Split(const std::string& s, const std::string& seperator);
//     void UpdateLocalizationByLog(const SteerLogKeyInfo& log_info);
//     void UpdateChassisByLog(const SteerLogKeyInfo& log_info);
//     bool InitLogFile();
//     bool MakeFullPathDir(const std::string& path);

// protected:
//     // conf
//     pnc::control::ControlConfig control_conf_;
//     DiagnosisConf diag_conf_;
//     pnc::common::VehicleParam veh_paras_;

//     pnc::localization::LocalizationEstimate localization_msg_;
//     pnc::chassis::Chassis chassis_msg_;
//     pnc::planning::ADCTrajectory planning_msg_;
//     pnc::control::ControlCommand cmd_msg_;
//     ControlDiagnosis controldiag_;
//     FILE* steer_angle_offset_mean_every_steps_file_{nullptr};

//     double curr_x_{0.2};
// };

// void ControlDiagnosisTest::InitLocalization(bool random) {
//     double curr_time = TimeUtility::GetCurrentTimesecond();
//     localization_msg_.mutable_header()->set_timestamp_sec(curr_time);
//     localization_msg_.mutable_header()->set_module_name("localization");
//     localization_msg_.mutable_header()->set_sequence_num(1);

//     double t_x = curr_x_;
//     double t_y = 0.01 * std::pow(t_x, 3) + 0.1 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
//     if (random) {
//         unsigned int seed = time(NULL);
//         t_x += (rand_r(&seed) % 20 - 10.0) / 10.0;
//         t_y += (rand_r(&seed) % 20 - 10.0) / 10.0;
//     }
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_x(t_x);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_y(t_y);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_z(0.1);
//     curr_x_ += 0.2;

//     double t_r = 0.0;                                 // roll
//     double t_p = 0.0;                                 // pitch
//     t_y = 0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2;  // yaw
//     t_y = NormalizeAngle(t_y);
//     EulerAnglesZXYf q(t_r, t_p, t_y);
//     Eigen::Quaternion<float> qua = q.ToQuaternion();
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qx(qua.x());
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qy(qua.y());
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qz(qua.z());
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qw(qua.w());

//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_x(0.5);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_y(0.5);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_z(0.1);

//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_z(0.0);

//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_x(0.0);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_y(0.0);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_z(0.0);
// }

// void ControlDiagnosisTest::InitChassis() {
//     double curr_time = TimeUtility::GetCurrentTimesecond();
//     chassis_msg_.mutable_header()->set_timestamp_sec(curr_time);
//     chassis_msg_.mutable_header()->set_module_name("chassis");
//     chassis_msg_.mutable_header()->set_sequence_num(1);
//     chassis_msg_.set_engine_started(true);
//     chassis_msg_.set_speed_mps(1.4);
//     chassis_msg_.set_odometer_m(100.2);
//     chassis_msg_.set_throttle_percentage(5.2);
//     chassis_msg_.set_brake_percentage(0.0);
//     chassis_msg_.set_steering_percentage(0.1);
//     chassis_msg_.set_parking_brake(false);
//     chassis_msg_.set_driving_mode(pnc::chassis::COMPLETE_AUTO_DRIVE);
//     chassis_msg_.set_gear_location(pnc::chassis::GEAR_DRIVE);
//     chassis_msg_.set_error_code(pnc::chassis::ErrorCode::NO_ERROR);
// }

// void ControlDiagnosisTest::InitPlanning() {
//     double curr_time = TimeUtility::GetCurrentTimesecond();
//     planning_msg_.mutable_header()->set_timestamp_sec(curr_time);
//     planning_msg_.mutable_header()->set_module_name("planning");
//     planning_msg_.mutable_header()->set_sequence_num(1);
//     // v = 1.2*sin(t) + 1.4
//     planning_msg_.set_total_path_time(8.0);
//     planning_msg_.mutable_estop()->set_is_estop(false);
//     planning_msg_.set_is_replan(false);
//     planning_msg_.set_gear(pnc::chassis::GEAR_DRIVE);
//     double t_x = localization_msg_.loc_pose().pose().position().x() - 1.0;
//     double accu_s = 0.0;
//     double init_v = 1.4;
//     for (double t = 0.0; t <= 8.0; t += 0.02) {
//         double v = 1.2 * std::sin(t) + init_v;
//         double a = 1.2 * std::cos(t);
//         auto* point = planning_msg_.add_trajectory_point();
//         t_x += v * 0.02;
//         point->mutable_path_point()->set_x(t_x);
//         double t_y = 0.01 * std::pow(t_x, 3) + 0.1 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
//         point->mutable_path_point()->set_y(t_y);
//         point->set_v(v);
//         point->set_a(a);
//         // curv
//         t_y = (0.06 * t_x + 0.2) / std::pow(1.0 + std::pow(0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2, 2), 1.5);
//         point->mutable_path_point()->set_kappa(t_y);
//         point->set_relative_time(t);
//         t_y = 0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2;
//         t_y = NormalizeAngle(t_y);
//         point->mutable_path_point()->set_theta(t_y);
//         point->mutable_path_point()->set_s(accu_s);
//         accu_s += v * 0.02;
//     }
// }

// void ControlDiagnosisTest::InitControlCommand(pnc::control::ControlCommand* const control_command_ptr,
//                                               double steering_target, double throttle, double brake,
//                                               double acceleration) {
//     control_command_ptr->set_steering_target(steering_target);
//     control_command_ptr->set_throttle(throttle);
//     control_command_ptr->set_brake(brake);
//     control_command_ptr->set_acceleration(acceleration);
// }

// void ControlDiagnosisTest::UpdateLocalizationByLog(const SteerLogKeyInfo& log_info) {
//     localization_msg_.mutable_header()->set_timestamp_sec(log_info.cur_time);
//     localization_msg_.mutable_header()->set_module_name("localization");
//     localization_msg_.mutable_header()->set_sequence_num(1);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_x(log_info.pose_x);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_y(log_info.pose_y);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_position()->set_z(0.1);

//     const double pose_roll = 0.0;        // roll
//     const double pose_pitch = 0.0;       // pitch
//     double pose_yaw = log_info.heading;  // yaw
//     pose_yaw = NormalizeAngle(pose_yaw);
//     EulerAnglesZXYf q(pose_roll, pose_pitch, pose_yaw);
//     Eigen::Quaternion<float> qua = q.ToQuaternion();
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qx(qua.x());
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qy(qua.y());
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qz(qua.z());
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qw(qua.w());

//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_x(0.5);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_y(0.5);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_z(0.1);

//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_z(0.0);

//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_x(0.0);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_y(0.0);
//     localization_msg_.mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_z(0.0);
// }

// void ControlDiagnosisTest::UpdateChassisByLog(const SteerLogKeyInfo& log_info) {
//     chassis_msg_.mutable_header()->set_timestamp_sec(log_info.cur_time);
//     chassis_msg_.mutable_header()->set_module_name("chassis");
//     chassis_msg_.mutable_header()->set_sequence_num(1);
//     chassis_msg_.set_engine_started(true);
//     chassis_msg_.set_speed_mps(log_info.vel);
//     chassis_msg_.set_odometer_m(100.2);
//     chassis_msg_.set_throttle_percentage(5.2);
//     chassis_msg_.set_brake_percentage(0.0);
//     chassis_msg_.set_steering_percentage(log_info.steer_feedback_deg);
//     chassis_msg_.set_parking_brake(false);
//     chassis_msg_.set_driving_mode(pnc::chassis::COMPLETE_AUTO_DRIVE);
//     // chassis_msg_.set_driving_mode(log_info.auto_mode);
//     chassis_msg_.set_gear_location(pnc::chassis::GEAR_DRIVE);
//     chassis_msg_.set_error_code(pnc::chassis::ErrorCode::NO_ERROR);
// }

// std::vector<std::string> ControlDiagnosisTest::Split(const std::string& s, const std::string& seperator) {
//     std::vector<std::string> result;
//     typedef std::string::size_type string_size;
//     string_size i = 0;

//     while (i != s.size()) {  // while i loop start
//         int flag = 0;
//         while (i != s.size() && flag == 0) {
//             flag = 1;
//             for (string_size x = 0; x < seperator.size(); ++x)
//                 if (s[i] == seperator[x]) {
//                     ++i;
//                     flag = 0;
//                     break;
//                 }
//         }
//         flag = 0;
//         string_size j = i;
//         while (j != s.size() && flag == 0) {
//             for (string_size x = 0; x < seperator.size(); ++x)
//                 if (s[j] == seperator[x]) {
//                     flag = 1;
//                     break;
//                 }
//             if (flag == 0) ++j;
//         }
//         if (i != j) {
//             result.push_back(s.substr(i, j - i));
//             i = j;
//         }
//     }  // while i loop end
//     return result;
// }

// bool ControlDiagnosisTest::ReadAnchorPoints(const std::string& file_name,
//                                             std::vector<SteerLogKeyInfo>* anchor_points_ptr) {
//     LOG_INFO("ReadAnchorPoints is ready!");  // debug
//     anchor_points_ptr->clear();
//     std::ifstream fp(file_name, std::ios::in);
//     if (!fp.is_open()) {
//         LOG_INFO("Failed to open fp!");  // debug
//         return false;
//     }
//     LOG_INFO("fp is open!");  // debug
//     SteerLogKeyInfo steer_info_;
//     const int LINE_LENGTH = 500;
//     char stringdata[LINE_LENGTH];
//     memset(stringdata, 0, sizeof(stringdata));
//     fp.getline(stringdata, LINE_LENGTH);  // abandon header in file
//     LOG_INFO("header in fp has got!");    // debug
//     while (!fp.eof()) {
//         memset(stringdata, 0, sizeof(stringdata));
//         fp.getline(stringdata, LINE_LENGTH);  // get useful data line by line
//         if (strlen(stringdata) < 1) break;
//         std::string processdata = stringdata;
//         std::vector<std::string> splitdata = Split(processdata, ",");
//         int j;
//         j = 1;  // time
//         steer_info_.cur_time = atof(splitdata[j].c_str());
//         j = 2;  // x
//         steer_info_.pose_x = atof(splitdata[j].c_str());
//         j = 3;  // y
//         steer_info_.pose_y = atof(splitdata[j].c_str());
//         j = 4;  // heading
//         steer_info_.heading = atof(splitdata[j].c_str());
//         j = 5;  // vel
//         steer_info_.vel = atof(splitdata[j].c_str());
//         j = 14;  // chassis_steer_angle
//         steer_info_.steer_feedback_deg = atof(splitdata[j].c_str());
//         anchor_points_ptr->push_back(steer_info_);
//     }
//     fp.close();
//     fp.clear();
//     return true;
// }

// bool ControlDiagnosisTest::InitLogFile() {
//     const std::string path_name = FLAGS_pnc_data_log_path + "unit_test/diagnosis/";
//     if (!MakeFullPathDir(path_name)) {
//         LOG_INFO("Error: MakeFullPathDir make full path Failed!");
//         return false;
//     }

//     // const std::string path_name = FLAGS_pnc_data_log_path;
//     const std::string csv_name = "steer_angle_offset_mean_every_steps.csv";
//     std::string file_name = path_name + csv_name;
//     steer_angle_offset_mean_every_steps_file_ = fopen(file_name.c_str(), "w");
//     if (steer_angle_offset_mean_every_steps_file_ == nullptr) {
//         return false;
//     }
//     fprintf(steer_angle_offset_mean_every_steps_file_, "index,is_update,steer_angle_offset_mean,\n");
//     std::string log_str = "\n\nThe result is in " + file_name + "\n\n";
//     LOG_INFO(log_str);
//     return true;
// }

// bool ControlDiagnosisTest::MakeFullPathDir(const std::string& path) {
//     if (path.empty()) {
//         return false;
//     }
//     return true;
// }

// // TEST_F(ControlDiagnosisTest, GeneralDetectPassed) {
// //     pnc::control::ControlConfig_ControllerType lat_init;
// //     pnc::control::ControlDebug control_debug;
// //     bool result;

// //     LOG_INFO("GeneralDetectPassed test start..");

// //     InitLocalization();
// //     InitChassis();
// //     InitPlanning();

// //     ChassisConstPtr chassis_ptr = std::make_shared<const pnc::chassis::Chassis>(chassis_msg_);
// //     LocalizationConstPtr localization_ptr =
// //         std::make_shared<const pnc::localization::LocalizationEstimate>(localization_msg_);
// //     TrajectoryConstPtr trajectory_ptr = std::make_shared<const pnc::planning::ADCTrajectory>(planning_msg_);
// //     ControlCommandConstPtr last_cmd_ptr = nullptr;

// //     lat_init = pnc::control::ControlConfig::CHASSIS_TEST_CONTROLLER;
// //     result = controldiag_.GeneralDetectPassed(lat_init, chassis_ptr, localization_ptr, trajectory_ptr,
// last_cmd_ptr,
// //                                               control_debug.mutable_diagnosis_debug());
// //     EXPECT_TRUE(result);
// //     LOG_INFO("CHASSIS_TEST_CONTROLLER result: %d", result);

// //     lat_init = pnc::control::ControlConfig::LQR_LAT_CONTROLLER;
// //     result = controldiag_.GeneralDetectPassed(lat_init, chassis_ptr, localization_ptr, trajectory_ptr,
// last_cmd_ptr,
// //                                               control_debug.mutable_diagnosis_debug());
// //     EXPECT_TRUE(result);
// //     LOG_INFO("LQR_LAT_CONTROLLER result: %d", result);
// // }

// // TEST_F(ControlDiagnosisTest, CheckSteerAngleOffsetPassed) {
// //     ChassisConstPtr chassis_ptr;
// //     LocalizationConstPtr localization_ptr;
// //     bool result{true};
// //     pnc::control::ControlDebug control_debug;

// //     // general test
// //     if (!FLAGS_enable_UT_diagnosis_depth_test) {
// //         LOG_INFO("CheckSteerAngleOffsetPassed general test start...");
// //         InitLocalization();
// //         InitChassis();

// //         chassis_ptr = std::make_shared<const pnc::chassis::Chassis>(chassis_msg_);
// //         localization_ptr = std::make_shared<const pnc::localization::LocalizationEstimate>(localization_msg_);

// //         auto& orientation = localization_ptr->loc_pose().pose().orientation();
// //         jiduauto::pnc::EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
// //         double cur_heading = q.yaw() + veh_paras_.heading_offset();  //+ M_PI * 0.5

// //         result =
// //             controldiag_.CheckSteerAngleOffsetPassed(chassis_ptr, cur_heading,
// //             control_debug.mutable_diagnosis_debug());
// //         EXPECT_TRUE(result);
// //         return;
// //     }

// //     // depth test
// //     LOG_INFO("CheckSteerAngleOffsetPassed depth test start...");
// //     // step 1: read anchor points from steer log
// //     std::vector<SteerLogKeyInfo> anchor_points;
// //     std::string file_name = FLAGS_control_unit_test_data_path + "diagnosis/steer_log_lqr.csv";
// //     bool bflag = ReadAnchorPoints(file_name, &anchor_points);
// //     EXPECT_TRUE(bflag);

// //     // step2: check steer angle offset based on steer log
// //     const int TOTAL_SIZE = anchor_points.size();
// //     SteerLogKeyInfo steer_log_info;
// //     const int PRINT_PERIOD = 100;

// //     for (int index = 0; index < TOTAL_SIZE; index++) {
// //         // [/start]Debug code
// //         int DEBUG_stop_index = 819;
// //         DEBUG_stop_index = std::min(DEBUG_stop_index, TOTAL_SIZE);
// //         if (index == DEBUG_stop_index) {
// //             LOG_INFO("\n\nDEBUG: total_index = %d, cur_index = %d\n\n", TOTAL_SIZE, index);
// //         }
// //         if (index % PRINT_PERIOD == 0) {
// //             LOG_INFO("Running: total_index = %d, cur_index = %d", TOTAL_SIZE, index);
// //         }
// //         // [/end]Debug code

// //         // main part: unit test of CheckSteerAngleOffsetPassed
// //         steer_log_info = anchor_points[index];
// //         UpdateLocalizationByLog(steer_log_info);
// //         UpdateChassisByLog(steer_log_info);

// //         chassis_ptr = std::make_shared<const pnc::chassis::Chassis>(chassis_msg_);
// //         localization_ptr = std::make_shared<const pnc::localization::LocalizationEstimate>(localization_msg_);

// //         auto& orientation = localization_ptr->loc_pose().pose().orientation();
// //         jiduauto::pnc::EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
// //         double cur_heading = q.yaw() + veh_paras_.heading_offset();  //+ M_PI * 0.5

// //         result =
// //             controldiag_.CheckSteerAngleOffsetPassed(chassis_ptr, cur_heading,
// //             control_debug.mutable_diagnosis_debug());

// //         // Collect data for comparison with matlab result
// //         const bool is_COMPARE_MATLAB = true;
// //         if (is_COMPARE_MATLAB) {
// //             if (!(steer_angle_offset_mean_every_steps_file_ == nullptr && !InitLogFile())) {
// //                 fprintf(steer_angle_offset_mean_every_steps_file_, "%d,%d,%.10f,\n", index,
// //                         control_debug.mutable_diagnosis_debug()
// //                             ->mutable_steer_offset_check_result()
// //                             ->temp_is_steer_angle_offset_mean_update(),
// //                         control_debug.mutable_diagnosis_debug()
// //                             ->mutable_steer_offset_check_result()
// //                             ->steer_angle_offset_after_check());
// //                 fflush(steer_angle_offset_mean_every_steps_file_);
// //             }
// //         }
// //     }
// //     LOG_INFO("CheckSteerAngleOffsetPassed test finish");
// //     EXPECT_TRUE(result);
// // }

// // TEST_F(ControlDiagnosisTest, EnsureControlCommandNotNan) {
// //     pnc::control::ControlCommand cmd_msg;
// //     pnc::control::DiagnosisDebug diag_debug;

// //     // normal
// //     double steering_target = 0.0;
// //     double throttle = 10.0;
// //     double brake = 0.0;
// //     double acceleration = 0.5;
// //     InitControlCommand(&cmd_msg, steering_target, throttle, brake, acceleration);
// //     bool result = controldiag_.EnsureControlCommandNotNan(&cmd_msg, &diag_debug);
// //     EXPECT_TRUE(result);

// //     // abnormal: only one loop
// //     double last_steering_target = steering_target;
// //     steering_target = NAN;
// //     InitControlCommand(&cmd_msg, steering_target, throttle, brake, acceleration);
// //     result = controldiag_.EnsureControlCommandNotNan(&cmd_msg, &diag_debug);
// //     EXPECT_TRUE(result);
// //     EXPECT_TRUE(cmd_msg.steering_target() == last_steering_target);

// //     // abnormal: continuous count of abnormal is greater than limit
// //     brake = NAN;
// //     for (int i = 0; i < diag_conf_.limit_control_command_continuous_abnormal() + 1; ++i) {
// //         InitControlCommand(&cmd_msg, steering_target, throttle, brake, acceleration);
// //         controldiag_.EnsureControlCommandNotNan(&cmd_msg, &diag_debug);
// //     }
// //     EXPECT_TRUE(diag_debug.mutable_control_command_post_process_result()
// //                     ->is_count_control_command_continuous_abnormal_beyond_limit());
// // }

// }  // namespace control
// }  // namespace jiduauto
