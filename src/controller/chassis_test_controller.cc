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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <string>
#include <vector>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/common/file_util.h"
#include "control/src/math/control_math.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::util::TimeUtility;
using pnc::chassis::Chassis;
using pnc::control::ControlCommand;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

ChassisTestController::ChassisTestController() { LOG_INFO("%s is used.", Name().c_str()); }

ChassisTestController::~ChassisTestController() { Stop(); }

bool ChassisTestController::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("%s start init.", Name().c_str());
    control_period_ = control_conf.control_period();
    chassis_test_conf_ = control_conf.chassis_test_controller_conf();
    veh_paras_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();

    controller_initialized_ = true;
    return true;
}

bool ChassisTestController::Control(const LocalizationEstimate* localization, const Chassis* chassis,
                                    const ADCTrajectory* trajectory, ControlCommand* const cmd) {
    if (!SanityCheck()) {
        LOG_ERROR("SanityCheck() failed!");
        return false;
    }
    uint64_t curr_time = TimeUtility::GetCurrentTimeMillsecond();
    if (curr_time - prev_time_ < static_cast<uint64_t>(control_period_ * 1000)) {
        LOG_INFO("Invoke too fast, wait until next cycle");
        return true;
    }
    if (chassis == nullptr || cmd == nullptr) {
        LOG_ERROR("Input is nullptr!");
        return false;
    }
    if (!controller_initialized_) {
        LOG_ERROR("Failed to initialize ChassisTestController!");
        return false;
    }
    // set params
    chassis_msg_ = chassis;
    cmd_msg_ = cmd;

    if (ControlMath::CheckBackward(chassis_msg_)) {
        cmd->set_driving_mode(pnc::chassis::COMPLETE_MANUAL);
        LOG_ERROR("Wrong gear location! set accel = %.4f", cmd_msg_->acceleration());
        return false;
    }

    UpdateChassisStatus();

    // test item
    switch (chassis_test_conf_.enable_test()) {
    case pnc::control::ChassisTestConf::LON_STANDSTILL_TEST:
        LonStandStillTest();
        break;
    case pnc::control::ChassisTestConf::LON_IDLESPEED_TEST:
        LonIdleSpeedTest();
        break;
    case pnc::control::ChassisTestConf::LON_STEP_TEST:
        LonStepTest();
        break;
    case pnc::control::ChassisTestConf::LON_SINUSOID_TEST:
        LonSinusoidTest();
        break;
    case pnc::control::ChassisTestConf::LAT_STANDSTILL_TEST:
        LatStandStillTest();
        break;
    case pnc::control::ChassisTestConf::LAT_STEP_TEST:
        LatStepTest();
        break;
    case pnc::control::ChassisTestConf::LAT_SINUSOID_TEST:
        LatSinusoidTest();
        break;
    case pnc::control::ChassisTestConf::DISABLE_TEST:
    default:
        control_cmd_.cmd_gear_ = pnc::chassis::GEAR_PARKING;
        control_cmd_.cmd_accel_ = kStopAcc_;
        control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_MANUAL;
        break;
    }

    // set cmd
    cmd->set_driving_mode(control_cmd_.cmd_mode_);
    cmd->set_gear_location(control_cmd_.cmd_gear_);
    cmd->set_acceleration(control_cmd_.cmd_accel_);
    cmd->set_steering_target(control_cmd_.cmd_angle_);
    cmd->set_steering_rate(veh_paras_.max_steer_angle_rate() * 180.0 / M_PI);
    cmd->set_throttle(0.0);
    cmd->set_brake(0.0);

    prev_time_ = curr_time;

    if (log_file_ == nullptr && !InitLogFile()) {
        LOG_ERROR("Failed to write log file!");
        return false;
    }
    fprintf(log_file_,
            "%.4ld, %.4f, %.4f, %.4f, %.4f, %d, %d, "
            "%.4f, %.4f, %.4f, %.4f,%.4f, %.4f, %d, %d\n",
            curr_time, chassis_status_.curr_speed_mps_, chassis_status_.curr_speed_kph_, control_cmd_.cmd_accel_,
            chassis_status_.curr_accel_, control_cmd_.cmd_gear_, chassis_status_.curr_gear_, control_cmd_.cmd_angle_,
            veh_paras_.max_steer_angle_rate() * 180.0 / M_PI, chassis_status_.curr_angle_,
            chassis_status_.curr_angle_rate_, chassis_msg_->throttle_percentage(), chassis_msg_->brake_percentage(),
            control_cmd_.cmd_mode_, chassis_status_.curr_mode_);
    return true;
}

bool ChassisTestController::Reset() {
    chassis_status_.curr_mode_ = pnc::chassis::COMPLETE_MANUAL;
    chassis_status_.curr_gear_ = pnc::chassis::GEAR_INVALID;
    chassis_status_.curr_speed_mps_ = 0.0;
    chassis_status_.prev_speed_mps_ = 0.0;
    chassis_status_.curr_speed_kph_ = 0.0;
    chassis_status_.curr_accel_ = 0.0;
    chassis_status_.curr_angle_ = 0.0;
    chassis_status_.prev_angle_ = 0.0;
    chassis_status_.curr_angle_rate_ = 0.0;
    control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_MANUAL;
    control_cmd_.cmd_gear_ = pnc::chassis::GEAR_PARKING;
    control_cmd_.cmd_accel_ = 0.0;
    control_cmd_.cmd_angle_ = 0.0;
    control_cmd_.cmd_angle_rate_ = 0.0;

    LOG_INFO("ChassisTestController Reset()");
    return true;
}

std::string ChassisTestController::Name() const { return "ChassisTestController"; }

void ChassisTestController::Stop() {
    if (log_file_ != nullptr) {
        fclose(log_file_);
        log_file_ = nullptr;
    }
    chassis_msg_ = nullptr;
    cmd_msg_ = nullptr;
    if (Reset()) {
        LOG_INFO("ChassisTestController Stop!");
    }
}

bool ChassisTestController::SanityCheck() const {
    return chassis_test_conf_.has_lon_stand_still_test() && chassis_test_conf_.has_lon_idle_test() &&
           chassis_test_conf_.has_lon_step_test() && chassis_test_conf_.has_lon_sinusoid_test() &&
           chassis_test_conf_.has_lat_stand_still_test() && chassis_test_conf_.has_lat_step_test() &&
           chassis_test_conf_.has_lat_sinusoid_test();
}

void ChassisTestController::UpdateChassisStatus() {
    if (chassis_msg_ == nullptr) {
        LOG_ERROR("chassis_msg_ is nullptr");
        return;
    }
    float delta_time = static_cast<float>(TimeUtility::GetCurrentTimeMillsecond() - prev_time_) / 1000.0;

    chassis_status_.curr_mode_ = chassis_msg_->driving_mode();
    chassis_status_.curr_gear_ = chassis_msg_->gear_location();
    chassis_status_.curr_speed_mps_ = chassis_msg_->speed_mps();
    chassis_status_.curr_speed_kph_ = chassis_msg_->speed_mps() * 3.6;
    chassis_status_.curr_accel_ =
        (chassis_status_.curr_speed_mps_ - chassis_status_.prev_speed_mps_) / delta_time;  // m/s^2
    chassis_status_.curr_angle_ = chassis_msg_->steering_percentage() * veh_paras_.max_steer_angle() * 180.0 / M_PI /
                                  veh_paras_.steer_max_output();  // deg
    chassis_status_.curr_angle_rate_ =
        (chassis_status_.curr_angle_ - chassis_status_.prev_angle_) / delta_time;  // deg/s

    chassis_status_.prev_speed_mps_ = chassis_status_.curr_speed_mps_;
    chassis_status_.prev_angle_ = chassis_status_.curr_angle_;
    return;
}

bool ChassisTestController::LonStandStillTest() {
    control_cmd_.cmd_gear_ = pnc::chassis::GEAR_DRIVE;
    control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_AUTO_DRIVE;

    // step1: send control_cmd_.cmd_accel_{0.0} within t_hold
    static double lonstandstart_time = TimeUtility::GetCurrentTimesecond();
    static bool lonstand_bfirst_step = true;
    static float standstill_velocity = 0.0;

    double curr_time = TimeUtility::GetCurrentTimesecond();
    if (lonstand_bfirst_step) {
        if (control_cmd_.cmd_gear_ != chassis_status_.curr_gear_ ||
            chassis_status_.curr_speed_kph_ > standstill_velocity ||
            chassis_status_.curr_mode_ != control_cmd_.cmd_mode_) {
            LOG_ERROR("Failed to met test condition! GEAR=[%d] SPEED_KPH=[%.4f] MODE=[%d]", control_cmd_.cmd_gear_,
                      chassis_status_.curr_speed_kph_, chassis_status_.curr_mode_);
            control_cmd_.cmd_accel_ = chassis_test_conf_.lon_stand_still_test().acc_init();
            return false;
        } else {
            if (curr_time < lonstandstart_time + chassis_test_conf_.lon_stand_still_test().t_hold()) {
                return true;
            } else {
                lonstand_bfirst_step = false;
                control_cmd_.cmd_accel_ += chassis_test_conf_.lon_stand_still_test().acc_resolution();
            }
        }
    }

    // step2: add control_cmd_.cmd_accel_ with acc_resolution in t_hold; break if
    // chassis_status_.curr_speed_kph_ > idle_velocity + v_threshold;
    static bool lonstand_bsecond_step = true;
    curr_time = TimeUtility::GetCurrentTimesecond();
    static double lonstand_step_time = TimeUtility::GetCurrentTimesecond();
    // check whether need stop
    if (!lonstand_bsecond_step || (chassis_status_.curr_speed_kph_ >
                                   standstill_velocity + chassis_test_conf_.lon_stand_still_test().v_threshold())) {
        // finished
        lonstand_bsecond_step = false;
        control_cmd_.cmd_accel_ = kStopAcc_;
        LOG_INFO("Test can be terminated!");
        return true;
    }

    if (curr_time >= lonstand_step_time + chassis_test_conf_.lon_stand_still_test().t_hold()) {
        control_cmd_.cmd_accel_ += chassis_test_conf_.lon_stand_still_test().acc_resolution();
        lonstand_step_time = curr_time;
    }
    return true;
}

bool ChassisTestController::LonIdleSpeedTest() {
    control_cmd_.cmd_gear_ = pnc::chassis::GEAR_DRIVE;
    control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_AUTO_DRIVE;
    if ((chassis_status_.curr_mode_ != control_cmd_.cmd_mode_) ||
        (chassis_status_.curr_gear_ != control_cmd_.cmd_gear_)) {
        control_cmd_.cmd_accel_ = 0.0;
        return true;
    }

    // step1: send control_cmd_.cmd_accel_{0.0} within t_init
    static double lonidle_start_time = TimeUtility::GetCurrentTimesecond();
    static bool lonidle_bfirst_step = true;
    static std::vector<float> speed_array{};
    static float idle_velocity = 0.0;

    double curr_time = TimeUtility::GetCurrentTimesecond();
    if (lonidle_bfirst_step) {
        if (curr_time < lonidle_start_time + chassis_test_conf_.lon_idle_test().t_init()) {
            speed_array.push_back(chassis_status_.curr_speed_kph_);
            control_cmd_.cmd_accel_ = 0.0;
            return true;
        } else {
            if (speed_array.empty()) {
                LOG_ERROR("speed_array.empty()");
                return false;
            }
            lonidle_bfirst_step = false;
            idle_velocity = std::accumulate(speed_array.begin(), speed_array.end(), 0.0);
            idle_velocity /= speed_array.size();
            control_cmd_.cmd_accel_ += chassis_test_conf_.lon_idle_test().acc_resolution();
        }
    }

    // step2: add control_cmd_.cmd_accel_ with acc_resolution in t_hold; break if
    // chassis_status_.curr_speed_kph_ > idle_velocity + v_threshold;
    static bool lonidle_bsecond_step = true;
    curr_time = TimeUtility::GetCurrentTimesecond();
    static double lonidle_step_time = TimeUtility::GetCurrentTimesecond();
    // check whether need stop
    if (!lonidle_bsecond_step ||
        (chassis_status_.curr_speed_kph_ > idle_velocity + chassis_test_conf_.lon_idle_test().v_threshold())) {
        // finished
        lonidle_bsecond_step = false;
        control_cmd_.cmd_accel_ = kStopAcc_;
        return true;
    }

    if (curr_time >= lonidle_step_time + chassis_test_conf_.lon_idle_test().t_hold()) {
        control_cmd_.cmd_accel_ += chassis_test_conf_.lon_idle_test().acc_resolution();
        lonidle_step_time = curr_time;
    }

    return true;
}

bool ChassisTestController::LonStepTest() {
    if (chassis_test_conf_.lon_step_test().lon_step_test_detail().size() < 1) {
        LOG_ERROR("conf error");
        return false;
    }
    control_cmd_.cmd_gear_ = pnc::chassis::GearPosition::GEAR_DRIVE;
    control_cmd_.cmd_mode_ = pnc::chassis::DrivingMode::COMPLETE_AUTO_DRIVE;
    if ((chassis_status_.curr_mode_ != control_cmd_.cmd_mode_) ||
        (chassis_status_.curr_gear_ != control_cmd_.cmd_gear_)) {
        control_cmd_.cmd_accel_ = 0.0;
        return true;
    }

    static double lonstep_start_time = TimeUtility::GetCurrentTimesecond();
    static int32_t lonstep_index = -1;
    double curr_time = TimeUtility::GetCurrentTimesecond();

    if (lonstep_index >= chassis_test_conf_.lon_step_test().lon_step_test_detail().size()) {
        // finished
        control_cmd_.cmd_accel_ = kStopAcc_;
        return true;
    }

    // step1: index{-1}
    if (lonstep_index == -1) {
        if (curr_time < lonstep_start_time + chassis_test_conf_.lon_step_test().lon_step_test_detail()[0].t_gap()) {
            // continue with control_cmd_.cmd_accel_;
            control_cmd_.cmd_accel_ = 0.0;
            return true;
        } else {
            ++lonstep_index;
            control_cmd_.cmd_accel_ = chassis_test_conf_.lon_step_test().lon_step_test_detail()[0].acc_step();
            lonstep_start_time = curr_time;
            return true;
        }
    }
    // step2: cycle accel and keep speed
    const auto& t_conf = chassis_test_conf_.lon_step_test().lon_step_test_detail()[lonstep_index];
    static bool lonstep_bkeep_time = false;
    if (lonstep_bkeep_time) {
        curr_time = TimeUtility::GetCurrentTimesecond();
        if (curr_time > lonstep_start_time + t_conf.t_gap()) {
            lonstep_bkeep_time = false;
            control_cmd_.cmd_accel_ = t_conf.acc_step();
            return true;
        } else {
            control_cmd_.cmd_accel_ = 0.0;
            return true;
        }
    }
    if ((t_conf.acc_step() > 0.0 && chassis_status_.curr_speed_kph_ >= t_conf.v_target()) ||
        (t_conf.acc_step() < 0.0 && chassis_status_.curr_speed_kph_ <= t_conf.v_target())) {
        lonstep_bkeep_time = true;
        ++lonstep_index;
        lonstep_start_time = TimeUtility::GetCurrentTimesecond();
        control_cmd_.cmd_accel_ = 0.0;
    }
    return true;
}

bool ChassisTestController::LonSinusoidTest() {
    if (chassis_test_conf_.lon_sinusoid_test().lon_sinusoid_test_detail().size() < 1) {
        LOG_ERROR("conf error");
        return false;
    }
    control_cmd_.cmd_gear_ = pnc::chassis::GEAR_DRIVE;
    control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_AUTO_DRIVE;
    if ((chassis_status_.curr_mode_ != control_cmd_.cmd_mode_) ||
        (chassis_status_.curr_gear_ != control_cmd_.cmd_gear_)) {
        control_cmd_.cmd_accel_ = 0.0;
        return true;
    }

    static double lonsinu_start_time = TimeUtility::GetCurrentTimesecond();
    static int32_t lonsinu_index = -1;
    double curr_time = TimeUtility::GetCurrentTimesecond();

    if (lonsinu_index >= chassis_test_conf_.lon_sinusoid_test().lon_sinusoid_test_detail().size()) {
        // finished
        control_cmd_.cmd_accel_ = kStopAcc_;
        return true;
    }
    // step1: keep control_cmd_.cmd_accel_ within t_init
    if (lonsinu_index < 0) {
        if (curr_time < lonsinu_start_time + chassis_test_conf_.lon_sinusoid_test().t_init()) {
            control_cmd_.cmd_accel_ = 0.0;
            return true;
        } else {
            lonsinu_index = 0;
            lonsinu_start_time = TimeUtility::GetCurrentTimesecond();
            control_cmd_.cmd_accel_ = 0.0;
            return true;
        }
    }
    // step2: sinusoid curv
    const auto& t_conf = chassis_test_conf_.lon_sinusoid_test().lon_sinusoid_test_detail()[lonsinu_index];
    double delta_time = curr_time - lonsinu_start_time;
    double num = 0.0;
    if (delta_time > 0) {
        num = delta_time / t_conf.t_period();
    }
    if (num >= t_conf.cycle_num()) {
        ++lonsinu_index;
        lonsinu_start_time = TimeUtility::GetCurrentTimesecond();
    }
    curr_time = TimeUtility::GetCurrentTimesecond();
    delta_time = std::max(0.0, curr_time - lonsinu_start_time);
    control_cmd_.cmd_accel_ =
        chassis_test_conf_.lon_sinusoid_test().acc_amplitude() * std::sin(2 * M_PI / t_conf.t_period() * delta_time);

    return true;
}

bool ChassisTestController::LatStandStillTest() {
    control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_AUTO_DRIVE;
    control_cmd_.cmd_accel_ = kStopAcc_;

    // step1: send control_cmd_.cmd_angle_{0.0} within t_init
    static double start_time = TimeUtility::GetCurrentTimesecond();
    static bool bfirst_step = true;
    static float standstill_velocity = 0.0;
    static float steer_offset = 0.0;
    static std::vector<float> steer_array{};

    double curr_time = TimeUtility::GetCurrentTimesecond();
    if (bfirst_step) {
        if (chassis_status_.curr_accel_ > 0 || chassis_status_.curr_speed_kph_ > standstill_velocity ||
            chassis_status_.curr_mode_ != control_cmd_.cmd_mode_ ||
            std::abs(chassis_status_.curr_angle_ - control_cmd_.cmd_angle_) > 1) {
            LOG_ERROR(
                "Failed to met test condition! ACCEL=[%.4f] VELOCITY=[%.4f] "
                "MODE=[%d] STEER=[%.4f]",
                chassis_status_.curr_accel_, chassis_status_.curr_speed_kph_, chassis_status_.curr_mode_,
                chassis_status_.curr_angle_);
            control_cmd_.cmd_angle_ = chassis_test_conf_.lat_stand_still_test().steer_init();
            return false;
        } else {
            if (curr_time < start_time + chassis_test_conf_.lat_stand_still_test().t_init()) {
                steer_array.push_back(chassis_status_.curr_angle_);
                return true;
            } else {
                if (steer_array.empty()) {
                    LOG_ERROR("steer_array.empty()");
                    return false;
                }
                bfirst_step = false;
                steer_offset = std::accumulate(steer_array.begin(), steer_array.end(), 0.0);
                steer_offset /= steer_array.size();
                control_cmd_.cmd_angle_ += chassis_test_conf_.lat_stand_still_test().steer_resolution();
            }
        }
    }

    // step2: add control_cmd_.cmd_angle_ with steer_resolution in t_hold; break
    // if chassis_status_.curr_angle_ > steer_offset + steer_threshold;
    static bool bsecond_step = true;
    curr_time = TimeUtility::GetCurrentTimesecond();
    static double step_time = TimeUtility::GetCurrentTimesecond();
    // check whether need stop
    if (!bsecond_step ||
        (chassis_status_.curr_angle_ > steer_offset + chassis_test_conf_.lat_stand_still_test().steer_threshold())) {
        // finished
        bsecond_step = false;
        control_cmd_.cmd_angle_ = 0.0;
        LOG_INFO("Test can be terminated!");
        return true;
    }

    if (curr_time >= step_time + chassis_test_conf_.lat_stand_still_test().t_hold()) {
        control_cmd_.cmd_angle_ += chassis_test_conf_.lat_stand_still_test().steer_resolution();
        step_time = curr_time;
    }
    return true;
}

bool ChassisTestController::LatStepTest() {
    control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_AUTO_DRIVE;
    control_cmd_.cmd_accel_ = kStopAcc_;

    if (chassis_test_conf_.lat_step_test().lat_step_test_detail().size() < 1) {
        LOG_ERROR("conf error");
        return false;
    }
    if ((chassis_status_.curr_mode_ != control_cmd_.cmd_mode_) ||
        (chassis_status_.curr_gear_ != control_cmd_.cmd_gear_)) {
        control_cmd_.cmd_angle_ = 0.0;
        return true;
    }

    static double latstep_start_time = TimeUtility::GetCurrentTimesecond();
    static int32_t latstep_index = -1;
    double curr_time = TimeUtility::GetCurrentTimesecond();

    if (latstep_index >= chassis_test_conf_.lat_step_test().lat_step_test_detail().size()) {
        // finished
        control_cmd_.cmd_accel_ = -1.0;
        control_cmd_.cmd_angle_ = 0.0;
        return true;
    }

    // step1: index{-1}
    if (latstep_index == -1) {
        if (curr_time < latstep_start_time + chassis_test_conf_.lat_step_test().t_reset()) {
            // continue with control_cmd_.cmd_angle_;
            control_cmd_.cmd_angle_ = 0.0;
            return true;
        } else {
            ++latstep_index;
            control_cmd_.cmd_angle_ = chassis_test_conf_.lat_step_test().lat_step_test_detail()[0].steer_angle();
            if (chassis_test_conf_.lat_step_test().direction() == pnc::control::LatStepTest::TURN_RIGHT) {
                control_cmd_.cmd_angle_ *= -1;
            }
            latstep_start_time = curr_time;
            return true;
        }
    }

    // step2: cycle accel and keep speed
    const auto& t_conf = chassis_test_conf_.lat_step_test().lat_step_test_detail()[latstep_index];
    static bool latstep_bkeep_time = false;
    if (latstep_bkeep_time) {
        latstep_bkeep_time = false;
        control_cmd_.cmd_angle_ = t_conf.steer_angle();
        if (chassis_test_conf_.lat_step_test().direction() == pnc::control::LatStepTest::TURN_RIGHT) {
            control_cmd_.cmd_angle_ *= -1;
        }
        latstep_start_time = curr_time;
        return true;
    }
    if (curr_time > latstep_start_time + chassis_test_conf_.lat_step_test().t_hold()) {
        latstep_bkeep_time = true;
        ++latstep_index;
        latstep_start_time = TimeUtility::GetCurrentTimesecond();
    }
    return true;
}

bool ChassisTestController::LatSinusoidTest() {
    control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_AUTO_DRIVE;
    control_cmd_.cmd_accel_ = kStopAcc_;
    if (chassis_test_conf_.lat_sinusoid_test().lat_sinusoid_test_detail().size() < 1) {
        LOG_ERROR("conf error");
        return false;
    }

    if ((chassis_status_.curr_mode_ != control_cmd_.cmd_mode_) ||
        (chassis_status_.curr_gear_ != control_cmd_.cmd_gear_)) {
        control_cmd_.cmd_angle_ = chassis_test_conf_.lat_sinusoid_test().steer_reset();
        return true;
    }

    static double latsinu_start_time = TimeUtility::GetCurrentTimesecond();
    static int32_t latsinu_index = -1;
    double curr_time = TimeUtility::GetCurrentTimesecond();

    if (latsinu_index >= chassis_test_conf_.lat_sinusoid_test().lat_sinusoid_test_detail().size()) {
        // finished
        control_cmd_.cmd_accel_ = -1.0;
        control_cmd_.cmd_angle_ = chassis_test_conf_.lat_sinusoid_test().steer_reset();
        return true;
    }
    // step1: keep control_cmd_.cmd_angle_ within t_init
    if (latsinu_index < 0) {
        if (curr_time < latsinu_start_time + chassis_test_conf_.lat_sinusoid_test().t_reset()) {
            control_cmd_.cmd_angle_ = chassis_test_conf_.lat_sinusoid_test().steer_reset();
            return true;
        } else {
            latsinu_index = 0;
            latsinu_start_time = TimeUtility::GetCurrentTimesecond();
            control_cmd_.cmd_angle_ = chassis_test_conf_.lat_sinusoid_test().steer_reset();
            return true;
        }
    }
    // step2: sinusoid curv
    const auto& t_conf = chassis_test_conf_.lat_sinusoid_test().lat_sinusoid_test_detail()[latsinu_index];
    double delta_time = curr_time - latsinu_start_time;
    double num = 0.0;
    if (delta_time > 0) {
        num = delta_time / chassis_test_conf_.lat_sinusoid_test().t_period();
    }
    if (num >= t_conf.cycle_num()) {
        ++latsinu_index;
        latsinu_start_time = TimeUtility::GetCurrentTimesecond();
    }
    curr_time = TimeUtility::GetCurrentTimesecond();
    delta_time = std::max(0.0, curr_time - latsinu_start_time);
    control_cmd_.cmd_angle_ =
        t_conf.steer_amplitude() * std::sin(2 * M_PI / chassis_test_conf_.lat_sinusoid_test().t_period() * delta_time);

    return true;
}

bool ChassisTestController::InitLogFile() {
    auto now_time = std::chrono::system_clock::now();
    auto now_time_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now_time);
    std::time_t now_c = std::chrono::system_clock::to_time_t(now_time_seconds);
    char name_buffer[80];
    strftime(name_buffer, 80, "steer_log_lqr_%Y%m%d%H%M%S.csv", std::localtime(&now_c));

    std::string file_name = FLAGS_pnc_data_log_path + "/control/" + name_buffer;
    log_file_ = fopen(file_name.c_str(), "w");
    if (log_file_ == nullptr) {
        LOG_ERROR("Failed to open log file!");
        return false;
    }
    fprintf(log_file_,
            "time, speed_mps, speed_kph, accel_cmd, accel_fb, gear_cmd, gear_fb, "
            "steer_cmd, steer_rate_cmd, steer_fb, steer_rate_fb, "
            "throttle_percentage, brake_percentage, automode_cmd, automode_fb, "
            "\n");
    return true;
}

bool ChassisTestController::ControlMain(const ControlInputStream& control_input_stream,
                                        ControlCommandStream* const cmd_stream) {
    if (!SanityCheck()) {
        LOG_ERROR("SanityCheck() failed!");
        return false;
    }
    uint64_t curr_time = TimeUtility::GetCurrentTimeMillsecond();
    if (curr_time - prev_time_ < static_cast<uint64_t>(control_period_ * 1000)) {
        LOG_INFO("Invoke too fast, wait until next cycle");
        return true;
    }
    if (cmd_stream == nullptr) {
        LOG_ERROR("command stream is nullptr!");
        return false;
    }
    if (!controller_initialized_) {
        LOG_ERROR("Failed to initialize ChassisTestController!");
        return false;
    }
    // set params
    chassis_msg_ = &control_input_stream.input_stream.chassis;
    cmd_msg_ = &(cmd_stream->control_command);

    if (ControlMath::CheckBackward(chassis_msg_)) {
        cmd_stream->control_command.set_driving_mode(pnc::chassis::COMPLETE_MANUAL);
        LOG_ERROR("Wrong gear location! set accel = %.4f", cmd_msg_->acceleration());
        return false;
    }

    UpdateChassisStatus();

    // test item
    switch (chassis_test_conf_.enable_test()) {
    case pnc::control::ChassisTestConf::LON_STANDSTILL_TEST:
        LonStandStillTest();
        break;
    case pnc::control::ChassisTestConf::LON_IDLESPEED_TEST:
        LonIdleSpeedTest();
        break;
    case pnc::control::ChassisTestConf::LON_STEP_TEST:
        LonStepTest();
        break;
    case pnc::control::ChassisTestConf::LON_SINUSOID_TEST:
        LonSinusoidTest();
        break;
    case pnc::control::ChassisTestConf::LAT_STANDSTILL_TEST:
        LatStandStillTest();
        break;
    case pnc::control::ChassisTestConf::LAT_STEP_TEST:
        LatStepTest();
        break;
    case pnc::control::ChassisTestConf::LAT_SINUSOID_TEST:
        LatSinusoidTest();
        break;
    case pnc::control::ChassisTestConf::DISABLE_TEST:
    default:
        control_cmd_.cmd_gear_ = pnc::chassis::GEAR_PARKING;
        control_cmd_.cmd_accel_ = kStopAcc_;
        control_cmd_.cmd_mode_ = pnc::chassis::COMPLETE_MANUAL;
        break;
    }

    // set ControlCommandStream
    cmd_stream->control_command.set_driving_mode(control_cmd_.cmd_mode_);
    cmd_stream->control_command.set_gear_location(control_cmd_.cmd_gear_);
    cmd_stream->control_command.set_acceleration(control_cmd_.cmd_accel_);
    cmd_stream->control_command.set_steering_target(control_cmd_.cmd_angle_);
    cmd_stream->control_command.set_steering_rate(veh_paras_.max_steer_angle_rate() * 180.0 / M_PI);
    cmd_stream->control_command.set_throttle(0.0);
    cmd_stream->control_command.set_brake(0.0);

    prev_time_ = curr_time;

    if (log_file_ == nullptr && !InitLogFile()) {
        LOG_ERROR("Failed to write log file!");
        return false;
    }
    fprintf(log_file_,
            "%.4ld, %.4f, %.4f, %.4f, %.4f, %d, %d, "
            "%.4f, %.4f, %.4f, %.4f,%.4f, %.4f, %d, %d\n",
            curr_time, chassis_status_.curr_speed_mps_, chassis_status_.curr_speed_kph_, control_cmd_.cmd_accel_,
            chassis_status_.curr_accel_, control_cmd_.cmd_gear_, chassis_status_.curr_gear_, control_cmd_.cmd_angle_,
            veh_paras_.max_steer_angle_rate() * 180.0 / M_PI, chassis_status_.curr_angle_,
            chassis_status_.curr_angle_rate_, chassis_msg_->throttle_percentage(), chassis_msg_->brake_percentage(),
            control_cmd_.cmd_mode_, chassis_status_.curr_mode_);
    return true;
}

}  // namespace control
}  // namespace jiduauto
