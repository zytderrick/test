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

#include "control/src/feature/diagnosis/actuator_check.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::util::TimeUtility;

bool ActuatorCheck::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("Diagnosis: ActuatorCheck Init, starting ...");
    control_conf_ = control_conf;
    veh_paras_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();

    if (!SanityCheck(control_conf)) {
        LOG_ERROR("Diagnosis: SanityCheck failed");
        return false;
    }

    global_paras_.steer_angle_offset_paras.steer_angle_offset_mean = veh_paras_.steer_angle_offset();
    global_paras_.check_result_string_vector.clear();

    return true;
}

void ActuatorCheck::Reset() {
    ClearQueue(&global_paras_.steer_angle_offset_paras.speed_que);
    ClearQueue(&global_paras_.steer_angle_offset_paras.heading_que);
    ClearQueue(&global_paras_.steer_angle_offset_paras.steering_angle_que);
    ClearQueue(&global_paras_.steer_angle_offset_paras.steering_offset_que);
    global_paras_.steer_angle_offset_paras.steer_angle_offset_mean = 0.0;
    global_paras_.steer_angle_offset_paras.temp_steer_angle_offset = 0.0;
    acc_cmd_vec_.clear();
    acc_feedback_vec_.clear();
    speed_vec_.clear();
    time_sec_vec_.clear();
    steer_match_list_.clear();
}

bool ActuatorCheck::SanityCheck(const pnc::control::ControlConfig& control_conf) {
    // Note: Please synchronize the new parameters to ./path_to_unit_test/data/control_conf.pb.txt
    if (!control_conf.has_control_period()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: control_period");
        return false;
    }

    // check veh_paras_
    if (!veh_paras_.has_steer_angle_offset() || !veh_paras_.has_wheel_base() || !veh_paras_.has_steer_ratio()) {
        LOG_ERROR("Diagnosis: Fail to check veh_paras_: steer_angle_offset or wheelbase or steer_ratio");
        return false;
    }

    return true;
}

bool ActuatorCheck::FaultCheck(const ControlInputStream& control_input_stream,
                               pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    // check output
    if (diag_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: diag_debug_ptr is nullptr");
        return false;
    }
    global_paras_.check_result_string_vector.clear();
    global_paras_.actuator_check_code = 0;
    SetCodeBitHigh(&global_paras_.actuator_check_code, pnc::control::ActuatorCheckBit::ACTUATOR_UNKNOWN_BIT);

    pnc::control::ControlCommand last_control_cmd = control_input_stream.input_stream.control_command;
    pnc::common::VehicleState vehicle_state = control_input_stream.vehicle_state;
    bool bflag = true;
    if ((control_input_stream.input_stream.pad_info.action() == pnc::control::START) &&
        control_conf_.diag_v2_conf().actuator_check_conf().enable_chassis_ready_check() &&
        !ChassisReadyCheckPass(vehicle_state,
                               diag_debug_ptr->mutable_acutator_check_debug()->mutable_chassis_ready_debug())) {
        LOG_DEBUG("Diagnosis: ChassisReadyCheckPass failed");
        global_paras_.check_result_string_vector.push_back("chassis unready");
        SetCodeBitHigh(&global_paras_.actuator_check_code, pnc::control::ActuatorCheckBit::ACTUATOR_CHASSIS_READY_BIT);
        bflag = false;
    }

    if (!PadmsgStartResponseCheckPass(
            vehicle_state, last_control_cmd,
            diag_debug_ptr->mutable_acutator_check_debug()->mutable_padmsg_response_debug())) {
        LOG_DEBUG("Diagnosis: PadmsgStartResponseCheckPass failed");
        global_paras_.check_result_string_vector.push_back("padmsg response check failed");
        SetCodeBitHigh(&global_paras_.actuator_check_code, pnc::control::ActuatorCheckBit::ACTUATOR_PADMSG_BIT);
        bflag = false;
    }

    // todo(bolin): only check in automode now
    if (vehicle_state.driving_mode() == pnc::chassis::COMPLETE_AUTO_DRIVE) {
        if (control_conf_.diag_v2_conf().actuator_check_conf().enable_steer_offset_check() &&
            !SteerAngleOffsetCheck(
                vehicle_state, diag_debug_ptr->mutable_acutator_check_debug()->mutable_steer_angle_offset_debug())) {
            LOG_DEBUG("Diagnosis: SteerAngleOffsetCheck failed");
            global_paras_.check_result_string_vector.push_back("steer offset failed");
            bflag = false;
        }

        if (control_conf_.diag_v2_conf().actuator_check_conf().enable_acc_check() &&
            !LongitudinalCheckPass(vehicle_state, last_control_cmd,
                                   diag_debug_ptr->mutable_acutator_check_debug()->mutable_acc_pedal_debug())) {
            LOG_DEBUG("Diagnosis: Longitudinal Response Check failed");
            global_paras_.check_result_string_vector.push_back("acc response failed");
            SetCodeBitHigh(&global_paras_.actuator_check_code, pnc::control::ActuatorCheckBit::ACTUATOR_ACC_BIT);
            bflag = false;
        }

        if (control_conf_.diag_v2_conf().actuator_check_conf().enable_gear_check() &&
            !GearResponseCheckPass(vehicle_state, last_control_cmd,
                                   diag_debug_ptr->mutable_acutator_check_debug()->mutable_gear_debug())) {
            LOG_DEBUG("Diagnosis: Gear Response Check failed");
            global_paras_.check_result_string_vector.push_back("gear response failed");
            SetCodeBitHigh(&global_paras_.actuator_check_code, pnc::control::ActuatorCheckBit::ACTUATOR_GEAR_BIT);
            bflag = false;
        }

        if (control_conf_.diag_v2_conf().actuator_check_conf().enable_steer_check() &&
            !SteerResponseCheckPass(vehicle_state, last_control_cmd,
                                    diag_debug_ptr->mutable_acutator_check_debug()->mutable_steer_debug())) {
            LOG_DEBUG("Diagnosis: Steer Response Check failed");
            global_paras_.check_result_string_vector.push_back("steer response failed");
            SetCodeBitHigh(&global_paras_.actuator_check_code, pnc::control::ActuatorCheckBit::ACTUATOR_STEER_BIT);
            bflag = false;
        }
    } else {
        Reset();
    }

    SetCodeBitLow(&global_paras_.actuator_check_code, pnc::control::ActuatorCheckBit::ACTUATOR_UNKNOWN_BIT);
    if (global_paras_.actuator_check_code ^ 0x0) {
        SetCodeBitHigh(&global_paras_.actuator_check_code, pnc::control::ActuatorCheckBit::ACTUATOR_SUMMARY_BIT);
    }

    std::string check_string;
    if (global_paras_.check_result_string_vector.empty()) {
        check_string = "No error";
    } else {
        check_string = global_paras_.check_result_string_vector[0];
        if (static_cast<int>(global_paras_.check_result_string_vector.size()) > 1) {
            for (int i = 1; i < static_cast<int>(global_paras_.check_result_string_vector.size()); ++i) {
                check_string += " & ";
                check_string += global_paras_.check_result_string_vector[i];
            }
        }
    }

    diag_debug_ptr->set_actuator_check_string(check_string);
    diag_debug_ptr->set_actuator_check_code(global_paras_.actuator_check_code);
    diag_debug_ptr->set_actuator_abnormal(!bflag);
    return bflag;
}

bool ActuatorCheck::SteerAngleOffsetCheck(const jiduauto::pnc::common::VehicleState& vehicle_state,
                                          pnc::control::SteerAngleOffsetDebug* const steer_angle_offset_debug_ptr) {
    // TODO(tan.ju): try replacing yaw_rate with dot_heading
    // check output
    if (steer_angle_offset_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: steer_angle_offset_debug_ptr is nullptr");
        return false;
    }
    // set debug proto, if normal
    steer_angle_offset_debug_ptr->set_is_steer_angle_offset_mean_update(false);
    steer_angle_offset_debug_ptr->set_steer_angle_offset_after_check(
        global_paras_.steer_angle_offset_paras.steer_angle_offset_mean);
    steer_angle_offset_debug_ptr->set_temp_steer_angle_offset(
        global_paras_.steer_angle_offset_paras.temp_steer_angle_offset);

    // check input
    if (!vehicle_state.has_heading() || std::isnan(vehicle_state.heading()) || !vehicle_state.has_linear_velocity() ||
        std::isnan(vehicle_state.linear_velocity()) || !vehicle_state.has_steering_angle() ||
        std::isnan(vehicle_state.steering_angle())) {
        LOG_ERROR("Diagnosis: Input is abnormal, heading or linear_velocity or steering_angle");
        // diag_debug_ptr->mutable_actuator_check_debug()->set_steer_offset_check_string("Input is abnormal");
        return false;
    }

    // get input
    const double cur_speed = vehicle_state.linear_velocity();
    const double cur_heading = vehicle_state.heading();
    const double cur_steering_angle = vehicle_state.steering_angle();

    // ensure cur_speed greater than kSpeedLowLimit
    const double kSpeedLowLimit = 0.27;
    if (cur_speed <= kSpeedLowLimit) {
        ClearQueue(&global_paras_.steer_angle_offset_paras.speed_que);
        ClearQueue(&global_paras_.steer_angle_offset_paras.heading_que);
        ClearQueue(&global_paras_.steer_angle_offset_paras.steering_angle_que);
        return true;
    }

    // directly push the first variable into queue
    if (global_paras_.steer_angle_offset_paras.speed_que.empty()) {
        global_paras_.steer_angle_offset_paras.speed_que.push(cur_speed);
        global_paras_.steer_angle_offset_paras.heading_que.push(cur_heading);
        global_paras_.steer_angle_offset_paras.steering_angle_que.push(cur_steering_angle);
        return true;
    }

    // Delete data: heading jumps from pi to -pi or from -pi to pi
    if (abs(cur_heading - global_paras_.steer_angle_offset_paras.heading_que.back()) > M_PI) {
        ClearQueue(&global_paras_.steer_angle_offset_paras.speed_que);
        ClearQueue(&global_paras_.steer_angle_offset_paras.heading_que);
        ClearQueue(&global_paras_.steer_angle_offset_paras.steering_angle_que);
        global_paras_.steer_angle_offset_paras.speed_que.push(cur_speed);
        global_paras_.steer_angle_offset_paras.heading_que.push(cur_heading);
        global_paras_.steer_angle_offset_paras.steering_angle_que.push(cur_steering_angle);
        return true;
    }

    // Record data when vehicle is at constant speed && in a straight line
    double speed_mean = GetQueueMean(global_paras_.steer_angle_offset_paras.speed_que);
    speed_mean = std::max(speed_mean, 1e-3);
    const double kSpeedFluctuatingPercent = 1.0 * 0.01;
    const double kRangeSteerFeedbackDeg = 0.1;
    double steering_angle_mean = GetQueueMean(global_paras_.steer_angle_offset_paras.steering_angle_que);
    if ((std::abs((cur_speed - speed_mean) / speed_mean) > kSpeedFluctuatingPercent) ||
        std::abs(cur_steering_angle - steering_angle_mean) > kRangeSteerFeedbackDeg) {
        ClearQueue(&global_paras_.steer_angle_offset_paras.speed_que);
        ClearQueue(&global_paras_.steer_angle_offset_paras.heading_que);
        ClearQueue(&global_paras_.steer_angle_offset_paras.steering_angle_que);
        global_paras_.steer_angle_offset_paras.speed_que.push(cur_speed);
        global_paras_.steer_angle_offset_paras.heading_que.push(cur_heading);
        global_paras_.steer_angle_offset_paras.steering_angle_que.push(cur_steering_angle);
        return true;
    }

    // normal, push data
    global_paras_.steer_angle_offset_paras.speed_que.push(cur_speed);
    global_paras_.steer_angle_offset_paras.heading_que.push(cur_heading);
    global_paras_.steer_angle_offset_paras.steering_angle_que.push(cur_steering_angle);

    // update steer_angle_offset
    const double kTotalTime = 3.0;  // Note: Variable parameter，Modify according to test result
    if (!control_conf_.has_control_period()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: control_period");
        return false;
    }
    int total_size = static_cast<int>(kTotalTime / control_conf_.control_period());
    if (static_cast<int>(global_paras_.steer_angle_offset_paras.speed_que.size()) >= total_size) {
        double dlt_heading =
            jiduauto::pnc::math::NormalizeAngle(global_paras_.steer_angle_offset_paras.heading_que.back() -
                                                global_paras_.steer_angle_offset_paras.heading_que.front());
        double dot_heading = dlt_heading / kTotalTime;
        speed_mean = GetQueueMean(global_paras_.steer_angle_offset_paras.speed_que);
        if (!veh_paras_.has_wheel_base() || std::isnan(veh_paras_.wheel_base()) || !veh_paras_.has_steer_ratio() ||
            std::isnan(veh_paras_.steer_ratio())) {
            LOG_ERROR("Diagnosis: Fail to check veh_paras_ : wheelbase or steer_ratio");
            return false;
        }
        double steer_cal_deg =
            dot_heading / speed_mean * veh_paras_.wheel_base() * veh_paras_.steer_ratio() * 180.0 / M_PI;
        steering_angle_mean = GetQueueMean(global_paras_.steer_angle_offset_paras.steering_angle_que);
        double new_steer_offset_deg = steer_cal_deg - steering_angle_mean;
        const int kSteerOffsetQueMaxsize = 1000;  // Note: Variable parameter，Modify according to test result
        if (static_cast<int>(global_paras_.steer_angle_offset_paras.steering_offset_que.size()) >=
            kSteerOffsetQueMaxsize) {
            global_paras_.steer_angle_offset_paras.steering_offset_que.pop();
        }
        global_paras_.steer_angle_offset_paras.steering_offset_que.push(new_steer_offset_deg);
        global_paras_.steer_angle_offset_paras.temp_steer_angle_offset = new_steer_offset_deg;
        global_paras_.steer_angle_offset_paras.steer_angle_offset_mean =
            GetQueueMean(global_paras_.steer_angle_offset_paras.steering_offset_que);
        steer_angle_offset_debug_ptr->set_is_steer_angle_offset_mean_update(true);
        steer_angle_offset_debug_ptr->set_steer_angle_offset_after_check(
            global_paras_.steer_angle_offset_paras.steer_angle_offset_mean);
        steer_angle_offset_debug_ptr->set_temp_steer_angle_offset(
            global_paras_.steer_angle_offset_paras.temp_steer_angle_offset);

        // keep queue.size() less than total_size
        global_paras_.steer_angle_offset_paras.speed_que.pop();
        global_paras_.steer_angle_offset_paras.heading_que.pop();
        global_paras_.steer_angle_offset_paras.steering_angle_que.pop();
    }

    return true;
}

bool ActuatorCheck::LongitudinalCheckPass(const pnc::common::VehicleState& vehicle_state,
                                          const pnc::control::ControlCommand& last_cmd,
                                          pnc::control::AccelerationPedalDebug* const acc_pedal_debug_ptr) {
    // This function to check whether the longitude response is correct
    // Check Last cmd and Current state
    // TODO(zhaobolin): add doc in wiki
    bool check_passed = true;

    acc_cmd_vec_.push_back(last_cmd.acceleration());
    acc_feedback_vec_.push_back(vehicle_state.linear_acceleration());
    speed_vec_.push_back(vehicle_state.linear_velocity());
    time_sec_vec_.push_back(vehicle_state.timestamp_sec());
    if (acc_cmd_vec_.size() > control_conf_.diag_v2_conf().actuator_check_conf().long_check_length()) {
        acc_cmd_vec_.erase(acc_cmd_vec_.begin());
        acc_feedback_vec_.erase(acc_feedback_vec_.begin());
        speed_vec_.erase(speed_vec_.begin());
        time_sec_vec_.erase(time_sec_vec_.begin());
    }
    const double acc_cmd_mean =
        std::accumulate(std::begin(acc_cmd_vec_), std::end(acc_cmd_vec_), 0.0) / acc_cmd_vec_.size();
    const double acc_feedback_mean =
        std::accumulate(std::begin(acc_feedback_vec_), std::end(acc_feedback_vec_), 0.0) / acc_feedback_vec_.size();

    double sum_diff_acc = 0.0;
    const double kEpsilon = 1.0e-6;
    int effect_length = 0;
    for (int i = 0; i < static_cast<int>(speed_vec_.size()) - 1; i++) {
        if (time_sec_vec_[i + 1] - time_sec_vec_[i] > kEpsilon) {
            sum_diff_acc =
                sum_diff_acc + (speed_vec_[i + 1] - speed_vec_[i]) / (time_sec_vec_[i + 1] - time_sec_vec_[i]);
            effect_length++;
        }
    }

    const double acc_spd_diff_mean = sum_diff_acc / static_cast<double>(effect_length);

    acc_pedal_debug_ptr->set_acc_spd_diff_mean(acc_spd_diff_mean);
    acc_pedal_debug_ptr->set_acc_cmd_mean(acc_cmd_mean);
    acc_pedal_debug_ptr->set_acc_feedback_mean(acc_feedback_mean);

    if (std::abs(std::abs(acc_spd_diff_mean) - std::abs(acc_feedback_mean)) >
        control_conf_.diag_v2_conf().actuator_check_conf().acc_diff_threshold()) {
        check_passed = false;
        acc_pedal_debug_ptr->set_acc_pedal_abnormal(!check_passed);
    }

    return check_passed;
}

bool ActuatorCheck::GearResponseCheckPass(const pnc::common::VehicleState& vehicle_state,
                                          const pnc::control::ControlCommand& last_cmd,
                                          pnc::control::GearDebug* const gear_debug_ptr) {
    // This check only for abnormal between control cmd gear and chassis gear. Not for planning gear
    const int kGearCheckLength = control_conf_.diag_v2_conf().actuator_check_conf().gear_check_length();

    bool gear_check_passed = false;
    gear_match_list_.push_back(last_cmd.gear_location() != vehicle_state.gear());

    if (gear_match_list_.size() > kGearCheckLength) {
        gear_match_list_.pop_front();
    }

    if (std::accumulate(std::begin(gear_match_list_), std::end(gear_match_list_), 0) == kGearCheckLength) {
        gear_check_passed = false;
    } else {
        gear_check_passed = true;
    }

    gear_debug_ptr->set_gear_abnormal(!gear_check_passed);
    return gear_check_passed;
}

bool ActuatorCheck::SteerResponseCheckPass(const pnc::common::VehicleState& vehicle_state,
                                           const pnc::control::ControlCommand& last_cmd,
                                           pnc::control::SteerDebug* const steer_debug_ptr) {
    const int kSteerCheckLength = control_conf_.diag_v2_conf().actuator_check_conf().steer_check_length();
    bool steer_check_passed = false;

    steer_match_list_.push_back(std::fabs(last_cmd.steering_target() - vehicle_state.steering_angle()) >
                                control_conf_.diag_v2_conf().actuator_check_conf().swa_diff_threshold_deg());

    if (steer_match_list_.size() > kSteerCheckLength) {
        steer_match_list_.pop_front();
    }

    if (std::accumulate(std::begin(steer_match_list_), std::end(steer_match_list_), 0) == kSteerCheckLength) {
        steer_check_passed = false;
    } else {
        steer_check_passed = true;
    }

    steer_debug_ptr->set_steer_abnormal(!steer_check_passed);
    return steer_check_passed;
}

bool ActuatorCheck::ChassisReadyCheckPass(const pnc::common::VehicleState& vehicle_state,
                                          pnc::control::ChassisReadyDebug* const chassis_ready_debug) {
    bool chassis_ready_passed{true};
    if (vehicle_state.chassis_ready_st() != pnc::chassis::CHASSIS_READY) {
        if (bflag_first_chassis_unready_) {
            first_chassis_unready_timestamp_ = TimeUtility::GetCurrentTimesecond();
            bflag_first_chassis_unready_ = false;
        }
        double unready_time_duration = TimeUtility::GetCurrentTimesecond() - first_chassis_unready_timestamp_;
        if (unready_time_duration > control_conf_.diag_v2_conf().actuator_check_conf().chassis_ready_wait_time_sec()) {
            chassis_ready_passed = false;
        }
    } else {
        chassis_ready_passed = true;
    }

    chassis_ready_debug->set_chassis_ready_abnormal(!chassis_ready_passed);
    return chassis_ready_passed;
}

bool ActuatorCheck::PadmsgStartResponseCheckPass(const pnc::common::VehicleState& vehicle_state,
                                                 const pnc::control::ControlCommand& last_cmd,
                                                 pnc::control::PadmsgResponseDebug* const padmsg_response_debug) {
    bool padmsg_response_passed{true};
    if (last_cmd.pad_msg().action() == pnc::control::STOP) {
        manual_driving_list_.clear();
        padmsg_response_passed = true;
    }
    if (last_cmd.pad_msg().action() == pnc::control::START) {
        manual_driving_list_.push_back(vehicle_state.driving_mode() == pnc::chassis::COMPLETE_MANUAL);

        const int kDrivingModeListLength =
            control_conf_.diag_v2_conf().actuator_check_conf().padmsg_response_driving_mode_list_length();
        while (manual_driving_list_.size() > kDrivingModeListLength) {
            manual_driving_list_.pop_front();
        }

        if (std::accumulate(manual_driving_list_.begin(), manual_driving_list_.end(), 0) == kDrivingModeListLength) {
            padmsg_response_passed = false;
        }
    }
    padmsg_response_debug->set_padmsg_response_abnormal(!padmsg_response_passed);
    return padmsg_response_passed;
}

// common function
void ActuatorCheck::ClearQueue(std::queue<double>* q_ptr) {
    std::queue<double> empty;
    swap(empty, *q_ptr);
}

double ActuatorCheck::GetQueueMean(std::queue<double> q) {
    // Warning: the input queue MUST NOT be empty!
    double sum = 0.0;
    int size = static_cast<int>(q.size());

    while (!q.empty()) {
        sum += q.front();
        q.pop();
    }
    return sum / size;
}

}  // namespace control
}  // namespace jiduauto
