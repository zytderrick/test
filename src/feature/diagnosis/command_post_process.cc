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

#include "control/src/feature/diagnosis/command_post_process.h"

#include <cmath>
#include <tuple>

#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/vehicle/vehicle_utils.h"

namespace jiduauto {
namespace control {

bool CommandPostProcess::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("Diagnosis: CommandPostProcess Init, starting ...");

    if (!SanityCheck(control_conf)) {
        LOG_ERROR("Diagnosis: SanityCheck failed");
        return false;
    }

    control_conf_ = control_conf;

    return true;
}

bool CommandPostProcess::SanityCheck(const pnc::control::ControlConfig& control_conf) {
    // Note: Please synchronize the new parameters to ./path_to_unit_test/data/control_conf.pb.txt

    if (!control_conf.has_diag_v2_conf() || !control_conf.diag_v2_conf().has_diagnosis_command_post_process_conf()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: diag_v2_conf or diagnosis_command_post_process_conf");
        return false;
    }

    const pnc::control::DiagnosisCommandPostProcessConf diagnosis_command_post_process_conf =
        control_conf.diag_v2_conf().diagnosis_command_post_process_conf();
    if (!diagnosis_command_post_process_conf.has_limit_control_command_continuous_abnormal()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: limit_control_command_continuous_abnormal");
        return false;
    }

    return true;
}

void CommandPostProcess::Check(const bool is_fault_detect_passed,
                               const pnc::control::ControlCommand* const control_command_ptr,
                               pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    // check output
    if (diag_debug_ptr == nullptr) {
        LOG_WARN("Diagnosis: diag_debug_ptr is nullptr");
        return;
    }

    // if normal
    diag_debug_ptr->set_post_check_code(pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_OK);
    diag_debug_ptr->set_post_check_string("normal");

    if (!estop_hold_) {
        estop_hold_reason_.clear();
    }

    // check fault detect result
    if (!is_fault_detect_passed) {
        diag_debug_ptr->set_post_check_code(
            pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_FAULT_DETECT_FALL_BACK);
        diag_debug_ptr->set_post_check_string("pre check not passed");
        if (!estop_hold_) {
            if (diag_debug_ptr->chassis_check_code() > 0) {
                estop_hold_reason_ += "chassis-" + diag_debug_ptr->chassis_check_string() + " ";
            }
            if (diag_debug_ptr->localization_check_code() > 0) {
                estop_hold_reason_ += "localization-" + diag_debug_ptr->localization_check_string() + " ";
            }
            if (diag_debug_ptr->trajectory_check_code() > 0) {
                estop_hold_reason_ += "trajectory-" + diag_debug_ptr->trajectory_check_string() + " ";
            }
            if (diag_debug_ptr->actuator_abnormal()) {
                estop_hold_reason_ += "actuator-" + diag_debug_ptr->actuator_check_string() + " ";
            }
        }
        LOG_WARN("Diagnosis: DIAGNOSIS_POST_CHECK_FAULT_DETECT_FALL_BACK");
        return;
    }

    // check continuous normal
    if (CheckContinuousNormal() == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
        diag_debug_ptr->set_post_check_code(
            pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_COMMAND_CONTINUOUS_ABNORMAL);
        diag_debug_ptr->set_post_check_string("continuous command abnormal");
        if (!estop_hold_) {
            estop_hold_reason_ += "continuous command abnormal";
            estop_hold_reason_ += " ";
        }
        LOG_WARN("Diagnosis: DIAGNOSIS_POST_CHECK_COMMAND_CONTINUOUS_ABNORMAL");
        return;
    }

    // check if estop_hold needed
    if (estop_hold_) {
        diag_debug_ptr->set_post_check_code(pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_ESTOP_HOLD);
        std::string estop_hold_string = "estop hold";
        if (!estop_hold_reason_.empty()) {
            estop_hold_string += ": " + estop_hold_reason_;
        }
        diag_debug_ptr->set_post_check_string(estop_hold_string);
        LOG_WARN("Diagnosis: DIAGNOSIS_POST_CHECK_ESTOP_HOLD");
        return;
    }

    // check command
    // Note: (Discuss with Bolin on 2023.04.28) ensure that control_command_ptr != nullptr in other file
    switch (CheckCommand(control_command_ptr)) {
    case pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TOTAL_COMMAND_ABNORMAL: {
        diag_debug_ptr->set_post_check_code(
            pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_TOTAL_COMMAND_ABNORMAL);
        diag_debug_ptr->set_post_check_string("total command abnormal");
        LOG_WARN("Diagnosis: DIAGNOSIS_POST_CHECK_TOTAL_COMMAND_ABNORMAL");
        return;
    }
    case pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_LATERAL_COMMAND_ABNORMAL: {
        diag_debug_ptr->set_post_check_code(
            pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_LATERAL_COMMAND_ABNORMAL);
        diag_debug_ptr->set_post_check_string("lateral command abnormal");
        LOG_WARN("Diagnosis: DIAGNOSIS_POST_CHECK_LATERAL_COMMAND_ABNORMAL");
        return;
    }
    case pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_LONGITUDINAL_COMMAND_ABNORMAL: {
        diag_debug_ptr->set_post_check_code(
            pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_LONGITUDINAL_COMMAND_ABNORMAL);
        diag_debug_ptr->set_post_check_string("longitudinal command abnormal");
        LOG_WARN("Diagnosis: DIAGNOSIS_POST_CHECK_LONGITUDINAL_COMMAND_ABNORMAL");
        return;
    }
    default:
        diag_debug_ptr->set_post_check_code(pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_OK);
        diag_debug_ptr->set_post_check_string("ok");
    }
}

pnc::control::DiagnosisReturnCode CommandPostProcess::CheckContinuousNormal() {
    if (!control_conf_.has_diag_v2_conf() || !control_conf_.diag_v2_conf().has_diagnosis_command_post_process_conf() ||
        !control_conf_.diag_v2_conf()
             .diagnosis_command_post_process_conf()
             .has_limit_control_command_continuous_abnormal()) {
        LOG_ERROR(
            "Diagnsosis: Fail to check conf paras: diag_v2_conf or diagnosis_command_post_process_conf or "
            "limit_control_command_continuous_abnormal");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    const int kLimitCountCommandContinuousAbnormal =
        control_conf_.diag_v2_conf().diagnosis_command_post_process_conf().limit_control_command_continuous_abnormal();

    if (global_paras_.count_command_continuous_abnormal > kLimitCountCommandContinuousAbnormal) {
        LOG_INFO(
            "Diagnosis: count_command_continuous_abnormal: %d is greater than kLimitCountCommandContinuousAbnormal: %d",
            global_paras_.count_command_continuous_abnormal, kLimitCountCommandContinuousAbnormal);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

pnc::control::DiagnosisReturnCode CommandPostProcess::CheckCommand(
    const pnc::control::ControlCommand* const control_command_ptr) {
    // if normal，design redundancy
    pnc::control::DiagnosisReturnCode check_code = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;

    // check LATERAL_COMMAND
    if (!control_command_ptr->has_steering_target()) {
        check_code = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_LATERAL_COMMAND_ABNORMAL;
        LOG_WARN("Diagnosis: STEERING_TARGET_IS_EMPTY");
    }

    if (control_command_ptr->has_steering_target() && std::isnan(control_command_ptr->steering_target())) {
        check_code = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_LATERAL_COMMAND_ABNORMAL;
        LOG_WARN("Diagnosis: STEERING_TARGET_IS_NAN");
    }

    // check LONGITUDINAL_COMMAND
    bool bflag_longitudinal_command_abnormal = false;
    if (!control_command_ptr->has_throttle()) {
        bflag_longitudinal_command_abnormal = true;
        LOG_WARN("Diagnosis: THROTTLE_IS_EMPTY");
    }

    if (control_command_ptr->has_throttle() && std::isnan(control_command_ptr->throttle())) {
        bflag_longitudinal_command_abnormal = true;
        LOG_WARN("Diagnosis: THROTTLE_IS_NAN");
    }

    if (!control_command_ptr->has_brake()) {
        bflag_longitudinal_command_abnormal = true;
        LOG_WARN("Diagnosis: BREAK_IS_EMPTY");
    }

    if (control_command_ptr->has_brake() && std::isnan(control_command_ptr->brake())) {
        bflag_longitudinal_command_abnormal = true;
        LOG_WARN("Diagnosis: BREAK_IS_NAN");
    }

    if (!control_command_ptr->has_acceleration()) {
        bflag_longitudinal_command_abnormal = true;
        LOG_WARN("Diagnosis: ACCELERATION_IS_EMPTY");
    }

    if (control_command_ptr->has_acceleration() && std::isnan(control_command_ptr->acceleration())) {
        bflag_longitudinal_command_abnormal = true;
        LOG_WARN("Diagnosis: ACCELERATION_IS_NAN");
    }

    // return result
    if (bflag_longitudinal_command_abnormal) {
        if (check_code == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_LATERAL_COMMAND_ABNORMAL) {
            check_code = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TOTAL_COMMAND_ABNORMAL;
        } else {
            check_code = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_LONGITUDINAL_COMMAND_ABNORMAL;
        }
    }

    return check_code;
}

void CommandPostProcess::React(const ControlInputStream& control_input_stream,
                               pnc::control::ControlCommand* const control_command_ptr,
                               pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    // Note: (Discuss with Bolin on 2023.04.28) ensure that control_command_ptr != nullptr in other file
    // check output
    if (diag_debug_ptr == nullptr) {
        LOG_WARN("Diagnosis: diag_debug_ptr is nullptr");
        return;
    }

    // react
    switch (diag_debug_ptr->post_check_code()) {
        // react normal
    case pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_OK: {
        diag_debug_ptr->set_reaction_code(pnc::control::DiagnosisReactCode::DIAGNOSIS_REACT_NONE);
        diag_debug_ptr->set_reaction_string("none");
        // LOG_INFO("Diagnosis: DiagnosisCode::NONE");
        return;
    }
        // react fault detect fall back or command continuous abnormal
    case pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_FAULT_DETECT_FALL_BACK:
    case pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_ESTOP_HOLD:
    case pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_COMMAND_CONTINUOUS_ABNORMAL: {
        SoftBrake(control_input_stream.vehicle_state, control_command_ptr);
        // LOG_INFO("Diagnosis: SOFT_BRAKE");
        diag_debug_ptr->set_reaction_code(pnc::control::DiagnosisReactCode::DIAGNOSIS_REACT_SOFT_BRAKE);
        diag_debug_ptr->set_reaction_string("soft brake");
        return;
    }
    // ReactLastLateralCommand
    case pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_LATERAL_COMMAND_ABNORMAL: {
        ReactLastLateralCommand(control_command_ptr);
        // LOG_INFO("Diagnosis: ReactLastLateralCommand");
        diag_debug_ptr->set_reaction_code(pnc::control::DiagnosisReactCode::DIAGNOSIS_REACT_LAST_LATERAL_COMMAND);
        diag_debug_ptr->set_reaction_string("hold last steer command");
        return;
    }
    // ReactLastLongitudinalCommand
    case pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_LONGITUDINAL_COMMAND_ABNORMAL: {
        ReactLastLongitudinalCommand(control_command_ptr);
        // LOG_INFO("Diagnosis: ReactLastLongitudinalCommand");
        diag_debug_ptr->set_reaction_code(pnc::control::DiagnosisReactCode::DIAGNOSIS_REACT_LAST_LONGITUDINAL_COMMAND);
        diag_debug_ptr->set_reaction_string("hold last acc command");
        return;
    }
    // ReactLastLateralCommand & ReactLastLongitudinalCommand
    case pnc::control::DiagnosisPostCheckCode::DIAGNOSIS_POST_CHECK_TOTAL_COMMAND_ABNORMAL: {
        ReactLastLateralCommand(control_command_ptr);
        ReactLastLongitudinalCommand(control_command_ptr);
        // LOG_INFO("Diagnosis: ReactLastLateralCommand & ReactLastLongitudinalCommand");
        diag_debug_ptr->set_reaction_code(pnc::control::DiagnosisReactCode::DIAGNOSIS_REACT_LAST_COMMAND);
        diag_debug_ptr->set_reaction_string("hold last control command");
        return;
    }
    default: {
        // do nothing
        LOG_INFO("Diagnosis: Maybe Error");
        return;
    }
    }
}

void CommandPostProcess::SoftBrake(const pnc::common::VehicleState& vehicle_state,
                                   pnc::control::ControlCommand* const control_command_ptr) {
    const double kSoftBrakeAcc = -std::fabs(control_conf_.post_process_conf().soft_brake_acc());
    control_command_ptr->set_throttle(0.0);
    control_command_ptr->set_brake(-30.0);
    control_command_ptr->set_acceleration(kSoftBrakeAcc);
    control_command_ptr->set_steering_target(0.0);
    const double kEstopParkingThreshold = 0.3;
    if (vehicle_state.linear_velocity() < kEstopParkingThreshold &&
        vehicle_state.localization_velocity() < kEstopParkingThreshold) {
        control_command_ptr->set_gear_location(pnc::chassis::GEAR_PARKING);
    } else {
        control_command_ptr->set_gear_location(vehicle_state.gear());
    }
    if (vehicle_state.driving_mode() != pnc::chassis::DrivingMode::COMPLETE_MANUAL &&
        control_conf_.diag_v2_conf().diagnosis_command_post_process_conf().enable_control_estop_hold()) {
        estop_hold_ = true;
    } else {
        estop_hold_ = false;
    }
    std::tuple<bool, bool> drive_off_req(pnc::vehicle::GenerateDriveOffRequest(
        vehicle_state.gear(), control_command_ptr->gear_location(), control_command_ptr->acceleration(), true, true));

    control_command_ptr->set_driveoff_req(std::get<0>(drive_off_req));
    control_command_ptr->set_standstill_req(std::get<1>(drive_off_req));
}

void CommandPostProcess::ReactLastLateralCommand(pnc::control::ControlCommand* const control_command_ptr) {
    control_command_ptr->set_steering_target(global_paras_.last_control_command.steer_target);
}

void CommandPostProcess::ReactLastLongitudinalCommand(pnc::control::ControlCommand* const control_command_ptr) {
    control_command_ptr->set_throttle(global_paras_.last_control_command.throttle);
    control_command_ptr->set_brake(global_paras_.last_control_command.brake);
    control_command_ptr->set_acceleration(global_paras_.last_control_command.acceleration);
}

void CommandPostProcess::UpdateLastCommand(const pnc::control::ControlCommand* const control_command_ptr) {
    if (control_command_ptr == nullptr) {
        LOG_ERROR("Diagnosis: control_command_ptr is nullptr, fail to UpdateLastCommand");
        return;
    }
    global_paras_.last_control_command.steer_target = control_command_ptr->steering_target();
    global_paras_.last_control_command.throttle = control_command_ptr->throttle();
    global_paras_.last_control_command.brake = control_command_ptr->brake();
    global_paras_.last_control_command.acceleration = control_command_ptr->acceleration();
}

}  // namespace control
}  // namespace jiduauto
