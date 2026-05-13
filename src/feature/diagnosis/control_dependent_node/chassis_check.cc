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
#include "control/src/feature/diagnosis/control_dependent_node/chassis_check.h"

#include <algorithm>
#include <cmath>

#include "pnc_common/src/common/pnc_logger.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::util::TimeUtility;

bool ChassisCheck::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("Diagnosis: ChassisCheck Init, starting ...");

    if (!SanityCheck(control_conf)) {
        LOG_ERROR("Diagnosis: SanityCheck failed");
        return false;
    }

    control_conf_ = control_conf;

    return true;
}

void ChassisCheck::Reset() {
    global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    global_paras_.check_result_string_vector.clear();
    global_paras_.chassis_check_code = 0;
    SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_UNKNOWN_BIT);
    ClearQueue(&global_paras_.chassis_check_code_queue);

    global_paras_.general_check_paras.count_timestamp_check = 0;
    global_paras_.general_check_paras.last_timestamp = TimeUtility::GetCurrentTimesecond();

    ClearQueue(&global_paras_.specific_check_paras.chassis_velocity_queue);
    ClearQueue(&global_paras_.specific_check_paras.chassis2localization_velocity_error_queue);
}

std::string ChassisCheck::Name() const { return "Chassis"; }

bool ChassisCheck::SanityCheck(const pnc::control::ControlConfig& control_conf) {
    // Note: Please synchronize the new parameters to ./path_to_unit_test/data/control_conf.pb.txt

    if (!control_conf.has_diag_v2_conf()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: diag_v2_conf");
        return false;
    }

    const pnc::control::DiagnosisV2Conf diag_v2_conf = control_conf.diag_v2_conf();
    if (!diag_v2_conf.has_frames_jump_timestamp_check()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: frames_jump_timestamp_check");
        return false;
    }

    return true;
}

bool ChassisCheck::GeneralCheck(const InputStream& input_stream, pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    global_paras_.chassis_check_code = 0;
    SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_UNKNOWN_BIT);

    // check output
    if (diag_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: Fail to check diag_debug");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        return false;
    }

    // clear chassis_check_result_code & check_result_string_vector
    global_paras_.check_result_string_vector.clear();

    // check pointer
    const pnc::chassis::Chassis* const chassis_ptr = &input_stream.chassis;
    if (chassis_ptr == nullptr) {
        global_paras_.check_result_string_vector.push_back("nullptr");
        SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_NULLPTR_BIT);
        LOG_DEBUG("Diagnosis: chassis_ptr is nullptr");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    // check header
    if (chassis_ptr != nullptr && !chassis_ptr->has_header()) {
        global_paras_.check_result_string_vector.push_back("header empty");
        SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_HEADER_BIT);
        LOG_DEBUG("Diagnosis: chassis header is empty");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    if (chassis_ptr != nullptr && chassis_ptr->has_header()) {
        // check timestamp
        if (std::isnan(chassis_ptr->header().timestamp_sec()) ||
            TimestampCheck(chassis_ptr->header().timestamp_sec(), &global_paras_.general_check_paras) ==
                pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_TIMESTAMP_BIT);
            SetCodeBitHigh(&global_paras_.chassis_check_code,
                           pnc::control::ChassisCheckBit::CHASSIS_TIMESTAMP_LEVEL_BIT);
            global_paras_.check_result_string_vector.push_back("timestamp abnormal");
            LOG_DEBUG("Diagnosis: chassis timestamp_sec is abnormal");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }

        // check gear location
        if (!chassis_ptr->has_gear_location()) {
            global_paras_.check_result_string_vector.push_back("no gear");
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_GEAR_BIT);
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_GEAR_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: chassis gear_location is empty");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        } else {
            switch (chassis_ptr->gear_location()) {
            case pnc::chassis::GEAR_NEUTRAL:
            case pnc::chassis::GEAR_DRIVE:
            case pnc::chassis::GEAR_REVERSE:
            case pnc::chassis::GEAR_PARKING:
                // normal, do nothing
                break;
            default:
                // abnormal
                global_paras_.check_result_string_vector.push_back("gear abnormal");
                SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_GEAR_BIT);
                SetCodeBitHigh(&global_paras_.chassis_check_code,
                               pnc::control::ChassisCheckBit::CHASSIS_GEAR_LEVEL_BIT);
                LOG_DEBUG("Diagnosis: chassis gear_location is abnormal");
                global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
            }
        }

        // check speed nan
        if (!chassis_ptr->has_speed_mps() || std::isnan(chassis_ptr->speed_mps()) ||
            !chassis_ptr->has_chassis_lon_acc() || std::isnan(chassis_ptr->chassis_lon_acc())) {
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SPEED_BIT);
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SPEED_LEVEL_BIT);
            global_paras_.check_result_string_vector.push_back("speed abnormal");
            LOG_DEBUG("Diagnosis: chassis speed/acc is NaN");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }

        // check steer
        if (!chassis_ptr->has_steering_percentage() || std::isnan(chassis_ptr->steering_percentage()) ||
            !chassis_ptr->has_steering_angle_spd() || std::isnan(chassis_ptr->steering_angle_spd()) ||
            !chassis_ptr->has_steering_torque_nm() || std::isnan(chassis_ptr->steering_torque_nm())) {
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_STEER_BIT);
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_STEER_LEVEL_BIT);
            global_paras_.check_result_string_vector.push_back("steer abnormal");
            LOG_DEBUG("Diagnosis: chassis steer percentage/steering_angle_spd/steering_torque_nm is NaN");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }

        // check driving mode
        if (!chassis_ptr->has_driving_mode()) {
            global_paras_.check_result_string_vector.push_back("no driving mode");
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_DRIVING_MODE_BIT);
            SetCodeBitHigh(&global_paras_.chassis_check_code,
                           pnc::control::ChassisCheckBit::CHASSIS_DRIVING_MODE_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: chassis driving mode is empty");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        } else {
            switch (chassis_ptr->driving_mode()) {
            case pnc::chassis::DrivingMode::COMPLETE_MANUAL:
            case pnc::chassis::DrivingMode::COMPLETE_AUTO_DRIVE:
                // normal, do nothing
                break;
            default:
                // abnormal
                global_paras_.check_result_string_vector.push_back("driving mode abnormal");
                SetCodeBitHigh(&global_paras_.chassis_check_code,
                               pnc::control::ChassisCheckBit::CHASSIS_DRIVING_MODE_BIT);
                SetCodeBitHigh(&global_paras_.chassis_check_code,
                               pnc::control::ChassisCheckBit::CHASSIS_DRIVING_MODE_LEVEL_BIT);
                LOG_DEBUG("Diagnosis: chassis driving mode is abnormal");
                global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
            }
        }
    }

    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP) {
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
    }

    bool bflag{true};
    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE) {
        bflag = true;
    } else {
        bflag = false;
        SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT);
        SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_LEVEL_BIT);
    }
    SetCodeBitLow(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_UNKNOWN_BIT);

    SetDebugInfo(global_paras_, diag_debug_ptr);

    return bflag;
}

pnc::control::DiagnosisReturnCode ChassisCheck::TimestampCheck(const double timestamp_sec,
                                                               GeneralCheckParas* const general_check_paras_ptr) {
    double time_interval_control2message =
        TimeUtility::GetCurrentTimesecond() - general_check_paras_ptr->last_timestamp;
    double time_interval_message2last = timestamp_sec - general_check_paras_ptr->last_timestamp;
    general_check_paras_ptr->last_timestamp = TimeUtility::GetCurrentTimesecond();
    if (std::max(std::fabs(time_interval_message2last), time_interval_control2message) >
        control_conf_.diag_v2_conf().chassis_check_conf().max_chassis_interval_sec()) {
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

bool ChassisCheck::SpecificCheck(const ControlInputStream& control_input_stream,
                                 pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    // check output
    if (diag_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: Fail to check diag_debug");
        return false;
    }

    bool bflag{true};
    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP) {
        global_paras_.check_result_string_vector.push_back("diagnosis skip");
        SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_UNKNOWN_BIT);
        bflag = false;
    } else if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
        SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT);
        SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_LEVEL_BIT);
        SetCodeBitLow(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_UNKNOWN_BIT);
        if (global_paras_.check_result_string_vector.empty()) {
            global_paras_.check_result_string_vector.push_back("fail to show error");
        }
        bflag = false;
    } else {
        // check chassis speed compared with localization velocity
        if (VelocityCheck(control_input_stream, &global_paras_.specific_check_paras) ==
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            LOG_DEBUG("Diagnosis:chassis velocity differ from localization velocity");
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SPEED_BIT);
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SPEED_LEVEL_BIT);
            global_paras_.check_result_string_vector.push_back("chassis2localization velocity");
        }

        // post process
        if (!global_paras_.check_result_string_vector.empty()) {
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT);
            SetCodeBitHigh(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_LEVEL_BIT);
            SetCodeBitLow(&global_paras_.chassis_check_code, pnc::control::ChassisCheckBit::CHASSIS_UNKNOWN_BIT);
        }
    }

    UpdateCodeQueue(&global_paras_);

    SetDebugInfo(global_paras_, diag_debug_ptr);

    return bflag;
}

pnc::control::DiagnosisReturnCode ChassisCheck::VelocityCheck(const ControlInputStream& control_input_stream,
                                                              SpecificCheckParas* const specific_check_paras_ptr) {
    const pnc::chassis::Chassis* const chassis_ptr = &control_input_stream.input_stream.chassis;
    const pnc::localization::LocalizationEstimate* const localization_ptr =
        &control_input_stream.input_stream.localization;
    if (localization_ptr == nullptr) {
        LOG_DEBUG("Diagnosis:localization_ptr is nullptr");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    if (!localization_ptr->has_header()) {
        LOG_DEBUG("Diagnosis:localization_ptr has no header");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    if (!localization_ptr->has_loc_pose() || !localization_ptr->loc_pose().has_local_pose() ||
        !localization_ptr->loc_pose().local_pose().has_position() ||
        !localization_ptr->loc_pose().local_pose().has_linear_velocity()) {
        LOG_DEBUG("Diagnosis:localization_ptr has no linear velocity");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    double current_chassis_speed = chassis_ptr->speed_mps();
    double current_localization_linear_velocity{0.0};
    if (control_conf_.diag_v2_conf().chassis_check_conf().enable_using_raw_localization_velocity()) {
        const auto& speed3d = localization_ptr->loc_pose().pose().linear_velocity();
        current_localization_linear_velocity = std::hypot(speed3d.x(), speed3d.y());
    } else {
        current_localization_linear_velocity = control_input_stream.vehicle_state.localization_velocity();
    }
    double chassis2localization_velocity_error{0.0};
    chassis2localization_velocity_error = current_chassis_speed - current_localization_linear_velocity;
    specific_check_paras_ptr->chassis_velocity_queue.push(current_chassis_speed);
    specific_check_paras_ptr->chassis2localization_velocity_error_queue.push(
        std::fabs(chassis2localization_velocity_error));
    const int kChassisVelocityCheckDuration =
        control_conf_.diag_v2_conf().chassis_check_conf().chassis_velocity_check_duration();
    const double kChassisVelocityCheckDeadzone =
        control_conf_.diag_v2_conf().chassis_check_conf().chassis_velocity_check_deadzone();

    if (static_cast<int>(specific_check_paras_ptr->chassis_velocity_queue.size()) < kChassisVelocityCheckDuration ||
        static_cast<int>(specific_check_paras_ptr->chassis2localization_velocity_error_queue.size()) <
            kChassisVelocityCheckDuration) {
        LOG_DEBUG("Diagnosis:chassis velocity queue size(%d) < chassis_velocity_check_duration(%d)",
                  static_cast<int>(specific_check_paras_ptr->chassis_velocity_queue.size()),
                  kChassisVelocityCheckDuration);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    if (current_chassis_speed < kChassisVelocityCheckDeadzone) {
        LOG_DEBUG("Diagnosis:chassis velocity(%.4f) < chassis_velocity_check_deadzone(%.4f)", current_chassis_speed,
                  kChassisVelocityCheckDeadzone);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    while (static_cast<int>(specific_check_paras_ptr->chassis_velocity_queue.size()) > kChassisVelocityCheckDuration) {
        specific_check_paras_ptr->chassis_velocity_queue.pop();
    }
    while (static_cast<int>(specific_check_paras_ptr->chassis2localization_velocity_error_queue.size()) >
           kChassisVelocityCheckDuration) {
        specific_check_paras_ptr->chassis2localization_velocity_error_queue.pop();
    }
    double mean_chassis_velocity = GetQueueMean(specific_check_paras_ptr->chassis_velocity_queue);
    double mean_chassis2localization_velocity_error =
        GetQueueMean(specific_check_paras_ptr->chassis2localization_velocity_error_queue);
    if (mean_chassis2localization_velocity_error >
            mean_chassis_velocity *
                control_conf_.diag_v2_conf().chassis_check_conf().chassis_velocity_check_threshold_ratio() ||
        std::fabs(chassis2localization_velocity_error) >
            control_conf_.diag_v2_conf().chassis_check_conf().chassis_velocity_check_threshold()) {
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

void ChassisCheck::SetDebugInfo(const GlobalParas& global_param, pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    if (diag_debug_ptr == nullptr) {
        LOG_WARN("Diagnosis:SetDebugInfo failed, diag_debug_ptr is nullptr");
        return;
    }

    std::string check_string;
    if (global_param.check_result_string_vector.empty()) {
        check_string = "No error";
    } else {
        check_string = global_param.check_result_string_vector[0];
        if (static_cast<int>(global_param.check_result_string_vector.size()) > 1) {
            for (int i = 1; i < static_cast<int>(global_param.check_result_string_vector.size()); ++i) {
                check_string += " & ";
                check_string += global_param.check_result_string_vector[i];
            }
        }
    }

    diag_debug_ptr->set_chassis_check_string(check_string);
    diag_debug_ptr->set_chassis_check_code(global_paras_.chassis_check_code);
}

void ChassisCheck::UpdateCodeQueue(GlobalParas* const global_param) {
    const int kCheckDuration = control_conf_.diag_v2_conf().chassis_check_conf().chassis_continuous_check_duration();
    global_param->chassis_check_code_queue.push(global_param->chassis_check_code);

    while (static_cast<int>(global_param->chassis_check_code_queue.size()) > kCheckDuration) {
        global_param->chassis_check_code_queue.pop();
    }
}

double ChassisCheck::GetQueueMean(const std::queue<double>& queue) {
    // note:queue size should not be 0!
    double sum{0.0};
    std::queue<double> q_tmp = queue;
    int size = static_cast<int>(q_tmp.size());

    while (!q_tmp.empty()) {
        sum += q_tmp.front();
        q_tmp.pop();
    }
    return sum / size;
}

}  // namespace control
}  // namespace jiduauto
