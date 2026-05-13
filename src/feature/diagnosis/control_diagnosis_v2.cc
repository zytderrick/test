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
#include "control/src/feature/diagnosis/control_diagnosis_v2.h"

#include <string>
#include <unordered_set>

#include "control/src/common/control_gflags.h"
#include "control/src/feature/diagnosis/control_dependent_node/chassis_check.h"
#include "control/src/feature/diagnosis/control_dependent_node/localization_check.h"
#include "control/src/feature/diagnosis/control_dependent_node/trajectory_check.h"
#include "pnc_common/src/common/pnc_logger.h"

namespace jiduauto {
namespace control {

ControlDiagnosisV2::ControlDiagnosisV2() {
    node_list_[pnc::control::DiagnosisV2Conf::CHASSIS] = std::make_shared<ChassisCheck>();
    node_list_[pnc::control::DiagnosisV2Conf::LOCALIZATION] = std::make_shared<LocalizationCheck>();
    node_list_[pnc::control::DiagnosisV2Conf::TRAJECTORY] = std::make_shared<TrajectoryCheck>();
}

// public
bool ControlDiagnosisV2::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("Diagnosis: Function Init, starting ...");

    if (!SanityCheck(control_conf)) {
        LOG_ERROR("Diagnosis: SanityCheck failed");
        return false;
    }

    control_conf_ = control_conf;

    for (auto& node : node_list_) {
        if (node.second == nullptr || !node.second->Init(control_conf)) {
            LOG_ERROR("Diagnosis: Node < %s > is nullptr or init failed", node.second->Name().c_str());
            return false;
        } else {
            LOG_INFO("Diagnosis: Node < %s > init done", node.second->Name().c_str());
        }
    }

    if (!actuator_check_.Init(control_conf)) {
        LOG_ERROR("Diagnosis: actuator_check init failed");
        return false;
    } else {
        LOG_INFO("Diagnosis: actuator_check init done");
    }

    if (!command_post_process_.Init(control_conf)) {
        LOG_ERROR("Diagnosis: command_post_process init failed");
        return false;
    } else {
        LOG_INFO("Diagnosis: command_post_process init done");
    }

    if (control_conf_.diag_v2_conf().has_enable_actuator_check()) {
        LOG_INFO("enable_actuator_check: %s", control_conf_.diag_v2_conf().enable_actuator_check() ? "True" : "False");
    }

    return true;
}

void ControlDiagnosisV2::Reset() {
    for (auto& node : node_list_) {
        if (node.second != nullptr) {
            node.second->Reset();
            LOG_INFO("Diagnosis: Node < %s > Reset done", node.second->Name().c_str());
        } else {
            LOG_INFO("Diagnosis: Node < %s > Reset failed", node.second->Name().c_str());
        }
    }

    actuator_check_.Reset();
}

bool ControlDiagnosisV2::SanityCheck(const pnc::control::ControlConfig& control_conf) {
    // Note: Please synchronize the new parameters to ./path_to_unit_test/data/control_conf.pb.txt

    if (!control_conf.has_diag_v2_conf()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: diag_v2_conf");
        return false;
    }

    const pnc::control::DiagnosisV2Conf diag_v2_conf = control_conf.diag_v2_conf();
    if (!diag_v2_conf.has_enable_actuator_check()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: enable_actuator_check");
        return false;
    }

    return true;
}

bool ControlDiagnosisV2::GeneralCheck(const InputStream& input_stream,
                                      pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    const auto& config = control_conf_.controller_config();
    if ((config.lat_init() == pnc::control::ControlConfig::CHASSIS_TEST_CONTROLLER &&
         config.lon_init() == pnc::control::ControlConfig::CHASSIS_TEST_CONTROLLER)) {
        return true;
    }
    bool bflag = true;
    for (auto& node : node_list_) {
        if (!node.second->GeneralCheck(input_stream, diag_debug_ptr)) {
            if (FLAGS_enable_control_info_print) {
                LOG_ERROR("Diagnosis: Node <%s> GeneralCheck failed", node.second->Name().c_str());
            }
            bflag = false;
        }
    }

    // set pre_check_string
    SetCheckResult(diag_debug_ptr);

    return bflag;
}

bool ControlDiagnosisV2::FaultDetect(const ControlInputStream& control_input_stream,
                                     pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    const auto& config = control_conf_.controller_config();
    if ((config.lat_init() == pnc::control::ControlConfig::CHASSIS_TEST_CONTROLLER &&
         config.lon_init() == pnc::control::ControlConfig::CHASSIS_TEST_CONTROLLER)) {
        return true;
    }
    bool bflag = true;

    // SpecificCheck
    std::string specific_check_string = "Diagnosis: SpecificCheck failed! ";
    bool bflag_specific_check = true;
    for (auto& node : node_list_) {
        if (!node.second->SpecificCheck(control_input_stream, diag_debug_ptr)) {
            std::string node_check_string{""};
            if (node.first == pnc::control::DiagnosisV2Conf::CHASSIS) {
                node_check_string = "Chassis: " + diag_debug_ptr->chassis_check_string();
            } else if (node.first == pnc::control::DiagnosisV2Conf::LOCALIZATION) {
                node_check_string = "Localization: " + diag_debug_ptr->localization_check_string();
            } else if (node.first == pnc::control::DiagnosisV2Conf::TRAJECTORY) {
                node_check_string = "Trajectory: " + diag_debug_ptr->trajectory_check_string();
            }
            specific_check_string += node_check_string + ". ";
            if (FLAGS_enable_control_info_print) {
                LOG_ERROR("Diagnosis: Node <%s> SpecificCheck failed: %s", node.second->Name().c_str(),
                          node_check_string.c_str());
            }
            bflag_specific_check = false;
            bflag = false;
        }
    }
    if (!bflag_specific_check) {
        LOG_ERROR(specific_check_string.c_str());
    }

    // set debug proto
    SetDiagnosisDebugProto(diag_debug_ptr);

    // ActuatorCheck
    if (!actuator_check_.FaultCheck(control_input_stream, diag_debug_ptr)) {
        if (!control_conf_.has_diag_v2_conf() || !control_conf_.diag_v2_conf().has_enable_actuator_check()) {
            LOG_ERROR("Diagnosis: Fail to check control_conf_: diag_v2_conf or enable_actuator_check");
            return false;
        }
        std::string actuator_check_string = diag_debug_ptr->actuator_check_string();
        LOG_WARN("Diagnosis: ActuatorCheckPassed failed, actuator_check_string: %s", actuator_check_string.c_str());
        if (control_conf_.diag_v2_conf().enable_actuator_check()) {
            bflag = false;
        }
    }

    // set pre_check_string
    SetCheckResult(diag_debug_ptr);

    global_paras_.is_fault_detect_passed = bflag;
    return bflag;
}

void ControlDiagnosisV2::SetDiagnosisDebugProto(pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    if (diag_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: diag_debug_ptr is nullptr");
        return;
    }

    // set switch
    if (control_conf_.has_diag_v2_conf() && control_conf_.diag_v2_conf().has_enable_actuator_check()) {
        diag_debug_ptr->mutable_diagnosis_switch()->set_enable_actuator_check(
            control_conf_.diag_v2_conf().enable_actuator_check());
    }

    if (control_conf_.has_diag_v2_conf() && control_conf_.diag_v2_conf().has_enable_localization_jump_check()) {
        diag_debug_ptr->mutable_diagnosis_switch()->set_enable_localization_jump_check(
            control_conf_.diag_v2_conf().enable_localization_jump_check());
    }

    if (control_conf_.has_diag_v2_conf() && control_conf_.diag_v2_conf().has_enable_localization_continuous_check()) {
        diag_debug_ptr->mutable_diagnosis_switch()->set_enable_localization_continuous_check(
            control_conf_.diag_v2_conf().enable_localization_continuous_check());
    }

    if (control_conf_.has_diag_v2_conf() && control_conf_.diag_v2_conf().has_enable_curvature_check()) {
        diag_debug_ptr->mutable_diagnosis_switch()->set_enable_curvature_check(
            control_conf_.diag_v2_conf().enable_curvature_check());
    }
}

void ControlDiagnosisV2::CommandPostProcessPassed(const ControlInputStream& control_input_stream,
                                                  pnc::control::ControlCommand* const control_command_ptr,
                                                  pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    command_post_process_.Check(global_paras_.is_fault_detect_passed, control_command_ptr, diag_debug_ptr);
    command_post_process_.React(control_input_stream, control_command_ptr, diag_debug_ptr);
    command_post_process_.UpdateLastCommand(control_command_ptr);
}

void ControlDiagnosisV2::SetCheckResult(pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    int localization_check_code = diag_debug_ptr->localization_check_code();
    int chassis_check_code = diag_debug_ptr->chassis_check_code();
    int trajectory_check_code = diag_debug_ptr->trajectory_check_code();
    std::vector<std::string> pre_check_string_vec;
    bool is_localization_abnormal =
        ((localization_check_code &
          (0x00000001 << (pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_BIT - 1))) >>
         (pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_BIT - 1)) == 0x00000001;
    if (is_localization_abnormal) {
        pre_check_string_vec.push_back("localization");
    }
    bool is_chassis_abnormal =
        ((chassis_check_code & (0x00000001 << (pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT - 1))) >>
         (pnc::control::ChassisCheckBit::CHASSIS_SUMMARY_BIT - 1)) == 0x00000001;
    if (is_chassis_abnormal) {
        pre_check_string_vec.push_back("chassis");
    }
    bool is_trajectory_abnormal =
        ((trajectory_check_code & (0x00000001 << (pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_BIT - 1))) >>
         (pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_BIT - 1)) == 0x00000001;
    if (is_trajectory_abnormal) {
        pre_check_string_vec.push_back("trajectory");
    }
    std::string string_tmp = "no error";
    if (!pre_check_string_vec.empty()) {
        string_tmp = pre_check_string_vec[0];
        if (static_cast<int>(pre_check_string_vec.size()) > 1) {
            for (int i = 1; i < static_cast<int>(pre_check_string_vec.size()); ++i) {
                string_tmp += " & ";
                string_tmp += pre_check_string_vec[i];
            }
        }
    }
    diag_debug_ptr->set_pre_check_string(string_tmp);
}

}  // namespace control
}  // namespace jiduauto
