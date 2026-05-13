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

// Control Diagnosis Module
// Refer to https://wiki.jiduauto.com/display/add/Control+Overall
// Refer to https://wiki.jiduauto.com/pages/viewpage.action?pageId=431645985

/**
 * @file control_diagnosis.h
 * @brief Defines the ControlDiagnosisV2 class.
 */
#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "control/src/common/control_message_stream.h"
#include "control/src/feature/diagnosis/actuator_check.h"
#include "control/src/feature/diagnosis/command_post_process.h"
#include "control/src/feature/diagnosis/control_dependent_node/node_check.h"
#include "pnc_common_vehicle_param.pb.h"
#include "pnc_control_config.pb.h"
#include "pnc_control_diagnosis_conf.pb.h"
#include "pnc_debug_control.pb.h"

/**
 * @namespace jiduauto::control
 */

namespace jiduauto {
namespace control {

class ControlDiagnosisV2 {
    struct GlobalParas {
        bool is_fault_detect_passed{true};
        int count_command_continuous_abnormal{0};  // default: 0
    };

public:
    ControlDiagnosisV2();
    ~ControlDiagnosisV2() = default;

    /**
     * @brief init
     */
    bool Init(const pnc::control::ControlConfig& control_conf);

    void Reset();

    bool GeneralCheck(const InputStream& input_stream, pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

    bool FaultDetect(const ControlInputStream& control_input_stream,
                     pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

    void CommandPostProcessPassed(const ControlInputStream& control_input_stream,
                                  pnc::control::ControlCommand* const control_command_ptr,
                                  pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

private:
    bool SanityCheck(const pnc::control::ControlConfig& control_conf);

    void SetDiagnosisDebugProto(pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

    void SetCheckResult(pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

private:
    // conf
    pnc::control::ControlConfig control_conf_;
    pnc::common::VehicleParam veh_paras_;
    // DiagnosisConf diag_conf_;

    std::unordered_map<int, std::shared_ptr<NodeCheck>> node_list_;
    CommandPostProcess command_post_process_;
    ActuatorCheck actuator_check_;

    // paras

    // struct
    GlobalParas global_paras_;
};

}  // namespace control
}  // namespace jiduauto
