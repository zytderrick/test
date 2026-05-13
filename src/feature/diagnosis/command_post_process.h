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

/**
 * @file
 * @brief Defines the CommandPostProcess class.
 */
#pragma once

#include <string>
#include <unordered_set>

#include "control/src/common/control_message_stream.h"
#include "pnc_common/src/common/status.h"
#include "pnc_control_config.pb.h"
#include "pnc_debug_control.pb.h"
/**
 * @namespace jiduauto::control
 */

namespace jiduauto {
namespace control {

class CommandPostProcess {
    struct ControlCommandInfo {
        double steer_target{0.0};
        double throttle{0.0};
        double brake{0.0};
        double acceleration{0.0};
    };

    struct GlobalParas {
        ControlCommandInfo last_control_command;
        int count_command_continuous_abnormal{0};  // default: 0
    };

public:
    CommandPostProcess() = default;
    ~CommandPostProcess() = default;

    /**
     * @brief init
     */
    bool Init(const pnc::control::ControlConfig& control_conf);

    void Check(const bool is_fault_detect_passed, const pnc::control::ControlCommand* const control_command_ptr,
               pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

    void React(const ControlInputStream& control_input_stream, pnc::control::ControlCommand* const control_command_ptr,
               pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

    void UpdateLastCommand(const pnc::control::ControlCommand* const control_command_ptr);

private:
    bool SanityCheck(const pnc::control::ControlConfig& control_conf);

    pnc::control::DiagnosisReturnCode CheckContinuousNormal();

    pnc::control::DiagnosisReturnCode CheckCommand(const pnc::control::ControlCommand* const control_command_ptr);

    void SoftBrake(const pnc::common::VehicleState& vehicle_state,
                   pnc::control::ControlCommand* const control_command_ptr);

    void ReactLastLateralCommand(pnc::control::ControlCommand* const control_command_ptr);

    void ReactLastLongitudinalCommand(pnc::control::ControlCommand* const control_command_ptr);

private:
    // conf
    pnc::control::ControlConfig control_conf_;

    // struct
    GlobalParas global_paras_;

    // hold soft brake status
    bool estop_hold_{false};
    std::string estop_hold_reason_{""};
};

}  // namespace control
}  // namespace jiduauto
