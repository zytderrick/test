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

#pragma once

#include "control/src/feature/control_message_manager.h"
#include "pnc_command_post_process_conf.pb.h"
#include "pnc_control_config.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

class ExtCommandPostProcesser final {
public:
    ExtCommandPostProcesser() = default;

    ~ExtCommandPostProcesser() = default;

    bool Init(const pnc::control::ControlConfig& control_conf);

    bool UpdateCommand(const ControlInputStream& control_input_stream,
                       ControlCommandStream* const control_command_stream) const;

private:
    // todo(someone): minimize the io
    void UpdateAutoModeCmd(const ControlInputStream& control_input_stream, const double curr_time_sec,
                           pnc::control::ControlCommand* const cmd) const;
    void UpdateGearCmd(const ControlInputStream& control_input_stream, const double curr_time_sec,
                       pnc::control::ControlCommand* const cmd) const;
    void UpdateDriveOffCmd(const ControlInputStream& control_input_stream,
                           pnc::control::ControlCommand* const cmd) const;
    void UpdatePlanningState(const ControlInputStream& control_input_stream,
                             pnc::control::ControlCommand* const cmd) const;

private:
    pnc::control::CommandPostProcessConf cmd_post_process_conf_;
    pnc::control::ControlConfig conf_;
};
}  // namespace control
}  // namespace jiduauto
