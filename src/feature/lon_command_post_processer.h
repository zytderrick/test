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
#include "pnc_common/src/math/filters/digital_filter.h"
#include "pnc_control_config.pb.h"
#include "pnc_debug_control.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

class LonCommandPostProcesser final {
public:
    LonCommandPostProcesser() = default;

    ~LonCommandPostProcesser() = default;

    bool Reset() const;

    bool Init(const pnc::control::ControlConfig& control_conf);

    bool UpdateCommand(const ControlInputStream& control_input_stream,
                       ControlCommandStream* const control_command_stream) const;

private:
    void LogDebugInfo(const double raw_cmd, const double compensation_cmd,
                      pnc::control::CommandPostProcessDebug* const command_post_process_debug) const;

    pnc::control::LonCommandPostProcessConf lon_cmd_post_process_conf_;

    pnc::filter::DigitalFilter slope_filter_;
};
}  // namespace control
}  // namespace jiduauto
