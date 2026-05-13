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

class LatCommandPostProcesser final {
public:
    LatCommandPostProcesser() = default;

    ~LatCommandPostProcesser() = default;

    bool Reset();

    bool Init(const pnc::control::ControlConfig& control_conf);

    bool UpdateCommand(ControlCommandStream* const control_command_stream);

private:
    void LogDebugInfo(const double raw_cmd, const double filtered_cmd,
                      pnc::control::CommandPostProcessDebug* const command_post_process_debug) const;

    pnc::control::LatCommandPostProcessConf lat_cmd_post_process_conf_;

    pnc::filter::DigitalFilter steer_cmd_deg_filter_;
};
}  // namespace control
}  // namespace jiduauto
