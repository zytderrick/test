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

#include "control/src/feature/lat_command_post_processer.h"

#include <vector>

#include "control/src/math/control_math.h"

namespace jiduauto {
namespace control {

bool LatCommandPostProcesser::Init(const pnc::control::ControlConfig& control_conf) {
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);

    const double ts = control_conf.control_period();

    lat_cmd_post_process_conf_ = control_conf.post_process_conf().lat_cmd_post_process_conf();

    ControlMath::GetLpfCoefficient(ts, lat_cmd_post_process_conf_.steer_cmd_deg_cut_off(), &den, &num);

    steer_cmd_deg_filter_.SetCoefficients(den, num);
    return true;
}

bool LatCommandPostProcesser::UpdateCommand(ControlCommandStream* const control_command_stream) {
    double raw_cmd = control_command_stream->control_command.steering_target();
    double filtered_cmd = raw_cmd;
    if (lat_cmd_post_process_conf_.enable_lat_post_process()) {
        filtered_cmd = steer_cmd_deg_filter_.Filter(raw_cmd);
    }
    control_command_stream->control_command.set_steering_target(filtered_cmd);

    LogDebugInfo(raw_cmd, filtered_cmd, control_command_stream->control_debug.mutable_cmd_post_process_debug());

    return true;
}

void LatCommandPostProcesser::LogDebugInfo(
    const double raw_cmd, const double filtered_cmd,
    pnc::control::CommandPostProcessDebug* const command_post_process_debug) const {
    command_post_process_debug->set_enable_lat_post_process(lat_cmd_post_process_conf_.enable_lat_post_process());
    command_post_process_debug->set_steer_cmd_deg_raw(raw_cmd);
    command_post_process_debug->set_steer_cmd_deg_filtered(filtered_cmd);
}

}  // namespace control
}  // namespace jiduauto
