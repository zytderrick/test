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

#include "control/src/feature/lon_command_post_processer.h"

#include <vector>

#include "pnc_common/src/common/pnc_loggerv2.h"
#include "pnc_common/src/math/math_utility/math_utils.h"

namespace jiduauto {
namespace control {

bool LonCommandPostProcesser::Init(const pnc::control::ControlConfig& control_conf) {
    lon_cmd_post_process_conf_ = control_conf.post_process_conf().lon_cmd_post_process_conf();
    return true;
}

bool LonCommandPostProcesser::Reset() const {
    std::cout << "Reset" << std::endl;
    return true;
}

bool LonCommandPostProcesser::UpdateCommand(const ControlInputStream& control_input_stream,
                                            ControlCommandStream* const control_command_stream) const {
    const double raw_acc_cmd{control_command_stream->control_command.acceleration()};
    double compensation_cmd{raw_acc_cmd};
    constexpr double kGravity{9.81};
    constexpr double kOne{1.0};
    const double gear_sign{
        (control_command_stream->control_command.gear_location() == pnc::chassis::GEAR_REVERSE) ? -kOne : kOne};

    // compensation by pitch(rad),pitch < 0 is uphill
    if (lon_cmd_post_process_conf_.enable_lon_pitch_compensation()) {
        constexpr double kFlatAngle{180.0};
        if (std::fabs(control_input_stream.vehicle_state.pitch()) >
            (lon_cmd_post_process_conf_.pitch_activate_threshold_deg() / kFlatAngle * M_PI)) {
            compensation_cmd =
                raw_acc_cmd + gear_sign * kGravity * std::sin(-kOne * control_input_stream.vehicle_state.pitch());
        }
    }

    if (lon_cmd_post_process_conf_.enable_lon_acc_limit_mode()) {
        compensation_cmd = pnc::math::Clamp(compensation_cmd, lon_cmd_post_process_conf_.acc_lower_limit(),
                                            lon_cmd_post_process_conf_.acc_upper_limit());
    }

    if (lon_cmd_post_process_conf_.enable_lon_post_process()) {
        control_command_stream->control_command.set_acceleration(compensation_cmd);
    }
    LogDebugInfo(raw_acc_cmd, compensation_cmd, control_command_stream->control_debug.mutable_cmd_post_process_debug());

    return true;
}

void LonCommandPostProcesser::LogDebugInfo(
    const double raw_cmd, const double compensation_cmd,
    pnc::control::CommandPostProcessDebug* const command_post_process_debug) const {
    if (command_post_process_debug == nullptr) {
        LLOG_ERROR("command_post_process_debug is nullptr");
        return;
    }
    command_post_process_debug->set_enable_lon_post_process(lon_cmd_post_process_conf_.enable_lon_post_process());
    command_post_process_debug->set_acc_cmd_raw(raw_cmd);
    command_post_process_debug->set_acc_compensation(compensation_cmd);
}

}  // namespace control
}  // namespace jiduauto
