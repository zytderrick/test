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

#include "control/src/feature/ext_command_post_processer.h"

#include <algorithm>
#include <tuple>

#include "pnc_common/src/common/pnc_loggerv2.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_utils.h"

namespace jiduauto {
namespace control {

using pnc::control::ControlCommand;

bool ExtCommandPostProcesser::Init(const pnc::control::ControlConfig& control_conf) {
    cmd_post_process_conf_ = control_conf.post_process_conf();
    conf_ = control_conf;
    return true;
}

bool ExtCommandPostProcesser::UpdateCommand(const ControlInputStream& control_input_stream,
                                            ControlCommandStream* const control_command_stream) const {
    pnc::control::ControlCommand* cmd_to_update{&control_command_stream->control_command};

    const double curr_time{pnc::util::TimeUtility::GetCurrentTimesecond()};
    UpdateAutoModeCmd(control_input_stream, curr_time, cmd_to_update);
    UpdateGearCmd(control_input_stream, curr_time, cmd_to_update);
    UpdateDriveOffCmd(control_input_stream, cmd_to_update);
    UpdatePlanningState(control_input_stream, cmd_to_update);

    return true;
}

void ExtCommandPostProcesser::UpdateAutoModeCmd(const ControlInputStream& control_input_stream,
                                                const double curr_time_sec, ControlCommand* const cmd) const {
    if (cmd == nullptr) {
        LLOG_ERROR("cmd is nullptr");
        return;
    }
    const auto cur_state = control_input_stream.vehicle_state;
    const auto cur_cmd = control_input_stream.input_stream.control_command;
    const auto cur_padmsg = control_input_stream.input_stream.pad_info;
    static double prev_mode_time{0.0};

    const pnc::control::DrivingAction last_action{cur_cmd.pad_msg().action()};
    pnc::control::DrivingAction target_action{cur_padmsg.action()};

    constexpr double kMinAutoModeReqLen{0.1};  // switch gap 100ms
    if ((target_action != last_action) && (curr_time_sec - prev_mode_time >= kMinAutoModeReqLen) &&
        (cur_state.chassis_ready_st() == pnc::chassis::CHASSIS_READY)) {
        prev_mode_time = curr_time_sec;
    } else {
        target_action = last_action;
    }
    cmd->mutable_pad_msg()->CopyFrom(cur_padmsg);
    cmd->mutable_pad_msg()->set_action(target_action);

    bool is_automode_confirmed{(target_action == pnc::control::START) ? true : false};
    if (is_automode_confirmed) {
        cmd->set_driving_mode(pnc::chassis::COMPLETE_AUTO_DRIVE);
    } else {
        cmd->set_driving_mode(pnc::chassis::COMPLETE_MANUAL);
    }
}

void ExtCommandPostProcesser::UpdateGearCmd(const ControlInputStream& control_input_stream, const double curr_time_sec,
                                            ControlCommand* const cmd) const {
    if ((conf_.controller_config().lat_init() == pnc::control::ControlConfig::CHASSIS_TEST_CONTROLLER) &&
        (conf_.controller_config().lon_init() == pnc::control::ControlConfig::CHASSIS_TEST_CONTROLLER)) {
        return;
    }
    if (cmd == nullptr) {
        LLOG_ERROR("cmd is nullptr");
        return;
    }
    const auto cur_state = control_input_stream.vehicle_state;
    const auto cur_cmd = control_input_stream.input_stream.control_command;
    const auto cur_traj = control_input_stream.input_stream.planning_info;
    static double prev_gear_time{0.0};

    if (cur_state.driving_mode() != pnc::chassis::COMPLETE_AUTO_DRIVE) {
        cmd->set_gear_location(cur_state.gear());
    } else {
        bool is_cur_stationary{cur_state.standstill()};
        constexpr double kMinGearReqLen{0.2};  // switch gap 200ms

        if ((cur_traj.gear() != cur_cmd.gear_location()) && (curr_time_sec - prev_gear_time >= kMinGearReqLen) &&
            is_cur_stationary) {
            prev_gear_time = curr_time_sec;
            cmd->set_gear_location(cur_traj.gear());
        } else {
            cmd->set_gear_location(cur_cmd.gear_location());
        }

        if (cur_traj.gear() > static_cast<uint8_t>(pnc::chassis::GEAR_PARKING)) {
            cmd->set_gear_location(cur_cmd.gear_location());
        }
    }
}

void ExtCommandPostProcesser::UpdateDriveOffCmd(const ControlInputStream& control_input_stream,
                                                ControlCommand* const cmd) const {
    if (cmd == nullptr) {
        LLOG_ERROR("cmd is nullptr");
        return;
    }
    const auto cur_state = control_input_stream.vehicle_state;
    const auto cur_traj = control_input_stream.input_stream.planning_info;

    bool is_cur_stationary{cur_state.standstill()};

    bool drive_off_intention{(cmd_post_process_conf_.enable_planning_driveoff_cmd()) ? cur_traj.standstill_req()
                                                                                     : false};

    std::tuple<bool, bool> drive_off_req(pnc::vehicle::GenerateDriveOffRequest(
        cur_state.gear(), cmd->gear_location(), cmd->acceleration(), is_cur_stationary, drive_off_intention));

    cmd->set_driveoff_req(std::get<0>(drive_off_req));
    cmd->set_standstill_req(std::get<1>(drive_off_req));
}

void ExtCommandPostProcesser::UpdatePlanningState(const ControlInputStream& control_input_stream,
                                                  ControlCommand* const cmd) const {
    cmd->set_planning_state(control_input_stream.input_stream.planning_info.planning_state());
}

}  // namespace control
}  // namespace jiduauto
