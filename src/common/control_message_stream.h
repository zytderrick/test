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

#include "pnc_canbus_chassis.pb.h"
#include "pnc_common_vehicle_state.pb.h"
#include "pnc_control_cmd.pb.h"
#include "pnc_control_pad_msg.pb.h"
#include "pnc_debug_control.pb.h"
#include "pnc_localization_estimate.pb.h"
#include "pnc_planning.pb.h"

namespace jiduauto {
namespace control {

struct InputStream {
    pnc::chassis::Chassis chassis;
    pnc::localization::LocalizationEstimate localization;
    pnc::planning::ADCTrajectory planning_info;
    pnc::control::PadMessage pad_info;
    pnc::control::ControlCommand control_command;
};

struct ControlInputStream {
    InputStream input_stream;
    jiduauto::pnc::common::VehicleState vehicle_state;
    pnc::planning::ADCTrajectory local_frame_trajectory;
    // TODO(all): all common required infomation should be added here
};

struct ControlCommandStream {
    pnc::control::ControlCommand control_command;
    pnc::control::ControlDebug control_debug;
    // TODO(all): all additional output or process should be added here
};

}  // namespace control
}  // namespace jiduauto
