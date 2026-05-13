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

// Message Manager
// 1) provide reader fcn for control component;
// 2) summary all input message to an united output;
// Refer to https://wiki.jiduauto.com/display/~bolin.zhao/Control+Overall

#pragma once

#include <list>
#include <memory>

#include "control/src/common/control_message_stream.h"
#include "pnc_canbus_chassis.pb.h"
#include "pnc_control_cmd.pb.h"
#include "pnc_control_pad_msg.pb.h"
#include "pnc_debug_control.pb.h"
#include "pnc_localization_estimate.pb.h"
#include "pnc_planning.pb.h"

namespace jiduauto {
namespace control {

using ChassisConstPtr = std::shared_ptr<const pnc::chassis::Chassis>;
using LocalizationConstPtr = std::shared_ptr<const pnc::localization::LocalizationEstimate>;
using TrajectoryConstPtr = std::shared_ptr<const pnc::planning::ADCTrajectory>;
using ControlCommandConstPtr = std::shared_ptr<const pnc::control::ControlCommand>;
using ControlCommandPtr = std::shared_ptr<pnc::control::ControlCommand>;
using ControlDebugPtr = std::shared_ptr<pnc::control::ControlDebug>;
using PadMessageConstPtr = std::shared_ptr<const pnc::control::PadMessage>;

class ControlMessageManager {
public:
    ControlMessageManager() = default;
    explicit ControlMessageManager(double max_control_interval_sec)
        : max_control_interval_sec_(max_control_interval_sec) {}

    const InputStream GetOnboardMessage();

    // Save message for ControlComponent
    void SaveChassisMessage(const ChassisConstPtr& chassis_ptr);
    void SavePlanningMessage(const TrajectoryConstPtr& planning_ptr);
    void SaveLocalizationMessage(const LocalizationConstPtr& localization_ptr);
    void SavePadMessage(const PadMessageConstPtr& pad_message_ptr);
    void SaveControlMessage(const ControlCommandConstPtr& control_command_ptr);

private:
    const bool CheckChassisMessage(const pnc::chassis::Chassis& chassis) const;
    ChassisConstPtr GetLatestChassisMessage();

    const bool CheckPlanningMessage(const pnc::planning::ADCTrajectory& planning) const;
    TrajectoryConstPtr GetLatestPlanningMessage();

    const bool CheckLocalizationMessage(const pnc::localization::LocalizationEstimate& localization) const;
    LocalizationConstPtr GetLatestLocalizationMessage();

    const bool CheckPadMessage(const pnc::control::PadMessage& pad_message) const;
    const pnc::control::PadMessage GetLatestPadMessage();

    const bool CheckControlMessage(const pnc::control::ControlCommand& control_command) const;
    ControlCommandConstPtr GetLatestControlMessage();

    // chassis
    std::mutex chassis_mutex_{};
    std::list<ChassisConstPtr> chassis_list_{};
    // localization
    std::mutex localization_mutex_{};
    std::list<LocalizationConstPtr> localization_list_{};
    // planning
    std::mutex planning_mutex_{};
    std::list<TrajectoryConstPtr> planning_list_{};
    // pad messgae
    std::mutex pad_message_mutex_{};
    std::list<pnc::control::PadMessage> pad_message_list_{};
    // control message
    std::mutex control_command_mutex_{};
    std::list<ControlCommandConstPtr> control_command_list_{};

    double max_control_interval_sec_ = 0.0;
};

}  // namespace control
}  // namespace jiduauto
