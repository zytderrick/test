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
 * @brief Defines the Controller base class.
 */

#pragma once

#include <string>

#include "control/src/common/control_message_stream.h"
#include "pnc_canbus_chassis.pb.h"
#include "pnc_control_cmd.pb.h"
#include "pnc_control_config.pb.h"
#include "pnc_localization_estimate.pb.h"
#include "pnc_planning.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

/**
 * @class Controller
 * @brief base class for all controllers.
 */
class Controller {
public:
    Controller() = default;
    virtual ~Controller() = default;

    virtual bool Init(const pnc::control::ControlConfig& control_conf) = 0;

    virtual bool Control(const pnc::localization::LocalizationEstimate* const localization,
                         const pnc::chassis::Chassis* const chassis,
                         const pnc::planning::ADCTrajectory* const trajectory,
                         pnc::control::ControlCommand* const cmd) = 0;

    virtual bool ControlMain(const ControlInputStream& control_input_stream, ControlCommandStream* const cmd) = 0;

    /**
     * @brief reset Controller
     * @return true if succeeded; otherwise false;
     */
    virtual bool Reset() = 0;

    /**
     * @brief controller name
     * @return string controller name in string
     */
    virtual std::string Name() const = 0;

    /**
     * @brief stop controller
     */
    virtual void Stop() = 0;
};

}  // namespace control
}  // namespace jiduauto
