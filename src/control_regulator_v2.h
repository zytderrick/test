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

#include <list>
#include <memory>
#include <mutex>
#include <string>

#include "control/src/common/control_message_stream.h"
#include "control/src/controller/controller_agent.h"
#include "control/src/feature/control_evaluator.h"
#include "control/src/feature/control_message_manager.h"
#include "control/src/feature/control_recorder.h"
#include "control/src/feature/diagnosis/control_diagnosis_v2.h"
#include "control/src/feature/ext_command_post_processer.h"
#include "control/src/feature/lat_command_post_processer.h"
#include "control/src/feature/lon_command_post_processer.h"
#include "control/src/feature/noise_generator.h"
#include "control/src/feature/virtual_sensors.h"
#include "pnc_canbus_chassis.pb.h"
#include "pnc_common_vehicle_param.pb.h"
#include "pnc_common_vehicle_state.pb.h"
#include "pnc_control_cmd.pb.h"
#include "pnc_control_config.pb.h"
#include "pnc_control_pad_msg.pb.h"
#include "pnc_debug_control.pb.h"
#include "pnc_localization_estimate.pb.h"
#include "pnc_planning.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {
using ChassisConstPtr = std::shared_ptr<const pnc::chassis::Chassis>;
using LocalizationConstPtr = std::shared_ptr<const pnc::localization::LocalizationEstimate>;
using TrajectoryConstPtr = std::shared_ptr<const pnc::planning::ADCTrajectory>;
using ControlCommandConstPtr = std::shared_ptr<const pnc::control::ControlCommand>;
using ControlCommandPtr = std::shared_ptr<pnc::control::ControlCommand>;
using ControlDebugPtr = std::shared_ptr<pnc::control::ControlDebug>;
using PadMessageConstPtr = std::shared_ptr<const pnc::control::PadMessage>;

class ControlRegulatorV2 {
public:
    explicit ControlRegulatorV2(const pnc::control::ControlConfig& control_conf);
    ~ControlRegulatorV2() = default;

    bool Init();

    bool IterationRoutine();

    void SaveChassisMessage(const ChassisConstPtr& msg);
    void SaveLocalizationMessage(const LocalizationConstPtr& msg);
    void SavePlanningMessage(const TrajectoryConstPtr& msg);
    void SavePadMessage(const PadMessageConstPtr& msg);
    void SaveControlMessage(const ControlCommandConstPtr& msg);

    ControlCommandPtr GetControlCommand();
    ControlDebugPtr GetControlDebug();

private:
    bool ProduceControlCommand(const ControlInputStream& control_input_steam, ControlCommandStream* const control_cmd);
    bool SetAdditionalInfo(const ControlInputStream& control_input_steam, ControlCommandStream* const control_cmd);
    void PreProcess(ControlInputStream* const control_input_stream, ControlCommandStream* const control_command_stream);
    void PostProcess(const ControlInputStream& control_input_stream,
                     ControlCommandStream* const control_command_stream);
    void AddNoiseToInput(ControlInputStream* const control_input_stream,
                         ControlCommandStream* const control_command_stream);
    void StandStillSteeringProcess(const ControlInputStream& control_input_stream,
                                   ControlCommandStream* const control_command_stream);
    void IndicatorControl(const ControlInputStream& control_input_stream,
                          ControlCommandStream* const control_command_stream);
    void TerminalStateProcess(const ControlInputStream& control_input_stream,
                              ControlCommandStream* const control_command_stream);

    pnc::control::ControlConfig control_conf_;
    ControlCommandStream control_command_stream_;
    pnc::common::VehicleParam vehicle_param_;
    int32_t sequence_id_{0};

    std::unique_ptr<ControlMessageManager> control_message_manager_;
    ControllerAgent controller_agent_;
    ControlDiagnosisV2 control_diagnosis_v2_;
    VirtualSensors virtual_sensors_;
    ControlEvaluator control_evaluator_;
    ControlRecorder control_recorder_;
    LatCommandPostProcesser lat_cmd_post_processer_;
    LonCommandPostProcesser lon_cmd_post_processer_;
    ExtCommandPostProcesser ext_cmd_post_processer_;
    NoiseGenerator noise_generator_;
};

}  // namespace control
}  // namespace jiduauto
