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

#include "control/src/control_regulator_v2.h"

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::util::TimeUtility;

ControlRegulatorV2::ControlRegulatorV2(const pnc::control::ControlConfig& control_conf) : control_conf_(control_conf) {}

bool ControlRegulatorV2::Init() {
    LOG_INFO("Control init, starting ...");

    if (!controller_agent_.Init(control_conf_)) {
        LOG_ERROR("Control init controller failed! Stopping...");
        return false;
    }
    if (!virtual_sensors_.Init(control_conf_)) {
        LOG_ERROR("Control init virtual_sensors failed! Stopping...");
        return false;
    }
    if (!control_diagnosis_v2_.Init(control_conf_)) {
        LOG_ERROR("Control init diagnosis_v2 failed! Stopping...");
        return false;
    }
    if (!lat_cmd_post_processer_.Init(control_conf_)) {
        LOG_ERROR("Control init lat_cmd_post_processer failed! Stopping...");
        return false;
    }
    if (!lon_cmd_post_processer_.Init(control_conf_)) {
        LOG_ERROR("Control init lon_cmd_post_processer failed! Stopping...");
        return false;
    }
    if (!ext_cmd_post_processer_.Init(control_conf_)) {
        LOG_ERROR("Control init ext_cmd_post_processer failed! Stopping...");
        return false;
    }

    control_message_manager_ = std::make_unique<ControlMessageManager>(control_conf_.max_control_interval_sec());
    vehicle_param_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();

    LOG_INFO("Control init done!");
    return true;
}

bool ControlRegulatorV2::IterationRoutine() {
    int64_t iteration_start_time = TimeUtility::GetCurrentTimeMillsecond();
    ControlInputStream control_input_stream;
    ControlCommandStream command_stream;

    // Pre- Process (Get information, VituralSensors, Diagnostics for input...)
    control_input_stream.input_stream = control_message_manager_->GetOnboardMessage();

    if (!control_diagnosis_v2_.GeneralCheck(control_input_stream.input_stream,
                                            command_stream.control_debug.mutable_diagnosis_debug_v2())) {
        LOG_ERROR("General Check failed: %s",
                  command_stream.control_debug.mutable_diagnosis_debug_v2()->pre_check_string().c_str());
    }

    int64_t pre_process_start_time = TimeUtility::GetCurrentTimeMillsecond();
    PreProcess(&control_input_stream, &command_stream);

    int64_t pre_process_end_time = TimeUtility::GetCurrentTimeMillsecond();
    command_stream.control_debug.mutable_diagnosis_debug_v2()->mutable_time_cost()->set_pre_process_cost_ms(
        static_cast<int32_t>(pre_process_end_time - pre_process_start_time));

    // Main- Process (Controllers, Diagnostics, etc...)
    if (control_diagnosis_v2_.FaultDetect(control_input_stream,
                                          command_stream.control_debug.mutable_diagnosis_debug_v2())) {
        ProduceControlCommand(control_input_stream, &command_stream);
    }
    int64_t produce_cmd_end_time = TimeUtility::GetCurrentTimeMillsecond();
    command_stream.control_debug.mutable_diagnosis_debug_v2()->mutable_time_cost()->set_produce_cmd_cost_ms(
        static_cast<int32_t>(produce_cmd_end_time - pre_process_end_time));

    // After- Process (Evalution, Diagnostics for command and reaction,etc...)
    const EvaluationInfo evaluation_result = control_evaluator_.UpdateEvaluationDebug(control_input_stream);
    control_evaluator_.EvaluationResultToMessage(evaluation_result, command_stream.control_command.mutable_debug(),
                                                 &command_stream.control_debug);

    SetAdditionalInfo(control_input_stream, &command_stream);

    control_diagnosis_v2_.CommandPostProcessPassed(control_input_stream, &command_stream.control_command,
                                                   command_stream.control_debug.mutable_diagnosis_debug_v2());

    int64_t iteration_end_time = TimeUtility::GetCurrentTimeMillsecond();
    command_stream.control_debug.mutable_diagnosis_debug_v2()->set_control_iteration_cost_ms(
        static_cast<int32_t>(iteration_end_time - iteration_start_time));

    PostProcess(control_input_stream, &command_stream);

    int64_t post_process_end_time = TimeUtility::GetCurrentTimeMillsecond();
    command_stream.control_debug.mutable_diagnosis_debug_v2()->mutable_time_cost()->set_post_process_cost_ms(
        static_cast<int32_t>(post_process_end_time - produce_cmd_end_time));

    control_command_stream_ = command_stream;

    return true;
}

ControlCommandPtr ControlRegulatorV2::GetControlCommand() {
    return std::make_shared<pnc::control::ControlCommand>(control_command_stream_.control_command);
}

ControlDebugPtr ControlRegulatorV2::GetControlDebug() {
    return std::make_shared<pnc::control::ControlDebug>(control_command_stream_.control_debug);
}

void ControlRegulatorV2::SaveChassisMessage(const ChassisConstPtr& msg) {
    control_message_manager_->SaveChassisMessage(msg);
}

void ControlRegulatorV2::SaveLocalizationMessage(const LocalizationConstPtr& msg) {
    control_message_manager_->SaveLocalizationMessage(msg);
}
void ControlRegulatorV2::SavePlanningMessage(const TrajectoryConstPtr& msg) {
    control_message_manager_->SavePlanningMessage(msg);
}

void ControlRegulatorV2::SavePadMessage(const PadMessageConstPtr& msg) {
    control_message_manager_->SavePadMessage(msg);
}

void ControlRegulatorV2::SaveControlMessage(const ControlCommandConstPtr& msg) {
    control_message_manager_->SaveControlMessage(msg);
}

bool ControlRegulatorV2::ProduceControlCommand(const ControlInputStream& control_input_stream,
                                               ControlCommandStream* const control_cmd) {
    if (control_cmd == nullptr) {
        LOG_ERROR("input is nullptr");
        return false;
    }

    if (!controller_agent_.ComputeControlCommand(control_input_stream, control_cmd)) {
        LOG_ERROR("Control main function failed");
        return false;
    }
    // TODO(any): post process handle rest

    return true;
}

bool ControlRegulatorV2::SetAdditionalInfo(const ControlInputStream& control_input_stream,
                                           ControlCommandStream* const control_cmd) {
    control_cmd->control_command.mutable_header()->set_module_name("control");
    control_cmd->control_command.mutable_header()->set_sequence_num(++sequence_id_);
    control_cmd->control_command.mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());

    ext_cmd_post_processer_.UpdateCommand(control_input_stream, control_cmd);

    if (sequence_id_ % 10 == 0 && FLAGS_enable_control_info_print) {
        LOG_INFO("%s", control_cmd->control_command.ShortDebugString().c_str());
    }

    return true;
}

void ControlRegulatorV2::PreProcess(ControlInputStream* const control_input_stream,
                                    ControlCommandStream* const control_command_stream) {
    if (control_input_stream != nullptr && control_conf_.enable_csv_debug()) {
        control_recorder_.LogTrajectoryFile(*control_input_stream, control_conf_);
    }

    AddNoiseToInput(control_input_stream, control_command_stream);

    virtual_sensors_.Update(control_input_stream->input_stream, &control_command_stream->control_debug);

    control_input_stream->vehicle_state.CopyFrom(virtual_sensors_.GetSenseView());
}

void ControlRegulatorV2::PostProcess(const ControlInputStream& control_input_stream,
                                     ControlCommandStream* const control_command_stream) {
    if (control_command_stream->control_debug.diagnosis_debug_v2().has_reaction_code() &&
        control_command_stream->control_debug.diagnosis_debug_v2().reaction_code() ==
            pnc::control::DiagnosisReactCode::DIAGNOSIS_REACT_NONE) {
        StandStillSteeringProcess(control_input_stream, control_command_stream);
    }  // some post process only for normal situation

    IndicatorControl(control_input_stream, control_command_stream);
    TerminalStateProcess(control_input_stream, control_command_stream);

    lat_cmd_post_processer_.UpdateCommand(control_command_stream);
    lon_cmd_post_processer_.UpdateCommand(control_input_stream, control_command_stream);

    if (control_command_stream != nullptr && control_conf_.enable_csv_debug()) {
        control_recorder_.LogDebugFile(*control_command_stream, control_conf_);
    }
}

void ControlRegulatorV2::AddNoiseToInput(ControlInputStream* const control_input_stream,
                                         ControlCommandStream* const control_command_stream) {
    noise_generator_.Init(control_conf_);
    if (FLAGS_enable_input_noise) {
        pnc::localization::LocalizationEstimate* localization_ptr = &control_input_stream->input_stream.localization;
        if (!noise_generator_.AddNoiseToLocalization(localization_ptr)) {
            LOG_WARN("AddNoiseToLocalization failed");
        }
    }
    noise_generator_.SetDebugInfo(&control_command_stream->control_debug);
}

void ControlRegulatorV2::StandStillSteeringProcess(const ControlInputStream& control_input_stream,
                                                   ControlCommandStream* const control_command_stream) {
    if (control_input_stream.vehicle_state.require_standstill_steering() &&
        control_conf_.post_process_conf().enable_standstill_steering()) {
        double smooth_swa_target = pnc::math::Clamp(control_input_stream.vehicle_state.standstill_swa(),
                                                    control_input_stream.vehicle_state.steering_angle() -
                                                        control_conf_.post_process_conf().standstill_swa_gain_limit(),
                                                    control_input_stream.vehicle_state.steering_angle() +
                                                        control_conf_.post_process_conf().standstill_swa_gain_limit());

        control_command_stream->control_command.set_steering_target(smooth_swa_target);
        control_command_stream->control_command.set_acceleration(control_conf_.post_process_conf().hold_brake_acc());
    }
}

void ControlRegulatorV2::IndicatorControl(const ControlInputStream& control_input_stream,
                                          ControlCommandStream* const control_command_stream) {
    if (control_command_stream->control_debug.diagnosis_debug_v2().has_reaction_code() &&
        control_command_stream->control_debug.diagnosis_debug_v2().reaction_code() !=
            pnc::control::DiagnosisReactCode::DIAGNOSIS_REACT_NONE) {
        control_command_stream->control_command.mutable_signal()->set_emergency_light(true);
    }
}

void ControlRegulatorV2::TerminalStateProcess(const ControlInputStream& control_input_stream,
                                              ControlCommandStream* const control_command_stream) {
    if (control_input_stream.vehicle_state.require_terminal_process() &&
        control_conf_.post_process_conf().enable_terminal_status_reaction()) {
        double terimanl_steering_angle = control_input_stream.vehicle_state.control_terminal_swa();
        control_command_stream->control_command.set_steering_target(terimanl_steering_angle);
        control_command_stream->control_command.set_acceleration(control_conf_.post_process_conf().hold_brake_acc());
        control_command_stream->control_debug.mutable_cmd_post_process_debug()->set_terminal_status_active(true);
    }
}

}  // namespace control
}  // namespace jiduauto
