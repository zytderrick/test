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
 * @file control_recorder.h
 * @brief Defines the ControlRecorder class.
 */
#pragma once

#include "control/src/feature/control_message_manager.h"
#include "pnc_control_config.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

class ControlRecorder {
public:
    ControlRecorder() = default;
    ~ControlRecorder() = default;

    void LogDebugFile(const ControlCommandStream& control_command_stream,
                      const pnc::control::ControlConfig& control_conf);
    void LogTrajectoryFile(const ControlInputStream& control_input_stream,
                           const pnc::control::ControlConfig& control_conf);

private:
    bool InitLogFile(const ControlCommandStream& control_command_stream,
                     const pnc::control::ControlConfig& control_conf);
    bool InitTrajectoryFile();
    FILE* control_log_file_{nullptr};     // only for debug log
    FILE* pid_speed_log_file_{nullptr};   // pid speed debug csv
    FILE* trajectory_log_file_{nullptr};  // only for debug log
    std::size_t log_index_{0};            // only for debug log
    std::size_t log_count_{0};
    int trajectory_log_index_{0};
};

}  // namespace control
}  // namespace jiduauto
