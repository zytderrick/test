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

// Performance Evaluation Module
// Refer to https://wiki.jiduauto.com/display/~bolin.zhao/Control+Overall

#pragma once

#include "control/src/common/control_message_stream.h"
#include "control/src/math/trajectory_analyzer.h"
#include "pnc_control_cmd.pb.h"
#include "pnc_debug_control.pb.h"

namespace jiduauto {
namespace control {

struct EvaluationInfo {
    pnc::control::LongitudinalDebug lon_debug;
    pnc::control::LateralDebug lat_debug;
};

class ControlEvaluator {
public:
    ControlEvaluator() = default;

    const EvaluationInfo UpdateEvaluationDebug(const pnc::planning::ADCTrajectory& trajectory,
                                               const jiduauto::pnc::common::VehicleState& vehicle_state);

    const EvaluationInfo UpdateEvaluationDebug(const ControlInputStream& control_input_stream);

    void EvaluationResultToMessage(const EvaluationInfo& eva_info, pnc::control::Debug* const cmd_debug_ptr,
                                   pnc::control::ControlDebug* const debug_ptr);

private:
    const pnc::control::LongitudinalDebug UpdateLongitudinalDebug();
    const pnc::control::LateralDebug UpdateLateralDebug() const;

    TrajectoryAnalyzer trajectory_analyzer_;
    pnc::planning::ADCTrajectory trajectory_info_;
    jiduauto::pnc::common::VehicleState vehicle_state_;

    // TODO(bolin.zhao): revisit this conf
    double forsee_time_ = 0.0;
    double max_ahead_station_error_ = 0.0;
    double max_behind_station_error_ = 0.0;
};

}  // namespace control
}  // namespace jiduauto
