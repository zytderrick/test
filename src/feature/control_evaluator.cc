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

#include "control/src/feature/control_evaluator.h"

#include <algorithm>

#include "control/src/common/control_utility.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::math::NormalizeAngle;

const EvaluationInfo ControlEvaluator::UpdateEvaluationDebug(const pnc::planning::ADCTrajectory& trajectory,
                                                             const pnc::common::VehicleState& vehicle_state) {
    trajectory_analyzer_ = TrajectoryAnalyzer(&trajectory);
    trajectory_info_ = trajectory;
    vehicle_state_ = vehicle_state;

    EvaluationInfo eva_info;

    eva_info.lon_debug = UpdateLongitudinalDebug();
    eva_info.lat_debug = UpdateLateralDebug();

    return eva_info;
}

const EvaluationInfo ControlEvaluator::UpdateEvaluationDebug(const ControlInputStream& control_input_stream) {
    trajectory_analyzer_ = TrajectoryAnalyzer(&control_input_stream.input_stream.planning_info);
    trajectory_info_ = control_input_stream.input_stream.planning_info;
    vehicle_state_ = control_input_stream.vehicle_state;

    EvaluationInfo eva_info;

    eva_info.lon_debug = UpdateLongitudinalDebug();
    eva_info.lat_debug = UpdateLateralDebug();

    return eva_info;
}

const pnc::control::LongitudinalDebug ControlEvaluator::UpdateLongitudinalDebug() {
    pnc::control::LongitudinalDebug lon_debug;

    double station = 0.0;
    double lateral = 0.0;
    int current_index = 0;
    trajectory_analyzer_.GetSL(&trajectory_info_, vehicle_state_.x(), vehicle_state_.y(), &station, &lateral,
                               &current_index);

    double target_speed = 0.0;
    double speed_error = 0.0;
    double target_station = 0.0;
    double station_error = 0.0;
    double target_acc = 0.0;
    double record_speed = 0.0;
    double record_acc = 0.0;

    trajectory_analyzer_.GetAbsoluteTimeSpeedAndStationError(
        &trajectory_info_, current_index, vehicle_state_.linear_velocity(), forsee_time_, &target_speed, &speed_error,
        &target_station, &station_error, &target_acc, &record_speed, &record_acc);

    lon_debug.set_station_error(station_error);  // target - self: positive is behind
    lon_debug.set_speed_error(speed_error);
    lon_debug.set_station_target(target_station);
    lon_debug.set_speed_target(target_speed);
    lon_debug.set_current_speed(vehicle_state_.linear_velocity());

    const double kLogicStationErrorLimit = 2.0;
    if (std::abs(station_error) < kLogicStationErrorLimit) {
        if (station_error < 0.0) {
            max_ahead_station_error_ = std::min(max_ahead_station_error_, station_error);
        } else {
            max_behind_station_error_ = std::max(max_behind_station_error_, station_error);
        }
    }
    lon_debug.set_hist_max_ahead_station_error(max_ahead_station_error_);
    lon_debug.set_hist_max_behind_station_error(max_behind_station_error_);
    // lon_debug.a_err =  reference_point.a() - acc_obs; TODO(zhaobolin): revist this by vehicle state

    return lon_debug;
}

const pnc::control::LateralDebug ControlEvaluator::UpdateLateralDebug() const {
    pnc::control::LateralDebug lat_debug;
    const pnc::common::TrajectoryPoint matched_point =
        trajectory_analyzer_.QueryNearestPointByPosition(vehicle_state_.x(), vehicle_state_.y());

    const double dx = vehicle_state_.x() - matched_point.path_point().x();
    const double dy = vehicle_state_.y() - matched_point.path_point().y();
    const double ref_heading = matched_point.path_point().theta();
    const double cos_matched_theta = std::cos(ref_heading);
    const double sin_matched_theta = std::sin(ref_heading);

    lat_debug.set_lateral_error_by_s(cos_matched_theta * dy - sin_matched_theta * dx);

    const double delta_theta = NormalizeAngle(vehicle_state_.heading() - ref_heading);
    lat_debug.set_heading_error_by_s(delta_theta);
    lat_debug.set_ref_kappa(matched_point.path_point().kappa());

    return lat_debug;
}

// This functuin Convert EvaluationInfo to apolloosproto and control debug message
void ControlEvaluator::EvaluationResultToMessage(const EvaluationInfo& eva_info,
                                                 pnc::control::Debug* const cmd_debug_ptr,
                                                 pnc::control::ControlDebug* const debug_ptr) {
    if (cmd_debug_ptr != nullptr) {
        cmd_debug_ptr->mutable_urban_lat_debug()->set_heading_error(eva_info.lat_debug.heading_error_by_s());
        cmd_debug_ptr->mutable_urban_lat_debug()->set_lateral_error(eva_info.lat_debug.lateral_error_by_s());

        cmd_debug_ptr->mutable_urban_lon_debug()->set_speed_error(eva_info.lon_debug.speed_error());
        cmd_debug_ptr->mutable_urban_lon_debug()->set_station_error(eva_info.lon_debug.station_error());
    }
    if (debug_ptr != nullptr) {
        debug_ptr->mutable_lateral_debug()->CopyFrom(eva_info.lat_debug);
        debug_ptr->mutable_longitudinal_debug()->CopyFrom(eva_info.lon_debug);
    }
}

}  // namespace control
}  // namespace jiduauto
