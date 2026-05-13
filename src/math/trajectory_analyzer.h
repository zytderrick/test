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
 * @file trajectory_analyzer.h
 * @brief Defines the TrajectoryAnalyzer class.
 */
#pragma once

#include <vector>

#include "pnc_localization_estimate.pb.h"
#include "pnc_planning.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

/**
 * @class TrajectoryAnalyzer
 * @brief process point query and conversion related to trajectory
 */
class TrajectoryAnalyzer {
public:
    TrajectoryAnalyzer() = default;
    ~TrajectoryAnalyzer() = default;

    explicit TrajectoryAnalyzer(const pnc::planning::ADCTrajectory* const trajectory);

    int32_t seq_num() const { return seq_num_; }

    pnc::common::TrajectoryPoint QueryNearestPointByAbsoluteTime(const double t) const;

    pnc::common::TrajectoryPoint QueryNearestPointByRelativeTime(const double t) const;

    pnc::common::TrajectoryPoint QueryNearestPointByPosition(const double x, const double y) const;

    pnc::common::PathPoint QueryMatchedPathPoint(const double x, const double y) const;

    void ToTrajectoryFrame(const double x, const double y, const double theta, const double v,
                           const pnc::common::PathPoint& matched_point, double* const ptr_s, double* const ptr_s_dot,
                           double* const ptr_d, double* const ptr_d_dot) const;

    // Transform the planning trajectory to local frame by a given reference point
    std::vector<pnc::common::TrajectoryPoint> ToVehicleFrame(const pnc::common::TrajectoryPoint& origin_point) const;

    bool TrajectoryTransformToCOM(const double rear_to_com_distance);

    pnc::common::PathPoint ComputeCOMPosition(const double rear_to_com_distance,
                                              const pnc::common::PathPoint& path_point) const;

    const std::vector<pnc::common::TrajectoryPoint>& trajectory_points() const;

    static bool GetAbsoluteTimeLateralAndHeadingError(const pnc::localization::LocalizationEstimate* const localization,
                                                      const pnc::planning::ADCTrajectory* const trajectory_message,
                                                      const double x, const double y, const double forward_time,
                                                      const double min_forward_dis, const double head_offset,
                                                      int32_t* curr_index, double* lateral_error, double* curr_heading,
                                                      double* ref_heading, double* heading_error,
                                                      double* inner_lat_error, double* inner_head_error,
                                                      double* record_x, double* record_y, double* record_head,
                                                      int32_t* target_index);

    static bool GetAbsoluteTimeSpeedAndStationError(const pnc::planning::ADCTrajectory* const trajectory_message,
                                                    const int32_t closest_index, const double current_speed,
                                                    const double forward_time, double* target_speed,
                                                    double* speed_error, double* target_station, double* station_error,
                                                    double* target_acc, double* record_speed, double* record_accel);

    static void GetSL(const pnc::planning::ADCTrajectory* const trajectory, const double& x, const double& y,
                      double* const station, double* const lateral, int32_t* const index);

    static double GetPathRemain(const pnc::planning::ADCTrajectory* const trajectory_message,
                                const double current_station, const double speed_deadzone, const bool full_stop);

private:
    pnc::common::PathPoint FindMinDistancePoint(const pnc::common::PathPoint& p0, const pnc::common::PathPoint& p1,
                                                const double x, const double y) const;

    double PointDistanceSquare(const pnc::common::TrajectoryPoint& point, const double x, const double y) const;

private:
    std::vector<pnc::common::TrajectoryPoint> trajectory_points_;

    double header_time_{0.0};
    int32_t seq_num_{0};
};

}  // namespace control
}  // namespace jiduauto
