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

#include "control/src/math/trajectory_analyzer.h"

#include <algorithm>
#include <limits>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/math/control_math.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/common/interpolation.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/time_utility.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::math::NormalizeAngle;
using jiduauto::pnc::util::TimeUtility;
using pnc::common::TrajectoryPoint;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

TrajectoryAnalyzer::TrajectoryAnalyzer(const ADCTrajectory* const trajectory) {
    if (trajectory == nullptr) return;
    header_time_ = trajectory->header().timestamp_sec();
    seq_num_ = trajectory->header().sequence_num();
    trajectory_points_.clear();
    for (auto& pt : trajectory->trajectory_point()) {
        trajectory_points_.push_back(pt);
    }
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(const double t) const {
    return QueryNearestPointByRelativeTime(t - header_time_);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(const double t) const {
    auto func_comp = [](const TrajectoryPoint& point, const double relative_time) {
        return point.relative_time() < relative_time;
    };

    auto it_low = std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(), t, func_comp);

    if (it_low == trajectory_points_.begin()) {
        return trajectory_points_.front();
    }

    if (it_low == trajectory_points_.end()) {
        return trajectory_points_.back();
    }

    if (FLAGS_query_forward_time_point_only) {
        return *it_low;
    } else {
        auto it_lower = it_low - 1;
        if (it_low->relative_time() - t < t - it_lower->relative_time()) {
            return *it_low;
        }
        return *it_lower;
    }
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPosition(const double x, const double y) const {
    if (trajectory_points_.empty()) {
        return TrajectoryPoint();
    }

    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return trajectory_points_[index_min];
}

pnc::common::PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x, const double y) const {
    if (trajectory_points_.empty()) {
        return pnc::common::PathPoint();
    }

    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }

    size_t index_start = index_min == 0 ? index_min : index_min - 1;
    size_t index_end = index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

    const double kEpsilon = 0.001;
    if (index_start == index_end || std::fabs(trajectory_points_[index_start].path_point().s() -
                                              trajectory_points_[index_end].path_point().s()) <= kEpsilon) {
        return trajectory_points_[index_start].path_point();
    }

    return FindMinDistancePoint(trajectory_points_[index_start].path_point(),
                                trajectory_points_[index_end].path_point(), x, y);
}

// reference: Optimal trajectory generation for dynamic street scenarios in a
// Frenet Frame,
// Moritz Werling, Julius Ziegler, Soren Kammel and Sebastian Thrun, ICRA 2010
// similar to the method in this paper without the assumption the "normal"
// vector
// (from vehicle position to ref_point position) and reference heading are
// perpendicular.
void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y, const double theta, const double v,
                                           const pnc::common::PathPoint& ref_point, double* const ptr_s,
                                           double* const ptr_s_dot, double* const ptr_d,
                                           double* const ptr_d_dot) const {
    if (ptr_s == nullptr || ptr_s_dot == nullptr || ptr_d == nullptr || ptr_d_dot == nullptr) {
        return;
    }
    double dx = x - ref_point.x();
    double dy = y - ref_point.y();

    double cos_ref_theta = std::cos(ref_point.theta());
    double sin_ref_theta = std::sin(ref_point.theta());

    // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
    // (dx, dy)
    double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
    *ptr_d = cross_rd_nd;

    // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
    // (dx, dy)
    double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
    *ptr_s = ref_point.s() + dot_rd_nd;

    double delta_theta = theta - ref_point.theta();
    double cos_delta_theta = std::cos(delta_theta);
    double sin_delta_theta = std::sin(delta_theta);

    *ptr_d_dot = v * sin_delta_theta;

    double one_minus_kappa_r_d = 1 - ref_point.kappa() * (*ptr_d);
    if (one_minus_kappa_r_d <= 0.0) {
        LOG_ERROR(
            "TrajectoryAnalyzer::ToTrajectoryFrame found fatal reference and "
            "actual difference. Control output might be unstable: "
            "ref_point.curvature: "
            "%.4f, ref_point.x: %.4f, "
            "ref_point.y: %.4f, car x: %.4f, car y: %.4f, *ptr_d: %.4f, "
            "one_minus_curvature_r_d: %.4f ",
            ref_point.kappa(), ref_point.x(), ref_point.y(), x, y, *ptr_d, one_minus_kappa_r_d);
        // currently set to a small value to avoid control crash.
        one_minus_kappa_r_d = 0.01;
    }

    *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

bool TrajectoryAnalyzer::TrajectoryTransformToCOM(const double rear_to_com_distance) {
    if (trajectory_points_.empty()) return false;
    for (size_t i = 0; i < trajectory_points_.size(); ++i) {
        const pnc::common::PathPoint com = ComputeCOMPosition(rear_to_com_distance, trajectory_points_[i].path_point());
        trajectory_points_[i].mutable_path_point()->set_x(com.x());
        trajectory_points_[i].mutable_path_point()->set_y(com.y());
    }
    return true;
}

pnc::common::PathPoint TrajectoryAnalyzer::ComputeCOMPosition(const double rear_to_com_distance,
                                                              const pnc::common::PathPoint& path_point) const {
    // Initialize the vector for coordinate transformation of the position
    // reference point
    Eigen::Vector3d v;
    const double cos_heading = std::cos(path_point.theta());
    const double sin_heading = std::sin(path_point.theta());
    v << rear_to_com_distance * cos_heading, rear_to_com_distance * sin_heading, 0.0;
    // Original position reference point at center of rear-axis
    Eigen::Vector3d pos_vec(path_point.x(), path_point.y(), path_point.z());
    // Transform original position with vector v
    Eigen::Vector3d com_pos_3d = v + pos_vec;
    // Return transfromed x and y
    pnc::common::PathPoint tmp = path_point;
    tmp.set_x(com_pos_3d[0]);
    tmp.set_y(com_pos_3d[1]);
    return tmp;
}

const std::vector<TrajectoryPoint>& TrajectoryAnalyzer::trajectory_points() const { return trajectory_points_; }

bool TrajectoryAnalyzer::GetAbsoluteTimeLateralAndHeadingError(
    const LocalizationEstimate* const localization, const ADCTrajectory* const trajectory_message, const double x,
    const double y, const double forward_time, const double min_forward_dis, const double head_offset,
    int32_t* curr_index, double* lateral_error, double* curr_heading, double* ref_heading, double* heading_error,
    double* inner_lat_error, double* inner_head_error, double* record_x, double* record_y, double* record_heading,
    int32_t* target_index) {
    int32_t curr_index_result = 0;
    if (trajectory_message == nullptr) {
        LOG_ERROR("trajectory_message is nullptr!");
        return false;
    }
    if (trajectory_message->trajectory_point_size() < 1) {
        LOG_ERROR("pnc_trajectory_points_size < 1");
        return false;
    }
    // step 1: find the closest point along path trajectory
    double closet_dist = std::numeric_limits<double>::max();
    for (int32_t i = 0; i < trajectory_message->trajectory_point_size(); ++i) {
        double curr_dist = std::hypot(trajectory_message->trajectory_point(i).path_point().x() - x,
                                      trajectory_message->trajectory_point(i).path_point().y() - y);
        if (curr_dist < closet_dist) {
            closet_dist = curr_dist;
            curr_index_result = i;
        }
        // reduce complexity
        if (curr_dist <= 0.16) {  // TODO(zhaobolin): (review) magic num
            break;
        }
    }

    // use time
    const double curr_time = TimeUtility::GetCurrentTimesecond();
    const double planning_time = trajectory_message->header().timestamp_sec();
    double delta_time = curr_time - planning_time;
    if (delta_time < 0.0) {
        LOG_WARN("delta_time[%.4f] less than 0, use closest pt instead", delta_time);
        delta_time = trajectory_message->trajectory_point(curr_index_result).relative_time();
    }
    delta_time = std::max(delta_time, trajectory_message->trajectory_point(curr_index_result).relative_time());

    int32_t index_target = -1;
    for (int32_t i = 0; i < trajectory_message->trajectory_point_size(); ++i) {
        double forward_dis = trajectory_message->trajectory_point(i).path_point().s() -
                             trajectory_message->trajectory_point(curr_index_result).path_point().s();
        // note: the judge below may not perform as expected, for example:
        // trajectory_message->pnc_trajectory_points(i).relative_time() = 1.2,
        // delta_time = 0.1, forward_time = 0.1; int math,
        // trajectory_message->pnc_trajectory_points(i).relative_time() - delta_time
        // == forward_time however, in our programe, the sentence in our judge may
        // not be executed, for the storation of the double type of data this
        // situation may be improved by doing this,
        // trajectory_message->pnc_trajectory_points(i).relative_time() - delta_time
        // >= (forward_time - 1e-6) here, we keep this detail thing till we focus on
        // the control module
        if (trajectory_message->trajectory_point(i).relative_time() - delta_time >= forward_time &&
            forward_dis >= min_forward_dis) {
            index_target = i;
            break;
        }
    }
    if (index_target < 0) {
        LOG_ERROR("failed to find target point with time: %.4f", delta_time + forward_time);
        return false;
    }

    int32_t target_index_result = index_target;
    // get error
    double point_angle = std::atan2(y - trajectory_message->trajectory_point(index_target).path_point().y(),
                                    x - trajectory_message->trajectory_point(index_target).path_point().x());
    double point2path_angle = point_angle - trajectory_message->trajectory_point(index_target).path_point().theta();
    double current_dist = std::hypot(trajectory_message->trajectory_point(index_target).path_point().x() - x,
                                     trajectory_message->trajectory_point(index_target).path_point().y() - y);
    double lateral_error_result = std::sin(point2path_angle) * current_dist;

    double ref_heading_result = trajectory_message->trajectory_point(index_target).path_point().theta();
    auto& orientation = localization->loc_pose().pose().orientation();
    EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());

    double curr_heading_result = q.yaw() + head_offset;  //+ M_PI * 0.5
    double heading_error_result = NormalizeAngle(curr_heading_result - ref_heading_result);

    // inner error
    point_angle = std::atan2(y - trajectory_message->trajectory_point(curr_index_result).path_point().y(),
                             x - trajectory_message->trajectory_point(curr_index_result).path_point().x());
    point2path_angle = point_angle - trajectory_message->trajectory_point(curr_index_result).path_point().theta();
    current_dist = std::hypot(trajectory_message->trajectory_point(curr_index_result).path_point().x() - x,
                              trajectory_message->trajectory_point(curr_index_result).path_point().y() - y);
    // used for evaluation
    double inner_lat_error_result = std::sin(point2path_angle) * current_dist;
    double inner_head_error_result = NormalizeAngle(
        curr_heading_result - trajectory_message->trajectory_point(curr_index_result).path_point().theta());

    // record x,y,head
    index_target = -1;
    for (int32_t i = 0; i < trajectory_message->trajectory_point_size(); ++i) {
        if (trajectory_message->trajectory_point(i).relative_time() - delta_time >= 0.1) {
            index_target = i;
            break;
        }
    }

    double record_x_result = trajectory_message->trajectory_point(index_target).path_point().x();
    double record_y_result = trajectory_message->trajectory_point(index_target).path_point().y();
    double record_heading_result = trajectory_message->trajectory_point(index_target).path_point().theta();

    if (FLAGS_enable_control_info_print) {
        LOG_INFO("lat error: %.4f, head error: %.4f", lateral_error_result, heading_error_result);
        LOG_INFO("inner lat error: %.4f, head error: %.4f,", inner_lat_error_result, inner_head_error_result);
        LOG_INFO("x: %.4f, y: %.4f, head: %.4f", x, y, curr_heading_result);
        LOG_INFO("record x: %.4f, record_y: %.4f, record_head: %.4f", record_x_result, record_y_result,
                 record_heading_result);
    }

    *curr_index = curr_index_result;
    *lateral_error = lateral_error_result;
    *curr_heading = curr_heading_result;
    *ref_heading = ref_heading_result;
    *heading_error = heading_error_result;
    *inner_lat_error = inner_lat_error_result;
    *inner_head_error = inner_head_error_result;
    *record_x = record_x_result;
    *record_y = record_y_result;
    *record_heading = record_heading_result;
    *target_index = target_index_result;

    return true;
}

bool TrajectoryAnalyzer::GetAbsoluteTimeSpeedAndStationError(const ADCTrajectory* const trajectory_message,
                                                             const int32_t closest_index, const double curr_speed,
                                                             const double forward_time, double* target_speed,
                                                             double* speed_error, double* target_station,
                                                             double* station_error, double* target_acc,
                                                             double* record_speed, double* record_acceleration) {
    if (trajectory_message == nullptr) {
        LOG_ERROR("trajectory_message is nullptr!");
        return false;
    }
    if (trajectory_message->trajectory_point_size() < 1) {
        if (FLAGS_enable_control_info_print) {
            LOG_ERROR("pnc_trajectory_points_size < 1");
        }
        return false;
    }
    if (closest_index >= trajectory_message->trajectory_point_size()) {
        LOG_ERROR("closest_index %d >= pts size %d", closest_index, trajectory_message->trajectory_point_size());
        return false;
    }
    double curr_time = TimeUtility::GetCurrentTimesecond();
    double planning_time = trajectory_message->header().timestamp_sec();
    double delta_time = curr_time - planning_time;
    if (delta_time < 0.0) {
        LOG_WARN("delta_time[%.4f] less than 0, use closest pt instead", delta_time);
        delta_time = trajectory_message->trajectory_point(closest_index).relative_time();
    }

    if (delta_time >= 100.0) {
        LOG_INFO("current pos is outside of planning valid traj");
        return false;
    }

    int32_t index_target = -1;
    for (int32_t i = 0; i < trajectory_message->trajectory_point_size(); ++i) {
        if (trajectory_message->trajectory_point(i).relative_time() - delta_time >= forward_time) {
            index_target = i;
            break;
        }
    }
    if (index_target < 0) {
        LOG_ERROR("failed to find target point with time: %.4f", delta_time + forward_time);
        return false;
    }

    double target_speed_result = trajectory_message->trajectory_point(index_target).v();
    double speed_error_result = target_speed_result - curr_speed;

    double target_station_result = trajectory_message->trajectory_point(index_target).path_point().s();
    double station_error_result =
        target_station_result - trajectory_message->trajectory_point(closest_index).path_point().s();
    double target_acc_result = trajectory_message->trajectory_point(index_target).a();
    if (FLAGS_enable_control_info_print) {
        LOG_INFO("current_index: %d, target_index: %d", closest_index, index_target);
    }

    // record 0.1s speed and accel
    index_target = closest_index;
    for (int32_t i = 0; i < trajectory_message->trajectory_point_size(); ++i) {
        if (trajectory_message->trajectory_point(i).relative_time() - delta_time >= 0.1) {
            index_target = i;
            break;
        }
    }
    double record_speed_result = trajectory_message->trajectory_point(index_target).v();
    double record_acceleration_result = trajectory_message->trajectory_point(index_target).a();

    *target_speed = target_speed_result;
    *speed_error = speed_error_result;
    *target_station = target_station_result;
    *station_error = station_error_result;
    *target_acc = target_acc_result;
    *record_speed = record_speed_result;
    *record_acceleration = record_acceleration_result;

    return true;
}

void TrajectoryAnalyzer::GetSL(const ADCTrajectory* const trajectory_message, const double& x, const double& y,
                               double* const station, double* const lateral, int32_t* const index) {
    if (trajectory_message == nullptr || station == nullptr || lateral == nullptr || index == nullptr) {
        LOG_ERROR("input is nullptr!");
        return;
    }

    // step 1: find the closest point along pathmsg
    const auto& trajectory_points = trajectory_message->trajectory_point();
    if (trajectory_points.size() <= 0) {
        if (FLAGS_enable_control_info_print) {
            LOG_ERROR("Trajectory point size is 0!");
        }
        return;
    } else if (trajectory_points.size() == 1) {
        *station = trajectory_points[0].path_point().s();
        *lateral = 0;
        *index = 0;
        LOG_WARN("Trajectory has only one point!");
        return;
    }

    double min_dis = std::numeric_limits<double>::max();
    for (int32_t i = 0; i < trajectory_points.size(); ++i) {
        double tmp_dis =
            std::hypot(trajectory_points[i].path_point().x() - x, trajectory_points[i].path_point().y() - y);
        if (tmp_dis < min_dis) {
            min_dis = tmp_dis;
            *index = i;
        }
    }

    // get s,l
    *station =
        (x - trajectory_points[*index].path_point().x()) * std::cos(trajectory_points[*index].path_point().theta()) +
        (y - trajectory_points[*index].path_point().y()) * std::sin(trajectory_points[*index].path_point().theta()) +
        trajectory_points[*index].path_point().s();
    *lateral =
        (trajectory_points[*index].path_point().x() - x) * std::sin(trajectory_points[*index].path_point().theta()) +
        (y - trajectory_points[*index].path_point().y()) * std::cos(trajectory_points[*index].path_point().theta());
}

/*
 * if current position p is before first point of trajectory path, path_remain
 * is + p is out of last point of trajectory path, path_remain is -
 */
double TrajectoryAnalyzer::GetPathRemain(const ADCTrajectory* const trajectory_message, const double current_station,
                                         const double speed_deadzone, const bool full_stop) {
    if (trajectory_message == nullptr) {
        LOG_ERROR("trajectory_message is nullptr!");
        return 0.0;
    }
    if (trajectory_message->trajectory_point_size() < 1) {
        LOG_ERROR("pnc_trajectory_points_size is empty!");
        return 0.0;
    }
    int32_t stop_index = trajectory_message->trajectory_point_size() - 1;
    // ignore invalid points
    while (trajectory_message->trajectory_point(stop_index).relative_time() > 100) {
        --stop_index;
    }
    if (stop_index < 0 || stop_index >= trajectory_message->trajectory_point_size()) {
        LOG_ERROR("stop_index [%d] calculated error", stop_index);
        return 0.0;
    }
    LOG_INFO("last point: spd: %.6f, acc: %.6f, ", trajectory_message->trajectory_point(stop_index).v(),
             trajectory_message->trajectory_point(stop_index).path_point().s());
    if (!full_stop) {
        if (std::fabs(trajectory_message->trajectory_point(stop_index).v()) < speed_deadzone &&
            trajectory_message->trajectory_point(stop_index).path_point().s() < FLAGS_control_stop_acc) {
            LOG_INFO("the last point stop");
        } else {
            LOG_INFO("the last point nostop");
            return 10000;
        }
    }
    return trajectory_message->trajectory_point(stop_index).path_point().s() - current_station;
}

pnc::common::PathPoint TrajectoryAnalyzer::FindMinDistancePoint(const pnc::common::PathPoint& p0,
                                                                const pnc::common::PathPoint& p1, const double x,
                                                                const double y) const {
    // given the fact that the discretized trajectory is dense enough,
    // we assume linear trajectory between consecutive trajectory points.
    auto dist_square = [&p0, &p1, &x, &y](const double s) {
        double px = pnc::interpolation::Lerp(p0.x(), p0.s(), p1.x(), p1.s(), s);
        double py = pnc::interpolation::Lerp(p0.y(), p0.s(), p1.y(), p1.s(), s);
        double dx = px - x;
        double dy = py - y;
        return dx * dx + dy * dy;
    };

    auto p = p0;
    double s = ControlMath::GoldenSectionSearch(dist_square, p0.s(), p1.s());
    p.set_s(s);
    p.set_x(pnc::interpolation::Lerp(p0.x(), p0.s(), p1.x(), p1.s(), s));
    p.set_y(pnc::interpolation::Lerp(p0.y(), p0.s(), p1.y(), p1.s(), s));
    p.set_theta(pnc::interpolation::Slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s));
    // approximate the curvature at the intermediate point
    p.set_kappa(pnc::interpolation::Lerp(p0.kappa(), p0.s(), p1.kappa(), p1.s(), s));
    return p;
}

// Squared distance from the point to (x, y).
double TrajectoryAnalyzer::PointDistanceSquare(const TrajectoryPoint& point, const double x, const double y) const {
    const double dx = point.path_point().x() - x;
    const double dy = point.path_point().y() - y;
    return dx * dx + dy * dy;
}

std::vector<TrajectoryPoint> TrajectoryAnalyzer::ToVehicleFrame(const TrajectoryPoint& origin_point) const {
    std::vector<TrajectoryPoint> trajectory_in_local_frame;
    for (const auto& point_glb : trajectory_points_) {
        TrajectoryPoint traj_point_loc;
        traj_point_loc.CopyFrom(point_glb);
        traj_point_loc.mutable_path_point()->set_x(
            (point_glb.path_point().x() - origin_point.path_point().x()) * std::cos(origin_point.path_point().theta()) +
            (point_glb.path_point().y() - origin_point.path_point().y()) * std::sin(origin_point.path_point().theta()));
        traj_point_loc.mutable_path_point()->set_y(
            (origin_point.path_point().x() - point_glb.path_point().x()) * std::sin(origin_point.path_point().theta()) +
            (point_glb.path_point().y() - origin_point.path_point().y()) * std::cos(origin_point.path_point().theta()));
        traj_point_loc.mutable_path_point()->set_theta(
            NormalizeAngle(point_glb.path_point().theta() - origin_point.path_point().theta()));
        trajectory_in_local_frame.push_back(traj_point_loc);
    }
    return trajectory_in_local_frame;
}

}  // namespace control
}  // namespace jiduauto
