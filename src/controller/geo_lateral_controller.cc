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

#include "control/src/controller/geo_lateral_controller.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "control/src/math/control_math.h"
#include "control/src/math/trajectory_analyzer.h"
#include "pnc_common/src/common/pnc_gflag.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

GeoLateralController::GeoLateralController() { LOG_INFO("%s used. ", Name().c_str()); }

GeoLateralController::~GeoLateralController() { Stop(); }

bool GeoLateralController::Init(const pnc::control::ControlConfig& control_conf) {
    // digital filter
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    vehicle_param_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();
    geo_lat_controller_conf_ = control_conf.geo_lat_controller_conf();

    target_steer_angle_filter_.SetCoefficients(den, num);
    controller_initialized_ = true;

    return true;
}

bool GeoLateralController::Control(const pnc::localization::LocalizationEstimate* const localization,
                                   const pnc::chassis::Chassis* const chassis,
                                   const pnc::planning::ADCTrajectory* const trajectory,
                                   pnc::control::ControlCommand* const cmd) {
    return false;
}

bool GeoLateralController::CalculateSteerCmd() {
    if (nullptr == trajectory_msg_) return false;

    std::vector<pnc::common::TrajectoryPoint> trajectory_points;
    for (auto& point : trajectory_msg_->trajectory_point()) {
        trajectory_points.emplace_back(point);
    }
    int32_t next_target_point_id = 0;
    if (!CalculateNextTargetPoint(trajectory_points, lookahead_distance_, &next_target_point_id)) {
        LOG_ERROR("calculate next target point failed");
        return false;
    }
    pnc::common::TrajectoryPoint target_point;
    if (next_target_point_id >= 0 && next_target_point_id < trajectory_points.size()) {
        target_point = trajectory_points.at(next_target_point_id);
        LOG_WARN(" target point x :(%.3f) y:(%.3f) z:(%.3f)next_target_point_id:(%d)", target_point.path_point().x(),
                 target_point.path_point().y(), target_point.path_point().z(), next_target_point_id);
        if (geo_lat_controller_conf_.use_interpolate_in_low_speed() &&
            current_speed_ < geo_lat_controller_conf_.low_speed() &&
            InterpolateNextTargetPoint(next_target_point_id, trajectory_points, &target_point)) {
            LOG_WARN("interpolate  target point x :(%.3f) y:(%.3f) z:(%.3f)", target_point.path_point().x(),
                     target_point.path_point().y(), target_point.path_point().z());
        }
    }

    double kappa = 0.0;
    kappa = CalculateCurvature(target_point);
    target_steer_angle_ = std::atan(kappa * vehicle_param_.wheel_base()) * 180.0 / 3.14 * vehicle_param_.steer_ratio();
    LOG_INFO("target steer angle:%.3f kappa:%.3f wheelbase:%.3f steer trans ratio:%.3f", target_steer_angle_, kappa,
             vehicle_param_.wheel_base(), vehicle_param_.steer_ratio());
    target_steer_angle_rate_ = kappa * current_speed_;

    return true;
}

double GeoLateralController::CalculateCurvature(const pnc::common::TrajectoryPoint& target_point) {
    double kappa = 0.;
    auto current_x = localization_msg_->loc_pose().pose().position().x();
    auto current_y = localization_msg_->loc_pose().pose().position().y();
    double denominator = std::pow((current_x - target_point.path_point().x()), 2) +
                         std::pow((current_y - target_point.path_point().y()), 2);
    LOG_INFO("current_x:%.3f target_point.path_point().x():%.3f current_y:%.3f target_point.path_point().y():%.3f",
             current_x, target_point.path_point().x(), current_y, target_point.path_point().y());
    LOG_INFO("DENOMINATOR:%.3f", denominator);
    pnc::common::TrajectoryPoint target_point_vehicle = ConvertToRelativeCoordinate(target_point);
    current_lateral_error_ = target_point_vehicle.path_point().y();
    LOG_INFO("current_lateral_error_:%.3f", current_lateral_error_);

    double numerator = 2 * target_point_vehicle.path_point().y();
    const double distance = 0.01;
    if (denominator > distance) {
        double kp = 1.0;
        if (current_speed_ > geo_lat_controller_conf_.high_speed()) {
            kp = geo_lat_controller_conf_.high_speed_kp();
        }
        kappa = kp * numerator / denominator;
    } else {
        kappa = 0.;
    }
    return kappa;
}

pnc::common::TrajectoryPoint GeoLateralController::ConvertToRelativeCoordinate(
    const pnc::common::TrajectoryPoint& pt_in_world) {
    pnc::common::TrajectoryPoint point_in_relative;
    double diff_x = pt_in_world.path_point().x() - localization_msg_->loc_pose().pose().position().x();
    double diff_y = pt_in_world.path_point().y() - localization_msg_->loc_pose().pose().position().y();
    double origin_yaw = localization_msg_->loc_pose().pose().position().z();
    auto& orientation = localization_msg_->loc_pose().pose().orientation();
    jiduauto::pnc::EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
    origin_yaw = q.yaw();
    double cosr = cos(origin_yaw);
    double sinr = sin(origin_yaw);
    point_in_relative.mutable_path_point()->set_x(diff_x * cosr + diff_y * sinr);
    point_in_relative.mutable_path_point()->set_y(diff_y * cosr - diff_x * sinr);
    point_in_relative.mutable_path_point()->set_z(
        jiduauto::pnc::math::NormalizeAngle(pt_in_world.path_point().z() - origin_yaw));
    return point_in_relative;
}

bool GeoLateralController::InterpolateNextTargetPoint(
    int32_t next_target_point_id, const std::vector<pnc::common::TrajectoryPoint>& trajectory_points,
    pnc::common::TrajectoryPoint* target_point) {
    const double kInterpolationOffset = 10e-5;
    int32_t path_size = trajectory_points.size();
    if (next_target_point_id >= path_size) {
        *target_point = trajectory_points.at(path_size - 1);
        return true;
    } else if (next_target_point_id <= 0) {
        *target_point = trajectory_points.at(0);
        return true;
    }
    return true;  // TODO(zhaobolin): have no idea why this ...
    auto end_point = trajectory_points.at(next_target_point_id);
    double direction = end_point.path_point().z();
    double search_radius = lookahead_distance_;
    auto start_point = trajectory_points.at(next_target_point_id - 1);
    double a, b, c = 0.;
    bool get_linear_flag = GetLinearEquation(start_point, end_point, &a, &b, &c);

    if (!get_linear_flag) return false;

    auto current_pose = localization_msg_->loc_pose().pose().position();
    double distance =
        std::fabs(a * current_pose.x() + b * current_pose.y() + c) / std::sqrt(a * a + b * b + kInterpolationOffset);
    if (distance > search_radius) return false;

    Eigen::Vector3d v((end_point.path_point().x() - start_point.path_point().x()),
                      (end_point.path_point().y() - start_point.path_point().y()), 0);
    Eigen::AngleAxisd t1(M_PI / 2, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd t2(-M_PI / 2, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d unit_v = v.normalized();
    Eigen::Vector3d unit_w1 = t1.toRotationMatrix() * unit_v;
    Eigen::Vector3d unit_w2 = t2.toRotationMatrix() * unit_v;

    pnc::common::TrajectoryPoint h1;
    h1.mutable_path_point()->set_x(current_pose.x() + distance * unit_w1.x());
    h1.mutable_path_point()->set_y(current_pose.y() + distance * unit_w1.y());
    h1.mutable_path_point()->set_z(current_pose.z());
    pnc::common::TrajectoryPoint h2;
    h2.mutable_path_point()->set_x(current_pose.x() + distance * unit_w2.x());
    h2.mutable_path_point()->set_y(current_pose.y() + distance * unit_w2.y());
    h2.mutable_path_point()->set_z(current_pose.z());

    pnc::common::TrajectoryPoint h;
    if (fabs(a * h1.path_point().x() + b * h1.path_point().y() + c) < kInterpolationOffset) {
        h = h1;
        LOG_DEBUG("use h1");
    } else if (fabs(a * h2.path_point().x() + b * h2.path_point().y() + c) < kInterpolationOffset) {
        LOG_DEBUG("use h2");
        h = h2;
    } else {
        return false;
    }

    if (distance == search_radius) {
        target_point->CopyFrom(h);
        target_point->mutable_path_point()->set_z(direction);
        return true;
    } else {
        // if there are two intersection
        // get intersection in front of vehicle
        double s = sqrt(pow(search_radius, 2) - pow(distance, 2));
        pnc::common::TrajectoryPoint target1;
        target1.mutable_path_point()->set_x(h.path_point().x() + s * unit_v.x());
        target1.mutable_path_point()->set_y(h.path_point().y() + s * unit_v.y());
        target1.mutable_path_point()->set_z(current_pose.z());

        pnc::common::TrajectoryPoint target2;
        target2.mutable_path_point()->set_x(h.path_point().x() - s * unit_v.x());
        target2.mutable_path_point()->set_y(h.path_point().y() - s * unit_v.y());
        target2.mutable_path_point()->set_z(current_pose.z());

        double interval = std::sqrt(std::pow(end_point.path_point().x() - start_point.path_point().x(), 2) +
                                    std::pow(end_point.path_point().y() - start_point.path_point().y(), 2));
        double end_target1_interval = std::sqrt(std::pow(end_point.path_point().x() - target1.path_point().x(), 2) +
                                                std::pow(end_point.path_point().y() - target1.path_point().y(), 2));
        double end_target2_interval = std::sqrt(std::pow(end_point.path_point().x() - target2.path_point().x(), 2) +
                                                std::pow(end_point.path_point().y() - target2.path_point().y(), 2));
        LOG_DEBUG("interval: {}, plane dis: {} {}", interval, end_target1_interval, end_target2_interval);
        if (end_target1_interval <= interval) {
            LOG_WARN("result : target1");
            target_point->CopyFrom(target1);
            target_point->mutable_path_point()->set_z(direction);
            return true;
        } else if (end_target2_interval <= interval) {
            LOG_WARN("result : target2");
            target_point->CopyFrom(target2);
            target_point->mutable_path_point()->set_z(direction);
            return true;
        } else {
            LOG_WARN("result : false ");
            return false;
        }
    }
    return true;
}

bool GeoLateralController::GetLinearEquation(const pnc::common::TrajectoryPoint& start,
                                             const pnc::common::TrajectoryPoint& end, double* a, double* b, double* c) {
    double sub_x = std::fabs(start.path_point().x() - end.path_point().x());
    double sub_y = std::fabs(start.path_point().y() - end.path_point().y());
    const double kEps = 10e-5;

    if (sub_x < kEps && sub_y < kEps) {
        LOG_WARN("two points are the same point!!");
        return false;
    }

    *a = end.path_point().y() - start.path_point().y();
    *b = (-1.0) * (end.path_point().x() - start.path_point().x());
    *c = (-1.0) * (end.path_point().y() - start.path_point().y()) * start.path_point().x() +
         (end.path_point().x() - start.path_point().x()) * start.path_point().y();

    return true;
}

bool GeoLateralController::CalculateNextTargetPoint(const std::vector<pnc::common::TrajectoryPoint>& trajectory_points,
                                                    double lookahead_distance, int32_t* next_target_point_id) {
    if (trajectory_points.empty()) {
        LOG_ERROR("trajecotry is empty");
        return false;
    }
    auto current_x = localization_msg_->loc_pose().pose().position().x();
    auto current_y = localization_msg_->loc_pose().pose().position().y();
    while (*next_target_point_id < trajectory_points.size() - 1) {
        auto point = trajectory_points.at(*next_target_point_id);
        double dist_diff = std::sqrt(std::pow((current_x - point.path_point().x()), 2) +
                                     std::pow((current_y - point.path_point().y()), 2));
        if (dist_diff >= lookahead_distance) {
            break;
        } else {
            (*next_target_point_id)++;
        }
    }
    return true;
}

double GeoLateralController::CalculateLookaheadDistance() {
    double ratio = CalculateLookaheadDistanceRatio();
    double lookahead_distance = current_speed_ * ratio;
    double max_lookahead_dist = geo_lat_controller_conf_.maximum_lookahead_distance();
    double traj_point_max_kappa = std::numeric_limits<double>::min();
    const double curve_radius_25_reciprocal = 0.04;
    const double curve_radius_100_reciprocal = 0.01;
    const double curve_radius_200_reciprocal = 0.005;
    const double curve_radius_500_reciprocal = 0.002;
    const double curve_radius_800_reciprocal = 0.00125;

    for (const auto& point : trajectory_msg_->trajectory_point()) {
        if (std::fabs(point.path_point().kappa()) > traj_point_max_kappa) {
            traj_point_max_kappa = std::fabs(point.path_point().kappa());
            LOG_INFO("traj_point_max_kappa : %f max look dis%f", traj_point_max_kappa, max_lookahead_dist);
        }
    }
    if (traj_point_max_kappa > curve_radius_25_reciprocal) {
        max_lookahead_dist = geo_lat_controller_conf_.curve25_maximum_lookahead_distance();
        LOG_INFO("curr curvature radius < 25 kappa: %f max look dis%f", traj_point_max_kappa, max_lookahead_dist);
    } else if (traj_point_max_kappa > curve_radius_100_reciprocal) {
        max_lookahead_dist = geo_lat_controller_conf_.curve25_maximum_lookahead_distance() +
                             (traj_point_max_kappa - curve_radius_25_reciprocal) /
                                 (curve_radius_100_reciprocal - curve_radius_25_reciprocal) *
                                 (geo_lat_controller_conf_.curve100_maximum_lookahead_distance() -
                                  geo_lat_controller_conf_.curve25_maximum_lookahead_distance());
        LOG_INFO("curr 25 < curvature radius < 100 kappa: %f max look dis%f", traj_point_max_kappa, max_lookahead_dist);
    } else if (traj_point_max_kappa > curve_radius_200_reciprocal) {
        max_lookahead_dist = geo_lat_controller_conf_.curve100_maximum_lookahead_distance() +
                             (traj_point_max_kappa - curve_radius_100_reciprocal) /
                                 (curve_radius_200_reciprocal - curve_radius_100_reciprocal) *
                                 (geo_lat_controller_conf_.curve200_maximum_lookahead_distance() -
                                  geo_lat_controller_conf_.curve100_maximum_lookahead_distance());
        LOG_INFO("curr 100 < curvature radius < 200 kappa: %f max look dis%f", traj_point_max_kappa,
                 max_lookahead_dist);
    } else if (traj_point_max_kappa > curve_radius_500_reciprocal) {
        max_lookahead_dist = geo_lat_controller_conf_.curve200_maximum_lookahead_distance() +
                             (traj_point_max_kappa - curve_radius_200_reciprocal) /
                                 (curve_radius_500_reciprocal - curve_radius_200_reciprocal) *
                                 (geo_lat_controller_conf_.curve500_maximum_lookahead_distance() -
                                  geo_lat_controller_conf_.curve200_maximum_lookahead_distance());
        LOG_INFO("curr 200 < curvature radius < 500 kappa: %f max look dis%f", traj_point_max_kappa,
                 max_lookahead_dist);
    } else if (traj_point_max_kappa > curve_radius_800_reciprocal) {
        max_lookahead_dist = geo_lat_controller_conf_.curve500_maximum_lookahead_distance() +
                             (traj_point_max_kappa - curve_radius_500_reciprocal) /
                                 (curve_radius_800_reciprocal - curve_radius_500_reciprocal) *
                                 (geo_lat_controller_conf_.curve800_maximum_lookahead_distance() -
                                  geo_lat_controller_conf_.curve500_maximum_lookahead_distance());
        LOG_INFO("curr 500 < curvature radius < 800 kappa: %f max look dis%f", traj_point_max_kappa,
                 max_lookahead_dist);
    }

    double minimum_lookahead_distance = geo_lat_controller_conf_.low_speed_minimum_lookahead_distance();
    if (current_speed_ > geo_lat_controller_conf_.low_speed()) {
        minimum_lookahead_distance = geo_lat_controller_conf_.high_speed_minimum_lookahead_distance();
    }
    lookahead_distance = std::fmax(minimum_lookahead_distance, lookahead_distance);
    lookahead_distance = std::fmin(lookahead_distance, max_lookahead_dist);
    LOG_INFO("lookahead_distance : %f", lookahead_distance);

    return lookahead_distance;
}

double GeoLateralController::CalculateLookaheadDistanceRatio() {
    InterpolationXY::InterpolatData xy;
    xy.emplace_back(std::make_pair(geo_lat_controller_conf_.low_speed(), geo_lat_controller_conf_.low_ratio()));
    xy.emplace_back(std::make_pair(geo_lat_controller_conf_.low_speed() + geo_lat_controller_conf_.smooth_vel_delta(),
                                   geo_lat_controller_conf_.med_ratio()));
    xy.emplace_back(std::make_pair(geo_lat_controller_conf_.high_speed() - geo_lat_controller_conf_.smooth_vel_delta(),
                                   geo_lat_controller_conf_.med_ratio()));
    xy.emplace_back(std::make_pair(geo_lat_controller_conf_.high_speed(), geo_lat_controller_conf_.high_ratio()));
    ratio_interpolation_->Init(xy);
    double ratio = ratio_interpolation_->Interpolate(current_speed_);
    ratio = std::fmax(ratio, geo_lat_controller_conf_.minimum_ratio());
    // TODO(someone): tuning ratio value according scenario
    return ratio;
}

bool GeoLateralController::Reset() {
    target_steer_angle_filter_.Reset();
    return true;
}

std::string GeoLateralController::Name() const { return "GeoLateralController"; }

void GeoLateralController::Stop() {
    // TODO(someone): reserved.
    localization_msg_ = nullptr;
    chassis_msg_ = nullptr;
    trajectory_msg_ = nullptr;
}

bool GeoLateralController::ControlMain(const ControlInputStream& control_input_stream,
                                       ControlCommandStream* const control_command_stream) {
    if (control_command_stream == nullptr) {
        LOG_ERROR("input is nullptr");
        return false;
    }

    if (!controller_initialized_) {
        LOG_ERROR("controller_initialized_ failed");
        return false;
    }
    localization_msg_ = &control_input_stream.input_stream.localization;
    chassis_msg_ = &control_input_stream.input_stream.chassis;
    trajectory_msg_ = &control_input_stream.input_stream.planning_info;

    bool curr_backward = ControlMath::CheckBackward(chassis_msg_);
    if (curr_backward != is_current_backward_) {
        is_current_backward_ = curr_backward;
        Reset();
        LOG_INFO("Reset()");
    }

    if (FLAGS_enable_using_localization_velocity) {
        const auto& speed3d = control_input_stream.input_stream.localization.loc_pose().pose().linear_velocity();
        current_speed_ = std::hypot(speed3d.x(), speed3d.y());
    } else {
        current_speed_ = control_input_stream.input_stream.chassis.speed_mps();
    }

    lookahead_distance_ = CalculateLookaheadDistance();

    if (!CalculateSteerCmd()) {
        LOG_ERROR("calculate steer cmd failed");
        return false;
    }

    target_steer_angle_ =
        jiduauto::pnc::math::Clamp(target_steer_angle_, -geo_lat_controller_conf_.target_steer_angle_max(),
                                   geo_lat_controller_conf_.target_steer_angle_max());

    LOG_INFO("before ramp filter steer angle: %.4f", target_steer_angle_);

    // add ramp filter, in case of big steer rate--> central acc, central jerk
    LOG_INFO("pre_target_steer_angle_ :%f target_steer_angle_:%f", pre_target_steer_angle_, target_steer_angle_);
    if (target_steer_angle_ > pre_target_steer_angle_ + geo_lat_controller_conf_.steer_delta_limit()) {
        target_steer_angle_ = pre_target_steer_angle_ + geo_lat_controller_conf_.steer_delta_limit();
    } else if (target_steer_angle_ < pre_target_steer_angle_ - geo_lat_controller_conf_.steer_delta_limit()) {
        target_steer_angle_ = pre_target_steer_angle_ - geo_lat_controller_conf_.steer_delta_limit();
    }

    target_steer_angle_ =
        jiduauto::pnc::math::Clamp(target_steer_angle_, -geo_lat_controller_conf_.target_steer_angle_max(),
                                   geo_lat_controller_conf_.target_steer_angle_max());
    LOG_INFO("after ramp filter steer angle: %.4f", target_steer_angle_);

    pre_target_steer_angle_ = target_steer_angle_;
    control_command_stream->control_command.set_steering_target(target_steer_angle_);
    control_command_stream->control_command.set_steering_rate(target_steer_angle_rate_);
    LOG_INFO("geo steer angle: %.4f", control_command_stream->control_command.steering_target());
    return true;
}

}  // namespace control
}  // namespace jiduauto
