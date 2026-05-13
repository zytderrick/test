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

#include <gtest/gtest.h>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/common/file_util.h"
#include "control/src/math/control_math.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/pnc_utility.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_control_config.pb.h"
#include "pnc_lqr_controller_conf.pb.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::EulerAnglesZXYd;
using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::util::TimeUtility;
using pnc::chassis::Chassis;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

class TrajectoryAnalyzerTest : public ::testing::Test {
public:
    virtual void SetUp() {
        CHECK(FileUtil::ReadPnCPath());
        std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));
        lqr_conf_paras_ = control_conf_.lqr_controller_conf().forward_paras(0);
        speed_conf_paras_ = control_conf_.pid_speed_controller_conf().forward_paras(0);
    }

protected:
    pnc::control::ControlConfig control_conf_;
    pnc::control::LqrTable lqr_conf_paras_;
    pnc::control::PidSpeedTable speed_conf_paras_;
};

void SetTrajectory(const std::vector<double>& xs, const std::vector<double>& ys, ADCTrajectory* adc_trajectory) {
    for (size_t i = 0; i < xs.size(); ++i) {
        auto* point = adc_trajectory->add_trajectory_point();
        point->mutable_path_point()->set_x(xs[i]);
        point->mutable_path_point()->set_y(ys[i]);
    }
    adc_trajectory->mutable_header()->set_sequence_num(123);
}

void SetTrajectoryWithTime(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& ts,
                           ADCTrajectory* adc_trajectory) {
    std::vector<double> ss;
    std::vector<double> headings;
    ss.push_back(0);

    for (std::size_t i = 1; i < xs.size(); i++) {
        ss.push_back(ss[i - 1] + std::sqrt(2) * 0.1);
    }

    std::vector<double> dxs;
    std::vector<double> dys;

    for (std::size_t i = 0; i < xs.size(); i++) {
        double dx = 0.0;
        double dy = 0.0;
        if (i == 0) {
            dx = xs[i + 1] - xs[i];
            dy = ys[i + 1] - ys[i];
        } else if (i == xs.size() - 1) {
            dx = xs[i] - xs[i - 1];
            dy = ys[i] - ys[i - 1];
        } else {
            dx = 0.5 * (xs[i + 1] - xs[i - 1]);
            dy = 0.5 * (ys[i + 1] - ys[i - 1]);
        }
        dxs.push_back(dx);
        dys.push_back(dy);
    }

    for (std::size_t i = 0; i < xs.size(); i++) {
        headings.push_back(std::atan2(dys[i], dxs[i]));
    }

    for (size_t i = 0; i < xs.size(); ++i) {
        auto* point = adc_trajectory->add_trajectory_point();
        point->mutable_path_point()->set_x(xs[i]);
        point->mutable_path_point()->set_y(ys[i]);
        point->mutable_path_point()->set_s(ss[i]);
        point->mutable_path_point()->set_theta(headings[i]);
        point->set_relative_time(ts[i]);
        point->set_v(1.0);
        point->set_a(0.0);
    }
}

void SetTrajectory(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& ss,
                   ADCTrajectory* adc_trajectory) {
    for (size_t i = 0; i < xs.size(); ++i) {
        auto* point = adc_trajectory->add_trajectory_point();
        point->mutable_path_point()->set_x(xs[i]);
        point->mutable_path_point()->set_y(ys[i]);
        point->mutable_path_point()->set_s(ss[i]);
    }
}

TEST_F(TrajectoryAnalyzerTest, SetADCTrajectory) {
    ADCTrajectory adc_trajectory;
    std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
    SetTrajectory(xs, ys, &adc_trajectory);
    int traj_size = adc_trajectory.trajectory_point_size();
    EXPECT_EQ(traj_size, 5);
    for (int i = 0; i < traj_size; ++i) {
        EXPECT_EQ(adc_trajectory.trajectory_point(i).path_point().x(), xs[i]);
        EXPECT_EQ(adc_trajectory.trajectory_point(i).path_point().y(), ys[i]);
    }
}

TEST_F(TrajectoryAnalyzerTest, Constructor) {
    ADCTrajectory adc_trajectory;
    std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
    SetTrajectory(xs, ys, &adc_trajectory);

    TrajectoryAnalyzer trajectory_analyzer(&adc_trajectory);
    EXPECT_EQ(trajectory_analyzer.trajectory_points().size(), 5);
    int i = 0;
    for (auto& point : trajectory_analyzer.trajectory_points()) {
        EXPECT_EQ(xs[i], point.path_point().x());
        EXPECT_EQ(ys[i], point.path_point().y());
        ++i;
    }
    EXPECT_EQ(trajectory_analyzer.seq_num(), 123);
}

TEST_F(TrajectoryAnalyzerTest, QueryMatchedPathPoint) {
    ADCTrajectory adc_trajectory;
    std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.8};
    std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.8};
    std::vector<double> ss = {1.0, 1.1, 1.2, 1.3, 1.8};
    SetTrajectory(xs, ys, ss, &adc_trajectory);
    TrajectoryAnalyzer trajectory_analyzer(&adc_trajectory);

    auto point = trajectory_analyzer.QueryMatchedPathPoint(1.21, 1.21);
    EXPECT_NEAR(point.x(), 1.21, 1e-3);
    EXPECT_NEAR(point.y(), 1.21, 1e-3);

    point = trajectory_analyzer.QueryMatchedPathPoint(1.56, 1.50);
    EXPECT_NEAR(point.x(), 1.53, 1e-3);
    EXPECT_NEAR(point.y(), 1.53, 1e-3);
}

TEST_F(TrajectoryAnalyzerTest, ToTrajectoryFrame) {
    ADCTrajectory adc_trajectory;
    std::vector<double> xs = {0.8, 0.9, 1.0, 1.1, 1.2};
    std::vector<double> ys = {0.0, 0.0, 0.0, 0.0, 0.0};
    SetTrajectory(xs, ys, &adc_trajectory);

    TrajectoryAnalyzer trajectory_analyzer(&adc_trajectory);
    double x = 2.0;
    double y = 1.0;
    double theta = M_PI / 4.0;
    double v = 1.0;
    pnc::common::PathPoint ref_point;
    ref_point.set_x(1.0);
    ref_point.set_y(0.0);
    ref_point.set_theta(0.0);
    ref_point.set_s(1.0);

    double s = 0.0;
    double s_dot = 0.0;
    double d = 0.0;
    double d_dot = 0.0;
    trajectory_analyzer.ToTrajectoryFrame(x, y, theta, v, ref_point, &s, &s_dot, &d, &d_dot);
    EXPECT_NEAR(s, 2.0, 1e-6);
    EXPECT_NEAR(s_dot, 0.707, 1e-3);
    EXPECT_NEAR(d, 1.0, 1e-3);
    EXPECT_NEAR(d_dot, 0.707, 1e-3);
}

// 有问题
TEST_F(TrajectoryAnalyzerTest, QueryNearestPointByAbsoluteTimeInterpolation) {
    ADCTrajectory adc_trajectory;
    std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ts = {0.0, 0.1, 0.2, 0.3, 0.4};
    SetTrajectoryWithTime(xs, ys, ts, &adc_trajectory);

    double timestamp_ = TimeUtility::GetCurrentTimesecond() - 2.0;
    adc_trajectory.mutable_header()->set_timestamp_sec(timestamp_);
    TrajectoryAnalyzer trajectory_analyzer(&adc_trajectory);

    double current_time = TimeUtility::GetCurrentTimesecond() - 20.0;
    auto point_2 = trajectory_analyzer.QueryNearestPointByAbsoluteTime(current_time);
    EXPECT_NEAR(point_2.path_point().x(), 1.0, 1e-6);

    current_time = timestamp_ + 50.0;
    auto point_4 = trajectory_analyzer.QueryNearestPointByAbsoluteTime(current_time);
    EXPECT_NEAR(point_4.path_point().x(), 1.4, 1e-6);

    current_time = timestamp_ + 0.2;
    auto point_6 = trajectory_analyzer.QueryNearestPointByAbsoluteTime(current_time);
    EXPECT_NEAR(point_6.path_point().x(), 1.2, 1e-6);
}

TEST_F(TrajectoryAnalyzerTest, GetAbsoluteTimeLateralAndHeadingError) {
    ADCTrajectory* adc_trajectory = new ADCTrajectory();
    LocalizationEstimate* adc_localization = new LocalizationEstimate();

    std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ts = {0.0, 0.1, 0.2, 0.3, 0.4};
    SetTrajectoryWithTime(xs, ys, ts, adc_trajectory);
    double timestamp_ = TimeUtility::GetCurrentTimesecond();
    adc_trajectory->mutable_header()->set_timestamp_sec(timestamp_);

    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_position()->set_x(1.2);
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_position()->set_y(1.15);

    EulerAnglesZXYd euler_angle(M_PI / 4);  // -M_PI/4
    Eigen::Quaternion<double> q = euler_angle.ToQuaternion();

    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qx(q.x());
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qy(q.y());
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qz(q.z());
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qw(q.w());

    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_x(1.0);

    TrajectoryAnalyzer trajectory_analyzer;
    int current_index_ = 0;
    double current_lateral_ = 0;
    double current_heading_ = 0;
    double current_ref_heading_ = 0;
    double current_heading_error_ = 1.0;
    double inner_lateral_error_ = 0;
    double inner_heading_error_ = 0;
    double record_x_ = 0;
    double record_y_ = 0;
    double record_head_ = 0;
    int target_index = 0;
    double heading_offset = 0;

    trajectory_analyzer.GetAbsoluteTimeLateralAndHeadingError(
        adc_localization, adc_trajectory, adc_localization->loc_pose().pose().position().x(),
        adc_localization->loc_pose().pose().position().y(), lqr_conf_paras_.lateral_preview_time(),
        lqr_conf_paras_.min_preview_dis(), heading_offset, &current_index_, &current_lateral_, &current_heading_,
        &current_ref_heading_, &current_heading_error_, &inner_lateral_error_, &inner_heading_error_, &record_x_,
        &record_y_, &record_head_, &target_index);
    EXPECT_NEAR(current_heading_error_, 0.0, 1e-6);
    EXPECT_NEAR(current_lateral_, -0.0354, 1e-3);
    EXPECT_NEAR(inner_lateral_error_, -0.0354, 1e-3);
    EXPECT_NEAR(inner_heading_error_, 0.0, 1e-6);
    EXPECT_EQ(target_index, 2);
}

TEST_F(TrajectoryAnalyzerTest, GetAbsoluteTimeSpeedAndStationErrorTest) {
    ADCTrajectory* adc_trajectory = new ADCTrajectory();

    std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ts = {0.0, 0.1, 0.2, 0.3, 0.4};
    SetTrajectoryWithTime(xs, ys, ts, adc_trajectory);
    double timestamp_ = TimeUtility::GetCurrentTimesecond();
    adc_trajectory->mutable_header()->set_timestamp_sec(timestamp_);

    TrajectoryAnalyzer trajectory_analyzer;

    double cur_index = 2;
    double current_speed_ = 1.0;
    double target_speed = 0.0;
    double speed_error_ = 0.0;
    double target_station_ = 0.0;
    double distance_error_ = 0.0;
    double target_acc_ = 0.0;
    double record_speed = -1.0;
    double record_accel = -10.0;

    trajectory_analyzer.GetAbsoluteTimeSpeedAndStationError(
        adc_trajectory, cur_index, current_speed_, speed_conf_paras_.forsee_time(), &target_speed, &speed_error_,
        &target_station_, &distance_error_, &target_acc_, &record_speed, &record_accel);

    EXPECT_NEAR(speed_error_, 0.0, 1e-6);
    EXPECT_NEAR(distance_error_, 0.0, 0.0);  // no time error
    EXPECT_NEAR(target_acc_, 0.0, 1e-6);
    EXPECT_NEAR(target_station_, 0.2828, 1e-3);
    EXPECT_NEAR(record_speed, 1.0, 1e-6);
    EXPECT_NEAR(record_accel, 0.0, 1e-6);
}

TEST_F(TrajectoryAnalyzerTest, GetSL) {
    ADCTrajectory* adc_trajectory = new ADCTrajectory();

    std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<double> ts = {0.0, 0.1, 0.2, 0.3, 0.4};
    SetTrajectoryWithTime(xs, ys, ts, adc_trajectory);

    double timestamp_ = TimeUtility::GetCurrentTimesecond();
    adc_trajectory->mutable_header()->set_timestamp_sec(timestamp_);

    TrajectoryAnalyzer trajectory_analyzer;
    LocalizationEstimate* adc_localization = new LocalizationEstimate();
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_position()->set_x(1.2);
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_position()->set_y(1.15);

    EulerAnglesZXYd euler_angle(-M_PI / 4);
    Eigen::Quaternion<double> q = euler_angle.ToQuaternion();

    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qx(q.x());
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qy(q.y());
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qz(q.z());
    adc_localization->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qw(q.w());

    double lateral = 0.0;
    double station = 0.0;
    int index = 0.0;
    trajectory_analyzer.GetSL(adc_trajectory, adc_localization->loc_pose().pose().position().x(),
                              adc_localization->loc_pose().pose().position().y(), &station, &lateral, &index);

    EXPECT_NEAR(station, 0.2475, 1e-3);
    EXPECT_NEAR(lateral, -0.0354, 1e-3);
    EXPECT_EQ(index, 2);
}
}  // namespace control
}  // namespace jiduauto
