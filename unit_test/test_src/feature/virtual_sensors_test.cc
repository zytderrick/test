// Copyright 2023 JiDuAuto LLC
// Author: Yanze LIU, Jidu PnC

#include "control/src/feature/virtual_sensors.h"

#include <gtest/gtest.h>
#include <math.h>

#include <string>
#include <utility>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_message_stream.h"
#include "control/src/common/file_util.h"
#include "control/src/feature/control_message_manager.h"
#include "control/src/math/control_math.h"
#include "pnc_common/src/math/filters/digital_filter.h"
#include "pnc_common/src/math/filters/kalman_filter.h"
#include "pnc_common/src/math/filters/mean_filter.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/utility/pnc_utility.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"
#include "pnc_common_vehicle_param.pb.h"
#include "pnc_control_config.pb.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::EulerAnglesZXYf;
using jiduauto::pnc::util::TimeUtility;
using jiduauto::pnc::vehicle::VehicleConfig;
using pnc::chassis::Chassis;
using pnc::common::Point3D;
using pnc::common::PointENU;
using pnc::localization::LocalizationEstimate;

class VirtualSensorsTest : public ::testing::Test {
public:
    virtual void SetUp() {
        CHECK(FileUtil::ReadPnCPath());
        std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));
        vehicle_paras_ = VehicleConfig::Instance()->GetParas();
    }

    void InitInputStream(bool random = false);
    void InitFilters();
    PointENU GetFilteredPosition(const std::shared_ptr<LocalizationEstimate>& localization);

protected:
    pnc::control::ControlConfig control_conf_;
    pnc::common::VehicleParam vehicle_paras_;
    std::shared_ptr<LocalizationEstimate> localization_msg_;
    std::shared_ptr<Chassis> chassis_msg_;
    std::unique_ptr<ControlMessageManager> control_message_manager_;
    std::unique_ptr<VirtualSensors> virtual_sensors_;

    // filters
    jiduauto::pnc::filter::KalmanFilter<double, 6, 6, 3> kalman_filter_;

    double curr_x_{0.2};
    const double cutoff_freq_{10.0};
    const int window_size_{10};
    double previous_timestamp_sec_{0.0};
};

void VirtualSensorsTest::InitInputStream(bool random) {
    const double curr_time = TimeUtility::GetCurrentTimesecond();

    // Init localization msg
    localization_msg_ = std::make_shared<pnc::localization::LocalizationEstimate>();
    localization_msg_->mutable_loc_pose()->set_measurement_time(curr_time);
    localization_msg_->mutable_header()->set_timestamp_sec(curr_time);
    localization_msg_->mutable_header()->set_module_name("localization");
    localization_msg_->mutable_header()->set_sequence_num(1);

    double t_x = curr_x_;
    double t_y = 0.01 * std::pow(t_x, 3) + 0.1 * std::pow(t_x, 2) - 0.2 * t_x + 0.3;
    if (random) {
        unsigned int seed = time(NULL);
        t_x += (rand_r(&seed) % 20 - 10.0) / 10.0;
        t_y += (rand_r(&seed) % 20 - 10.0) / 10.0;
    }
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_position()->set_x(t_x);
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_position()->set_y(t_y);
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_position()->set_z(0.1);
    curr_x_ += 0.2;

    double t_r = 0.0;                                 // roll
    double t_p = 0.0;                                 // pitch
    t_y = 0.03 * std::pow(t_x, 2) + 0.2 * t_x - 0.2;  // yaw
    t_y = jiduauto::pnc::math::NormalizeAngle(t_y - M_PI_2);
    EulerAnglesZXYf q(t_r, t_p, t_y);
    Eigen::Quaternion<float> qua = q.ToQuaternion();
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qx(qua.x());
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qy(qua.y());
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qz(qua.z());
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qw(qua.w());

    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_x(0.5);
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_y(0.5);
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_linear_velocity()->set_z(0.1);

    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_x(2.3);
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_y(3.2);
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_linear_acceleration()->set_z(4.1);

    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_x(1.5);
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_y(2.6);
    localization_msg_->mutable_loc_pose()->mutable_pose()->mutable_angular_velocity()->set_z(3.7);

    localization_msg_->mutable_loc_pose()->mutable_local_pose()->CopyFrom(
        localization_msg_->mutable_loc_pose()->pose());

    // Init chassis msg
    chassis_msg_ = std::make_shared<pnc::chassis::Chassis>();
    chassis_msg_->mutable_header()->set_timestamp_sec(curr_time);
    chassis_msg_->mutable_header()->set_module_name("chassis");
    chassis_msg_->mutable_header()->set_sequence_num(1);
    chassis_msg_->set_engine_started(true);
    chassis_msg_->set_speed_mps(5.6);
    chassis_msg_->set_chassis_lon_acc(0.2334455);
    chassis_msg_->set_odometer_m(100.2);
    chassis_msg_->set_throttle_percentage(5.2);
    chassis_msg_->set_brake_percentage(0.0);
    chassis_msg_->set_steering_percentage(23.33);
    chassis_msg_->set_steering_torque_nm(2.56);
    chassis_msg_->set_steering_angle_spd(100.0);
    chassis_msg_->set_parking_brake(false);
    chassis_msg_->set_driving_mode(pnc::chassis::COMPLETE_AUTO_DRIVE);
    chassis_msg_->set_gear_location(pnc::chassis::GEAR_DRIVE);
    chassis_msg_->set_error_code(pnc::chassis::ErrorCode::NO_ERROR);

    // Init input stream
    control_message_manager_ = std::make_unique<ControlMessageManager>(control_conf_.max_control_interval_sec());
    control_message_manager_->SaveLocalizationMessage(localization_msg_);
    control_message_manager_->SaveChassisMessage(chassis_msg_);
}

void VirtualSensorsTest::InitFilters() {
    // kalman filter
    // Initial state
    Eigen::Matrix<double, 6, 1> x;
    x.setZero();
    x(0, 0) = localization_msg_->loc_pose().pose().position().x();
    x(1, 0) = localization_msg_->loc_pose().pose().position().y();
    x(2, 0) = localization_msg_->loc_pose().pose().position().z();
    x(3, 0) = localization_msg_->loc_pose().pose().linear_velocity().x();
    x(4, 0) = localization_msg_->loc_pose().pose().linear_velocity().y();
    x(5, 0) = localization_msg_->loc_pose().pose().linear_velocity().z();

    Eigen::Matrix<double, 6, 1> u;
    u.setZero();

    // Initial state belief covariance
    Eigen::Matrix<double, 6, 6> P;
    P.setZero();
    P(0, 0) = 1.0;
    P(1, 1) = 1.0;
    P(2, 2) = 1.0;
    P(3, 3) = 100.0;
    P(4, 4) = 100.0;
    P(5, 5) = 100.0;

    // Transition matrix
    Eigen::Matrix<double, 6, 6> F;
    F.setIdentity();

    // Transition noise covariance
    Eigen::Matrix<double, 6, 6> Q;
    Q.setIdentity();

    // Observation matrix
    Eigen::Matrix<double, 6, 6> H;
    H.setIdentity();

    // Observation noise covariance
    Eigen::Matrix<double, 6, 6> R;
    R.setIdentity();

    // Control matrix
    Eigen::Matrix<double, 6, 3> B;
    B.setZero();
    B(0, 0) = 0.5;
    B(1, 1) = 0.5;
    B(2, 2) = 0.5;
    B(3, 0) = 1.0;
    B(4, 1) = 1.0;
    B(5, 2) = 1.0;

    kalman_filter_.SetStateEstimate(x, P);
    kalman_filter_.SetTransitionMatrix(F);
    kalman_filter_.SetTransitionNoise(Q);
    kalman_filter_.SetObservationMatrix(H);
    kalman_filter_.SetObservationNoise(R);
    kalman_filter_.SetControlMatrix(B);

    previous_timestamp_sec_ = localization_msg_->loc_pose().measurement_time();
}  // update transition matrix

PointENU VirtualSensorsTest::GetFilteredPosition(const std::shared_ptr<LocalizationEstimate>& localization) {
    const auto& position = localization->loc_pose().pose().position();
    const auto& speed3d = localization->loc_pose().pose().linear_velocity();
    const auto& acceleration_ego_3d = localization->loc_pose().pose().linear_acceleration();
    const auto& orientation = localization->loc_pose().pose().orientation();
    EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
    double current_timestamp_sec = localization->loc_pose().measurement_time();
    double delta_t = current_timestamp_sec - previous_timestamp_sec_;
    previous_timestamp_sec_ = current_timestamp_sec;

    // Update control state
    Eigen::Matrix<double, 3, 1> u_in;
    u_in << (acceleration_ego_3d.x() * std::cos(q.yaw()) + acceleration_ego_3d.y() * std::sin(q.yaw())),
        (acceleration_ego_3d.x() * std::sin(q.yaw()) + acceleration_ego_3d.y() * std::cos(q.yaw())),
        acceleration_ego_3d.z();

    // update transition matrix
    Eigen::Matrix<double, 6, 6> F_in = kalman_filter_.GetTransitionMatrix();
    F_in(0, 3) = delta_t;
    F_in(1, 4) = delta_t;
    F_in(2, 5) = delta_t;
    kalman_filter_.SetTransitionMatrix(F_in);

    // update control matrix
    Eigen::Matrix<double, 6, 3> B_in = kalman_filter_.GetControlMatrix();
    B_in(0, 0) = B_in(0, 0) * std::pow(delta_t, 2);
    B_in(1, 1) = B_in(1, 0) * std::pow(delta_t, 2);
    B_in(2, 2) = B_in(2, 0) * std::pow(delta_t, 2);
    kalman_filter_.SetControlMatrix(B_in);

    // Update position
    kalman_filter_.Predict(u_in);

    Eigen::Matrix<double, 6, 1> z_in;
    z_in << position.x(), position.y(), position.z(), speed3d.x(), speed3d.y(), speed3d.z();

    kalman_filter_.Correct(z_in);

    Eigen::Matrix<double, 6, 1> x_out = kalman_filter_.GetStateEstimate();

    PointENU point_enu;
    point_enu.set_x(x_out(0, 0));
    point_enu.set_y(x_out(1, 0));
    point_enu.set_z(x_out(2, 0));

    Point3D velocity_component;
    velocity_component.set_x(x_out(3, 0));
    velocity_component.set_y(x_out(4, 0));
    velocity_component.set_z(x_out(5, 0));

    return point_enu;
}

TEST_F(VirtualSensorsTest, SenseViewTest) {
    virtual_sensors_ = std::make_unique<VirtualSensors>();
    EXPECT_TRUE(virtual_sensors_->Init(control_conf_));

    InitInputStream();

    ControlInputStream control_input_stream;
    ControlCommandStream command_stream;

    const InputStream input_stream = control_message_manager_->GetOnboardMessage();
    pnc::common::VehicleState sense_view;
    virtual_sensors_->Update(input_stream, &command_stream.control_debug);

    // raw state
    control_input_stream.vehicle_state = virtual_sensors_->GetSenseView();
    EXPECT_EQ(localization_msg_->loc_pose().measurement_time(), control_input_stream.vehicle_state.timestamp_sec());
    EXPECT_EQ(localization_msg_->loc_pose().pose().position().x(), control_input_stream.vehicle_state.x());
    EXPECT_EQ(localization_msg_->loc_pose().pose().position().y(), control_input_stream.vehicle_state.y());
    EXPECT_EQ(localization_msg_->loc_pose().pose().position().z(), control_input_stream.vehicle_state.z());

    const auto& orientation = localization_msg_->loc_pose().pose().orientation();
    EulerAnglesZXYf q(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
    const double heading =
        jiduauto::pnc::QuaternionToHeading(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
    EXPECT_EQ(q.roll(), control_input_stream.vehicle_state.roll());
    EXPECT_EQ(q.pitch(), control_input_stream.vehicle_state.pitch());
    EXPECT_EQ(q.yaw(), control_input_stream.vehicle_state.yaw());
    EXPECT_NEAR(heading, control_input_stream.vehicle_state.heading(), 0.001);

    EXPECT_EQ(localization_msg_->loc_pose().pose().angular_velocity().z(),
              control_input_stream.vehicle_state.angular_velocity());
    EXPECT_EQ(pnc::chassis::GEAR_DRIVE, control_input_stream.vehicle_state.gear());
    EXPECT_EQ(chassis_msg_->steering_percentage(), control_input_stream.vehicle_state.steering_percentage());
    EXPECT_EQ(
        chassis_msg_->steering_percentage() *
            control_conf_.control_preprocess_conf().virtual_sensors_conf().revise_percentage_max_steer_angle_deg() /
            100.0,
        control_input_stream.vehicle_state.steering_angle());
    EXPECT_EQ(chassis_msg_->steering_angle_spd(), control_input_stream.vehicle_state.steering_angle_rate());
    EXPECT_EQ(chassis_msg_->steering_torque_nm(), control_input_stream.vehicle_state.steering_torque());
    EXPECT_EQ(chassis_msg_->speed_mps(), control_input_stream.vehicle_state.linear_velocity());
    const auto& speed3d = localization_msg_->loc_pose().pose().linear_velocity();
    EXPECT_NEAR(speed3d.x(), control_input_stream.vehicle_state.velocity_component_enu().x(), 0.1);
    EXPECT_NEAR(speed3d.y(), control_input_stream.vehicle_state.velocity_component_enu().y(), 0.1);
    EXPECT_NEAR(speed3d.z(), control_input_stream.vehicle_state.velocity_component_enu().z(), 0.1);
    EXPECT_NEAR(std::hypot(speed3d.x(), speed3d.y()), control_input_stream.vehicle_state.localization_velocity(), 0.1);
    EXPECT_EQ(chassis_msg_->chassis_lon_acc(), control_input_stream.vehicle_state.linear_acceleration());
    EXPECT_EQ(localization_msg_->loc_pose().pose().angular_velocity().z() /
                  std::hypot(localization_msg_->loc_pose().pose().linear_velocity().x(),
                             localization_msg_->loc_pose().pose().linear_velocity().y()),
              control_input_stream.vehicle_state.curvature());
    EXPECT_EQ(pnc::chassis::COMPLETE_AUTO_DRIVE, control_input_stream.vehicle_state.driving_mode());
}

}  // namespace control
}  // namespace jiduauto
