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

#include "control/src/common/mrac_controller.h"

#include <gtest/gtest.h>

namespace jiduauto {
namespace control {

// test the input-output of the model reference adaptive controller
TEST(MracController, MracControl) {
    pnc::control::MracConf mrac_conf;
    mrac_conf.set_mrac_model_order(1);
    mrac_conf.set_reference_time_constant(0.01);
    mrac_conf.set_reference_natural_frequency(10);
    mrac_conf.set_reference_damping_ratio(0.9);
    mrac_conf.mutable_adaption_state_gain()->Add(1.0);
    mrac_conf.set_adaption_desired_gain(1.0);
    mrac_conf.set_adaption_nonlinear_gain(1.0);
    mrac_conf.mutable_adaption_matrix_p()->Add(1.0);
    mrac_conf.set_mrac_saturation_level(1.0);
    mrac_conf.mutable_anti_windup_compensation_gain()->Add(0.0);
    // mrac_conf.set_clamping_time_constant(0.08);
    pnc::control::LatencyParam steering_latency_param;
    steering_latency_param.set_dead_time(0.0);
    steering_latency_param.set_rise_time(0.0);
    steering_latency_param.set_peak_time(0.0);
    steering_latency_param.set_settling_time(0.0);

    double dt = 0.01;
    MracController mrac_controller;

    mrac_controller.Init(mrac_conf, steering_latency_param, dt);

    mrac_controller.SetInitialInputAdaptionGain(0.0);

    Eigen::MatrixXd state = Eigen::MatrixXd::Zero(1, 1);
    double limit = 100.0;
    double rate_limit = 100.0 / dt;
    state(0, 0) = 6.0;

    EXPECT_NEAR(mrac_controller.Control(18.0, state, limit, rate_limit), 0.0, 1e-6);

    mrac_controller.Reset();
    mrac_controller.SetInitialInputAdaptionGain(0.0);
    state(0, 0) = 10.0;
    EXPECT_NEAR(mrac_controller.Control(18.0, state, limit, rate_limit), -8.48, 1e-6);
    EXPECT_NEAR(mrac_controller.CurrentReferenceState()(0, 0), 6.0, 1e-6);
    EXPECT_NEAR(mrac_controller.CurrentStateAdaptionGain()(0, 0), -0.2, 1e-6);
    EXPECT_NEAR(mrac_controller.CurrentInputAdaptionGain()(0, 0), -0.36, 1e-6);
    EXPECT_NEAR(mrac_controller.Control(18.0, state, limit, rate_limit), -8.48, 1e-6);
    EXPECT_NEAR(mrac_controller.CurrentReferenceState()(0, 0), 14.0, 1e-6);
    EXPECT_NEAR(mrac_controller.CurrentStateAdaptionGain()(0, 0), -0.2, 1e-6);
    EXPECT_NEAR(mrac_controller.CurrentInputAdaptionGain()(0, 0), -0.36, 1e-6);
    mrac_controller.Reset();
    mrac_controller.SetInitialInputAdaptionGain(0.0);
    state(0, 0) = -10.0;
    double control_value = mrac_controller.Control(-18.0, state, limit, rate_limit);
    EXPECT_NEAR(control_value, 8.48, 1e-6);
    // test the bounded conditions of the system output
    dt = 0.01;
    mrac_controller.Init(mrac_conf, steering_latency_param, dt);
    mrac_controller.SetInitialInputAdaptionGain(0.0);
    state(0, 0) = 10.0;
    EXPECT_NEAR(mrac_controller.Control(18.0, state, 100.0, 1.0 / dt), -1.0, 1e-6);
    EXPECT_EQ(mrac_controller.ReferenceSaturationStatus(), 2);
    EXPECT_EQ(mrac_controller.ControlSaturationStatus(), -2);
    mrac_controller.Reset();
    mrac_controller.SetInitialInputAdaptionGain(0.0);
    state(0, 0) = 10.0;
    EXPECT_NEAR(mrac_controller.Control(18.0, state, 10.0, 100.0 / dt), -8.48, 1e-6);
    EXPECT_NEAR(mrac_controller.CurrentReferenceState()(0, 0), 6.0, 1e-6);
    EXPECT_EQ(mrac_controller.ReferenceSaturationStatus(), 0);
    EXPECT_EQ(mrac_controller.ControlSaturationStatus(), 0);
    EXPECT_NEAR(mrac_controller.Control(18.0, state, 10.0, 100.0 / dt), -10.0, 1e-6);
    EXPECT_NEAR(mrac_controller.CurrentReferenceState()(0, 0), 10.0, 1e-6);
    EXPECT_EQ(mrac_controller.ReferenceSaturationStatus(), 1);
    EXPECT_EQ(mrac_controller.ControlSaturationStatus(), -1);
}

// test the higher-order input-output of the model reference adaptive controller
TEST(MracController, HighOrderMracControl) {
    // Re-design the 2nd order model parameters
    pnc::control::MracConf mrac_conf;
    mrac_conf.set_mrac_model_order(1);
    mrac_conf.set_reference_time_constant(0.01);
    mrac_conf.set_reference_natural_frequency(10);
    mrac_conf.set_reference_damping_ratio(0.9);
    mrac_conf.mutable_adaption_state_gain()->Add(1.0);
    mrac_conf.set_adaption_desired_gain(1.0);
    mrac_conf.set_adaption_nonlinear_gain(1.0);
    mrac_conf.mutable_adaption_matrix_p()->Add(1.0);
    mrac_conf.set_mrac_saturation_level(1.0);
    mrac_conf.mutable_anti_windup_compensation_gain()->Add(0.0);
    // mrac_conf.set_clamping_time_constant(0.08);
    pnc::control::LatencyParam steering_latency_param;
    steering_latency_param.set_dead_time(0.0);
    steering_latency_param.set_rise_time(0.0);
    steering_latency_param.set_peak_time(0.0);
    steering_latency_param.set_settling_time(0.0);

    double dt = 0.01;
    Eigen::MatrixXd state = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd reference_init = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd action_init = Eigen::MatrixXd::Zero(2, 1);
    double command_init = 10.0;

    MracController mrac_controller;
    mrac_conf.set_mrac_model_order(2);
    mrac_conf.clear_adaption_state_gain();
    mrac_conf.add_adaption_state_gain(0.1);
    mrac_conf.add_adaption_state_gain(0.1);
    mrac_conf.set_adaption_desired_gain(0.1);
    mrac_conf.clear_adaption_matrix_p();
    mrac_conf.add_adaption_matrix_p(10.0);
    mrac_conf.add_adaption_matrix_p(0.1);
    mrac_conf.add_adaption_matrix_p(0.1);
    mrac_conf.add_adaption_matrix_p(0.1);
    // Fill the initial reference / actuation states and command
    mrac_controller.Init(mrac_conf, steering_latency_param, dt);
    double limit = 100.0;
    double rate_limit = 100.0 / dt;
    state(0, 0) = 10.0;
    state(1, 0) = 0.0;
    reference_init(0, 0) = 10.0;
    reference_init(1, 0) = 0.0;
    action_init(0, 0) = 10.0;
    action_init(1, 0) = 0.0;
    mrac_controller.SetInitialReferenceState(reference_init);
    mrac_controller.SetInitialActionState(action_init);
    mrac_controller.SetInitialCommand(command_init);
    mrac_controller.SetInitialInputAdaptionGain(0.0);
    // 2nd order system test
    EXPECT_NEAR(mrac_controller.Control(15.0, state, limit, rate_limit), 3.738, 1e-3);
    EXPECT_NEAR(mrac_controller.Control(15.0, state, limit, rate_limit), 18.08, 1e-2);
    // Reference system convergence test
    const int settling_cycle = static_cast<int>(
        std::round(4.6 / (mrac_conf.reference_natural_frequency() * mrac_conf.reference_damping_ratio()) / dt));
    for (int i = 0; i < settling_cycle; ++i) {
        mrac_controller.Control(15.0, state, limit, rate_limit);
    }
    EXPECT_NEAR(mrac_controller.CurrentReferenceState()(0, 0), 15.0, 0.05);
}

// test the judgement of the symmetric positive definite solution of the
// Lyapunov equation
TEST(MracController, CheckLyapunovPD) {
    pnc::control::MracConf mrac_conf;
    mrac_conf.set_mrac_model_order(1);
    mrac_conf.set_reference_time_constant(0.01);
    mrac_conf.set_reference_natural_frequency(10);
    mrac_conf.set_reference_damping_ratio(0.9);
    mrac_conf.mutable_adaption_state_gain()->Add(1.0);
    mrac_conf.set_adaption_desired_gain(1.0);
    mrac_conf.set_adaption_nonlinear_gain(1.0);
    mrac_conf.mutable_adaption_matrix_p()->Add(1.0);
    mrac_conf.set_mrac_saturation_level(1.0);
    mrac_conf.mutable_anti_windup_compensation_gain()->Add(0.0);
    // mrac_conf.set_clamping_time_constant(0.08);
    pnc::control::LatencyParam steering_latency_param;
    steering_latency_param.set_dead_time(0.0);
    steering_latency_param.set_rise_time(0.0);
    steering_latency_param.set_peak_time(0.0);
    steering_latency_param.set_settling_time(0.0);

    double dt = 0.01;
    MracController mrac_controller;
    mrac_controller.Init(mrac_conf, steering_latency_param, dt);
    // test on 1st order adaption dynamics
    Eigen::MatrixXd matrix_a = Eigen::MatrixXd::Zero(1, 1);
    Eigen::MatrixXd matrix_p = Eigen::MatrixXd::Zero(1, 1);
    matrix_a(0, 0) = -100.0;
    matrix_p(0, 0) = 1.0;
    EXPECT_EQ(mrac_controller.CheckLyapunovPD(matrix_a, matrix_p), true);
    matrix_a(0, 0) = -100.0;
    matrix_p(0, 0) = -1.0;
    EXPECT_EQ(mrac_controller.CheckLyapunovPD(matrix_a, matrix_p), false);
    // test on 2nd order adaption dynamics
    matrix_a = Eigen::MatrixXd::Zero(2, 2);
    matrix_p = Eigen::MatrixXd::Zero(2, 2);
    matrix_a(0, 1) = 1.0;
    matrix_a(1, 0) = -100.0;
    matrix_a(1, 1) = -18.0;
    matrix_p(0, 0) = 1.0;
    matrix_p(1, 1) = 1.0;
    EXPECT_EQ(mrac_controller.CheckLyapunovPD(matrix_a, matrix_p), false);
    matrix_p(0, 0) = 1.0;
    matrix_p(1, 1) = 0.01;
    EXPECT_EQ(mrac_controller.CheckLyapunovPD(matrix_a, matrix_p), false);
    matrix_p(0, 0) = 10.0;
    matrix_p(0, 1) = 0.1;
    matrix_p(1, 0) = 0.1;
    matrix_p(1, 1) = 0.1;
    EXPECT_EQ(mrac_controller.CheckLyapunovPD(matrix_a, matrix_p), true);
}

}  // namespace control
}  // namespace jiduauto
