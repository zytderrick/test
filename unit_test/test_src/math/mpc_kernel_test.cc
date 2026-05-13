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

#include "control/src/math/mpc_kernel.h"

#include <gtest/gtest.h>

#include <chrono>
#include <ctime>
#include <limits>
#include <utility>
#include <vector>

#include "control/src/common/control_utility.h"
#include "control/src/math/control_math.h"
#include "control/src/math/mpc_solver.h"
#include "control/src/math/mpc_solver_osqp.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/utility/time_utility.h"

namespace jiduauto {
namespace control {
using jiduauto::pnc::util::TimeUtility;

TEST(MPCKernelTest, ComputationTimeTest) {
    MPCModel mpc_model(4, 2, 4, 3, 3, 0.02);

    const double max = std::numeric_limits<double>::max();

    mpc_model.matrix_a = Eigen::MatrixXd::Zero(mpc_model.state_num, mpc_model.state_num);
    mpc_model.matrix_a << 5., 6., 7., 8., 7., 8., 7., 8., 9., 10., 7., 8., 11., 4., 7., 8.;

    mpc_model.matrix_b = Eigen::MatrixXd::Zero(mpc_model.state_num, mpc_model.input_num);
    mpc_model.matrix_b << 2., 3, 2., 7, 2, 9, 3, 8;

    mpc_model.matrix_c = Eigen::MatrixXd::Zero(mpc_model.state_num, 1);
    mpc_model.matrix_d = Eigen::MatrixXd::Identity(mpc_model.output_num, mpc_model.state_num);

    mpc_model.matrix_q = Eigen::MatrixXd::Zero(mpc_model.output_num, mpc_model.output_num);
    mpc_model.matrix_q << 10., 0, 0, 0, 0, 10., 0, 0, 0, 0, 10.0, 0, 0, 0, 0, 10.0;
    mpc_model.matrix_qp = Eigen::MatrixXd::Zero(mpc_model.output_num, mpc_model.output_num);
    mpc_model.matrix_qp = mpc_model.matrix_q;

    mpc_model.matrix_r1 = Eigen::MatrixXd::Zero(mpc_model.input_num, mpc_model.input_num);
    mpc_model.matrix_r1 << 0.1, 0, 0, 0.1;
    mpc_model.matrix_r2 = Eigen::MatrixXd::Zero(mpc_model.input_num, mpc_model.input_num);
    mpc_model.matrix_r2 << 0.1, 0, 0, 0.1;

    mpc_model.u_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.u_lower_boundary << 9.6 - 10.5916, 9.6 - 10.5916;
    mpc_model.u_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.u_upper_boundary << 13 - 10.5916, 13 - 10.5916;
    mpc_model.du_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.du_lower_boundary << -0.5, -0.5;
    mpc_model.du_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.du_upper_boundary << 0.5, 0.5;

    mpc_model.y_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_lower_boundary << -M_PI / 6, -M_PI / 6, -1 * max, -1 * max;
    mpc_model.y_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_upper_boundary << M_PI / 6, M_PI / 6, max, max;
    mpc_model.y_lower_terminal_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_lower_terminal_boundary << -M_PI / 6, -M_PI / 6, -1 * max, -1 * max;
    mpc_model.y_upper_terminal_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_upper_terminal_boundary << M_PI / 6, M_PI / 6, max, max;

    mpc_model.state_init = Eigen::MatrixXd::Zero(mpc_model.state_num, 1);
    mpc_model.state_init << 0, 0, 0, 0;

    mpc_model.previous_input = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.previous_input << 11.0 - 10.5916, 11.0 - 10.5916;
    mpc_model.reference.clear();
    Eigen::MatrixXd reference_state(mpc_model.output_num, 1);
    reference_state << 0, 0, 1, 0;
    for (int i = 0; i < mpc_model.horizon_prediction; ++i) {
        mpc_model.reference.push_back(reference_state);
    }
    std::vector<Eigen::MatrixXd> control_cmd;
    auto start_time_osqp = std::chrono::system_clock::now();
    MPCKernel kernel;

    EXPECT_TRUE(kernel.Solve(mpc_model, &control_cmd));
    auto end_time_osqp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff_OSQP = end_time_osqp - start_time_osqp;
    LOG_INFO("time: %.4f", diff_OSQP);
    LOG_INFO("size: %d", control_cmd.size());
    for (int i = 0; i < control_cmd.size(); ++i) {
        LOG_INFO("index: %d", i);
        for (int j = 0; j < control_cmd[i].rows(); ++j) {
            std::string str = "";
            for (int k = 0; k < control_cmd[i].cols(); ++k) {
                str += std::to_string(control_cmd[i](j, k)) + ", ";
            }
            LOG_INFO("%s", str.c_str());
        }
    }
}

TEST(MPCKernelTest, OSQPTEST) {
    const int states = 6;
    const int controls = 2;
    const int horizon = 10;
    const int max_iter = 100;
    const double eps = 0.001;
    const double max = std::numeric_limits<double>::max();

    Eigen::MatrixXd A(states, states);
    A << 1.000000, 0.016945, 0.017721, -0.000227, 0.000000, 0.000000, 0.000000, 0.694460, 1.772135, -0.022746, 0.000000,
        0.000000, 0.000000, -0.000169, 1.000983, 0.016905, 0.000000, 0.000000, 0.000000, -0.016944, 0.098272, 0.690494,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.020000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd B(states, controls);
    B << 0.000000, 0.000000, 1.046762, 0.000000, 0.000000, 0.000000, 0.773380, 0.000000, 0.000000, 0.000000, 0.000000,
        -0.020000;

    Eigen::MatrixXd Q(states, states);
    Q << 3.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 35.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 50.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 10.000000;

    Eigen::MatrixXd R(controls, controls);
    R << 3.250000, 0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd lower_control_bound(controls, 1);
    lower_control_bound << -0.7082, -4.0;

    Eigen::MatrixXd upper_control_bound(controls, 1);
    upper_control_bound << 0.7082, 4.0;

    Eigen::MatrixXd lower_state_bound(states, 1);
    lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max, -1.0 * max;

    Eigen::MatrixXd upper_state_bound(states, 1);
    upper_state_bound << max, max, M_PI, max, max, max;

    Eigen::MatrixXd initial_state(states, 1);
    initial_state << 0.3, 0.02, -0.1, 0.03, 0.2, 1.0;
    std::vector<double> control_cmd(controls, 0);

    Eigen::MatrixXd reference_state = Eigen::MatrixXd::Zero(states, 1);
    std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

    // OSQP
    auto start_time_osqp = TimeUtility::GetCurrentTimesecond();
    MpcSolver mpc_osqp_solver(A, B, Q, R, initial_state, lower_control_bound, upper_control_bound, lower_state_bound,
                              upper_state_bound, reference_state, max_iter, horizon, eps);
    EXPECT_TRUE(mpc_osqp_solver.Execute(&control_cmd));
    auto end_time_osqp = TimeUtility::GetCurrentTimesecond();
    LOG_INFO("osqp time: %.4f", end_time_osqp - start_time_osqp);
    LOG_INFO("controls: %.4f, %.4f", control_cmd.at(0), control_cmd.at(1));
}

TEST(MPCKernelTest, QPOASETEST) {
    const int states = 6;
    const int controls = 2;
    const int horizon = 10;
    const double max = std::numeric_limits<double>::max();
    MPCModel mpc_model(states, controls, states, horizon, horizon, 0.02);

    mpc_model.matrix_a = Eigen::MatrixXd::Zero(mpc_model.state_num, mpc_model.state_num);
    mpc_model.matrix_a << 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -18.047620, 104.676198,
        -2.635134, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, -1.183170,
        6.862385, -18.393129, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000;

    mpc_model.matrix_b = Eigen::MatrixXd::Zero(mpc_model.state_num, mpc_model.input_num);
    mpc_model.matrix_b << 0.000000, 0.000000, 52.338099, 0.000000, 0.000000, 0.000000, 38.668997, 0.000000, 0.000000,
        0.000000, 0.000000, -1.000000;

    mpc_model.matrix_c = Eigen::MatrixXd::Zero(mpc_model.state_num, 1);
    mpc_model.matrix_d = Eigen::MatrixXd::Identity(mpc_model.output_num, mpc_model.state_num);

    mpc_model.matrix_q = Eigen::MatrixXd::Zero(mpc_model.output_num, mpc_model.output_num);
    mpc_model.matrix_q << 3.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 35.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 50.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 10.000000;
    mpc_model.matrix_qp = Eigen::MatrixXd::Zero(mpc_model.output_num, mpc_model.output_num);
    mpc_model.matrix_qp = mpc_model.matrix_q;

    mpc_model.matrix_r1 = Eigen::MatrixXd::Zero(mpc_model.input_num, mpc_model.input_num);
    mpc_model.matrix_r1 << 3.250000, 0.000000, 0.000000, 1.000000;
    mpc_model.matrix_r2 = Eigen::MatrixXd::Zero(mpc_model.input_num, mpc_model.input_num);
    mpc_model.matrix_r2 << 3.250000, 0.000000, 0.000000, 1.000000;

    mpc_model.u_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.u_lower_boundary << -0.7082, -4.0;
    mpc_model.u_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.u_upper_boundary << 0.7082, 4.0;
    mpc_model.du_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.du_lower_boundary << -0.5, -4.0;
    mpc_model.du_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.du_upper_boundary << 0.5, 4.0;

    mpc_model.y_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_lower_boundary << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max, -1.0 * max;
    mpc_model.y_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_upper_boundary << max, max, M_PI, max, max, max;

    mpc_model.y_lower_terminal_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_lower_terminal_boundary << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max, -1.0 * max;
    mpc_model.y_upper_terminal_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_upper_terminal_boundary << max, max, M_PI, max, max, max;

    mpc_model.state_init = Eigen::MatrixXd::Zero(mpc_model.state_num, 1);
    mpc_model.state_init << 0.3, 0.02, -0.1, 0.03, 0.2, 1.0;

    mpc_model.previous_input = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);

    mpc_model.reference.clear();
    Eigen::MatrixXd reference_state = Eigen::MatrixXd::Zero(states, 1);
    for (int i = 0; i < mpc_model.horizon_prediction; ++i) {
        mpc_model.reference.push_back(reference_state);
    }

    std::vector<Eigen::MatrixXd> control_cmd;

    // OSQP
    auto start_time_osqp = TimeUtility::GetCurrentTimesecond();
    MPCKernel kernel;
    EXPECT_TRUE(kernel.Solve(mpc_model, &control_cmd));

    auto end_time_osqp = TimeUtility::GetCurrentTimesecond();
    LOG_INFO("qpoase time: %.4f", end_time_osqp - start_time_osqp);
    LOG_INFO("controls: %.4f, %.4f", control_cmd[0](0, 0), control_cmd[0](1, 0));
}

TEST(MPCKernelTest, MPCSolverOsqpTest) {
    const int states = 6;
    const int controls = 2;
    const int horizon = 10;
    const int max_iter = 100;
    const double eps = 0.001;
    const double max = std::numeric_limits<double>::max();

    Eigen::MatrixXd A(states, states);
    A << 1.000000, 0.016945, 0.017721, -0.000227, 0.000000, 0.000000, 0.000000, 0.694460, 1.772135, -0.022746, 0.000000,
        0.000000, 0.000000, -0.000169, 1.000983, 0.016905, 0.000000, 0.000000, 0.000000, -0.016944, 0.098272, 0.690494,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.020000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd B(states, controls);
    B << 0.000000, 0.000000, 1.046762, 0.000000, 0.000000, 0.000000, 0.773380, 0.000000, 0.000000, 0.000000, 0.000000,
        -0.020000;

    Eigen::MatrixXd Q(states, states);
    Q << 3.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 35.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 50.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 10.000000;

    Eigen::MatrixXd R(controls, controls);
    R << 3.250000, 0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd lower_control_bound(controls, 1);
    lower_control_bound << -0.7082, -4.0;

    Eigen::MatrixXd upper_control_bound(controls, 1);
    upper_control_bound << 0.7082, 4.0;

    Eigen::MatrixXd lower_state_bound(states, 1);
    lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max, -1.0 * max;

    Eigen::MatrixXd upper_state_bound(states, 1);
    upper_state_bound << max, max, M_PI, max, max, max;

    Eigen::MatrixXd initial_state(states, 1);
    initial_state << 0.3, 0.02, -0.1, 0.03, 0.2, 1.0;
    std::vector<double> control_cmd(controls, 0);

    Eigen::MatrixXd reference_state = Eigen::MatrixXd::Zero(states, 1);
    std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

    // OSQP
    auto start_time_osqp = TimeUtility::GetCurrentTimesecond();
    MpcSolverOsqp mpc_osqp_solver(A, B, Q, R, initial_state, lower_control_bound, upper_control_bound,
                                  lower_state_bound, upper_state_bound, reference_state, max_iter, horizon, eps);
    EXPECT_TRUE(mpc_osqp_solver.Execute(&control_cmd));
    auto end_time_osqp = TimeUtility::GetCurrentTimesecond();
    LOG_INFO("osqp time: %.4f", end_time_osqp - start_time_osqp);
    LOG_INFO("controls: %.4f, %.4f", control_cmd.at(0), control_cmd.at(1));
}

// test loop, evaluate time cost
TEST(MPCKernelTest, OSQPTESTLOOP) {
    const int states = 6;
    const int controls = 2;
    const int horizon = 10;
    const int max_iter = 100;
    const double eps = 0.001;
    const double max = std::numeric_limits<double>::max();

    Eigen::MatrixXd A(states, states);
    A << 1.000000, 0.016945, 0.017721, -0.000227, 0.000000, 0.000000, 0.000000, 0.694460, 1.772135, -0.022746, 0.000000,
        0.000000, 0.000000, -0.000169, 1.000983, 0.016905, 0.000000, 0.000000, 0.000000, -0.016944, 0.098272, 0.690494,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.020000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd B(states, controls);
    B << 0.000000, 0.000000, 1.046762, 0.000000, 0.000000, 0.000000, 0.773380, 0.000000, 0.000000, 0.000000, 0.000000,
        -0.020000;

    Eigen::MatrixXd Q(states, states);
    Q << 3.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 35.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 50.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 10.000000;

    Eigen::MatrixXd R(controls, controls);
    R << 3.250000, 0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd lower_control_bound(controls, 1);
    lower_control_bound << -0.7082, -4.0;

    Eigen::MatrixXd upper_control_bound(controls, 1);
    upper_control_bound << 0.7082, 4.0;

    Eigen::MatrixXd lower_state_bound(states, 1);
    lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max, -1.0 * max;

    Eigen::MatrixXd upper_state_bound(states, 1);
    upper_state_bound << max, max, M_PI, max, max, max;

    Eigen::MatrixXd initial_state(states, 1);
    initial_state << 0.3, 0.02, -0.1, 0.03, 0.2, 1.0;
    Eigen::MatrixXd initial_state2(states, 1);
    initial_state << -0.3, +0.02, 0.1, -0.03, -0.2, -1.0;
    Eigen::MatrixXd initial_state3 = initial_state;

    std::vector<double> control_cmd(controls, 0);

    Eigen::MatrixXd reference_state = Eigen::MatrixXd::Zero(states, 1);
    std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

    // OSQP
    auto start_time_osqp = TimeUtility::GetCurrentTimesecond();
    for (int i = 0; i < 1000; ++i) {
        if (i % 2 == 0) {
            initial_state = initial_state2;
        } else {
            initial_state = initial_state3;
        }
        MpcSolver mpc_osqp_solver(A, B, Q, R, initial_state, lower_control_bound, upper_control_bound,
                                  lower_state_bound, upper_state_bound, reference_state, max_iter, horizon, eps);
        mpc_osqp_solver.Execute(&control_cmd);
    }

    auto end_time_osqp = TimeUtility::GetCurrentTimesecond();
    LOG_INFO("osqp 1000 times: %.4f", end_time_osqp - start_time_osqp);
}

TEST(MPCKernelTest, QPOASETESTLOOP) {
    const int states = 6;
    const int controls = 2;
    const int horizon = 10;
    const double max = std::numeric_limits<double>::max();
    MPCModel mpc_model(states, controls, states, horizon, horizon, 0.02);

    mpc_model.matrix_a = Eigen::MatrixXd::Zero(mpc_model.state_num, mpc_model.state_num);
    mpc_model.matrix_a << 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -18.047620, 104.676198,
        -2.635134, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, -1.183170,
        6.862385, -18.393129, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000;

    mpc_model.matrix_b = Eigen::MatrixXd::Zero(mpc_model.state_num, mpc_model.input_num);
    mpc_model.matrix_b << 0.000000, 0.000000, 52.338099, 0.000000, 0.000000, 0.000000, 38.668997, 0.000000, 0.000000,
        0.000000, 0.000000, -1.000000;

    mpc_model.matrix_c = Eigen::MatrixXd::Zero(mpc_model.state_num, 1);
    mpc_model.matrix_d = Eigen::MatrixXd::Identity(mpc_model.output_num, mpc_model.state_num);

    mpc_model.matrix_q = Eigen::MatrixXd::Zero(mpc_model.output_num, mpc_model.output_num);
    mpc_model.matrix_q << 3.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 35.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 50.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 10.000000;
    mpc_model.matrix_qp = Eigen::MatrixXd::Zero(mpc_model.output_num, mpc_model.output_num);
    mpc_model.matrix_qp = mpc_model.matrix_q;

    mpc_model.matrix_r1 = Eigen::MatrixXd::Zero(mpc_model.input_num, mpc_model.input_num);
    mpc_model.matrix_r1 << 3.250000, 0.000000, 0.000000, 1.000000;
    mpc_model.matrix_r2 = Eigen::MatrixXd::Zero(mpc_model.input_num, mpc_model.input_num);
    mpc_model.matrix_r2 << 3.250000, 0.000000, 0.000000, 1.000000;

    mpc_model.u_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.u_lower_boundary << -0.7082, -4.0;
    mpc_model.u_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.u_upper_boundary << 0.7082, 4.0;
    mpc_model.du_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.du_lower_boundary << -0.5, -4.0;
    mpc_model.du_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);
    mpc_model.du_upper_boundary << 0.5, 4.0;

    mpc_model.y_lower_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_lower_boundary << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max, -1.0 * max;
    mpc_model.y_upper_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_upper_boundary << max, max, M_PI, max, max, max;

    mpc_model.y_lower_terminal_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_lower_terminal_boundary << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max, -1.0 * max;
    mpc_model.y_upper_terminal_boundary = Eigen::MatrixXd::Zero(mpc_model.output_num, 1);
    mpc_model.y_upper_terminal_boundary << max, max, M_PI, max, max, max;

    mpc_model.state_init = Eigen::MatrixXd::Zero(mpc_model.state_num, 1);
    mpc_model.state_init << 0.3, 0.02, -0.1, 0.03, 0.2, 1.0;

    Eigen::MatrixXd initial_state2(mpc_model.state_num, 1);
    initial_state2 << -0.3, +0.02, 0.1, -0.03, -0.2, -1.0;
    Eigen::MatrixXd initial_state3 = mpc_model.state_init;

    mpc_model.previous_input = Eigen::MatrixXd::Zero(mpc_model.input_num, 1);

    mpc_model.reference.clear();
    Eigen::MatrixXd reference_state = Eigen::MatrixXd::Zero(states, 1);
    for (int i = 0; i < mpc_model.horizon_prediction; ++i) {
        mpc_model.reference.push_back(reference_state);
    }

    std::vector<Eigen::MatrixXd> control_cmd;

    // OSQP
    auto start_time_osqp = TimeUtility::GetCurrentTimesecond();
    MPCKernel kernel;
    for (int i = 0; i < 1000; ++i) {
        if (i % 2 == 0) {
            mpc_model.state_init = initial_state2;
        } else {
            mpc_model.state_init = initial_state3;
        }
        EXPECT_TRUE(kernel.Solve(mpc_model, &control_cmd));
    }

    auto end_time_osqp = TimeUtility::GetCurrentTimesecond();
    LOG_INFO("qpoase 1000 time: %.4f", end_time_osqp - start_time_osqp);
}

TEST(MPCKernelTest, MPCSolverOsqpTestLOOP) {
    const int states = 6;
    const int controls = 2;
    const int horizon = 10;
    const int max_iter = 100;
    const double eps = 0.001;
    const double max = std::numeric_limits<double>::max();

    Eigen::MatrixXd A(states, states);
    A << 1.000000, 0.016945, 0.017721, -0.000227, 0.000000, 0.000000, 0.000000, 0.694460, 1.772135, -0.022746, 0.000000,
        0.000000, 0.000000, -0.000169, 1.000983, 0.016905, 0.000000, 0.000000, 0.000000, -0.016944, 0.098272, 0.690494,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.020000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd B(states, controls);
    B << 0.000000, 0.000000, 1.046762, 0.000000, 0.000000, 0.000000, 0.773380, 0.000000, 0.000000, 0.000000, 0.000000,
        -0.020000;

    Eigen::MatrixXd Q(states, states);
    Q << 3.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 35.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 50.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 10.000000;

    Eigen::MatrixXd R(controls, controls);
    R << 3.250000, 0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd lower_control_bound(controls, 1);
    lower_control_bound << -0.7082, -4.0;

    Eigen::MatrixXd upper_control_bound(controls, 1);
    upper_control_bound << 0.7082, 4.0;

    Eigen::MatrixXd lower_state_bound(states, 1);
    lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max, -1.0 * max, -1.0 * max;

    Eigen::MatrixXd upper_state_bound(states, 1);
    upper_state_bound << max, max, M_PI, max, max, max;

    Eigen::MatrixXd initial_state(states, 1);
    initial_state << 0.3, 0.02, -0.1, 0.03, 0.2, 1.0;
    Eigen::MatrixXd initial_state2(states, 1);
    initial_state << -0.3, +0.02, 0.1, -0.03, -0.2, -1.0;
    Eigen::MatrixXd initial_state3 = initial_state;

    std::vector<double> control_cmd(controls, 0);

    Eigen::MatrixXd reference_state = Eigen::MatrixXd::Zero(states, 1);
    std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

    // OSQP
    auto start_time_osqp = TimeUtility::GetCurrentTimesecond();
    MpcSolverOsqp mpc_osqp_solver(A, B, Q, R, initial_state, lower_control_bound, upper_control_bound,
                                  lower_state_bound, upper_state_bound, reference_state, max_iter, horizon, eps);
    for (int i = 0; i < 1000; ++i) {
        if (i % 2 == 0) {
            initial_state = initial_state2;
        } else {
            initial_state = initial_state3;
        }
        mpc_osqp_solver.ResetMatrix(A, B, Q, R, initial_state, lower_control_bound, upper_control_bound,
                                    lower_state_bound, upper_state_bound, reference_state, max_iter, horizon, eps);
        EXPECT_TRUE(mpc_osqp_solver.Execute(&control_cmd));
    }
    auto end_time_osqp = TimeUtility::GetCurrentTimesecond();
    LOG_INFO("osqp 1000 time: %.4f", end_time_osqp - start_time_osqp);
}

}  // namespace control
}  // namespace jiduauto
