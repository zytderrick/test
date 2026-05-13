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

#include "control/src/math/mpc_solver.h"

#include <gtest/gtest.h>

#include <chrono>
#include <ctime>
#include <limits>
#include <utility>
#include <vector>

#include "pnc_common/src/common/pnc_logger.h"

namespace jiduauto {
namespace control {

TEST(MpcSolverTest, ComputationTimeTest) {
    const int states = 4;
    const int controls = 2;
    const int horizon = 3;
    const int max_iter = 100;
    const double eps = 0.001;
    const double max = std::numeric_limits<double>::max();

    Eigen::MatrixXd A(states, states);
    A << 5., 6., 7., 8., 7., 8., 7., 8., 9., 10., 7., 8., 11., 4., 7., 8.;

    Eigen::MatrixXd B(states, controls);
    B << 2., 3, 2., 7, 2, 9, 3, 8;

    Eigen::MatrixXd Q(states, states);
    Q << 10., 0, 0, 0, 0, 10., 0, 0, 0, 0, 10.0, 0, 0, 0, 0, 10.0;

    Eigen::MatrixXd R(controls, controls);
    R << 0.1, 0, 0, 0.1;

    Eigen::MatrixXd lower_control_bound(controls, 1);
    lower_control_bound << 9.6 - 10.5916, 9.6 - 10.5916;

    Eigen::MatrixXd upper_control_bound(controls, 1);
    upper_control_bound << 13 - 10.5916, 13 - 10.5916;

    Eigen::MatrixXd lower_state_bound(states, 1);
    lower_state_bound << -M_PI / 6, -M_PI / 6, -1 * max, -1 * max;

    Eigen::MatrixXd upper_state_bound(states, 1);
    upper_state_bound << M_PI / 6, M_PI / 6, max, max;

    Eigen::MatrixXd initial_state(states, 1);
    initial_state << 0, 0, 0, 0;
    std::vector<double> control_cmd(controls, 0);

    Eigen::MatrixXd reference_state(states, 1);
    reference_state << 0, 0, 1, 0;
    std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

    // OSQP
    MpcSolver mpc_osqp_solver(A, B, Q, R, initial_state, lower_control_bound, upper_control_bound, lower_state_bound,
                              upper_state_bound, reference_state, max_iter, horizon, eps);
    auto start_time_osqp = std::chrono::system_clock::now();
    mpc_osqp_solver.Execute(&control_cmd);
    auto end_time_osqp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff_OSQP = end_time_osqp - start_time_osqp;
}

TEST(MpcSolverTest, NonFullRankMatrix) {
    const int states = 2;
    const int controls = 1;
    const int horizon = 10;
    const int max_iter = 100;
    const double eps = 0.01;
    const double max = std::numeric_limits<double>::max();

    Eigen::MatrixXd A(states, states);
    A << 0, 1, 0, 0;

    Eigen::MatrixXd B(states, controls);
    B << -1, 1;

    Eigen::MatrixXd Q(states, states);
    Q << 100, 0, 0, 0;

    Eigen::MatrixXd R(controls, controls);
    R << 0;

    Eigen::MatrixXd lower_bound(controls, 1);
    lower_bound << -5;

    Eigen::MatrixXd upper_bound(controls, 1);
    upper_bound << 5;

    Eigen::MatrixXd initial_state(states, 1);
    initial_state << 0, 5;

    Eigen::MatrixXd reference_state(states, 1);
    reference_state << 0, 0;

    Eigen::MatrixXd state_lower_bound(states, 1);
    state_lower_bound << -max, -max;

    Eigen::MatrixXd state_upper_bound(states, 1);
    state_upper_bound << max, max;

    std::vector<double> control_cmd(controls, 0);

    MpcSolver mpc_osqp_solver(A, B, Q, R, initial_state, lower_bound, upper_bound, state_lower_bound, state_upper_bound,
                              reference_state, max_iter, horizon, eps);
    mpc_osqp_solver.Execute(&control_cmd);
    EXPECT_FLOAT_EQ(upper_bound(0), control_cmd[0]);
}

TEST(MpcSolverTest, NullMatrix) {
    const int states = 2;
    const int controls = 1;
    const int horizon = 10;
    const int max_iter = 100;
    const double eps = 0.01;

    Eigen::MatrixXd A(states, states);
    A << 0, 0, 0, 0;

    Eigen::MatrixXd B(states, controls);
    B << 0, 3;

    Eigen::MatrixXd Q(states, states);
    Q << 0, 0, 0, 10;

    Eigen::MatrixXd R(controls, controls);
    R << 0.1;

    Eigen::MatrixXd lower_bound(controls, 1);
    lower_bound << -10;

    Eigen::MatrixXd upper_bound(controls, 1);
    upper_bound << 10;

    Eigen::MatrixXd initial_state(states, 1);
    initial_state << 0, 0;

    Eigen::MatrixXd reference_state(states, 1);
    reference_state << 0, 0;

    Eigen::MatrixXd state_lower_bound(states, 1);
    state_lower_bound << -5, -5;

    Eigen::MatrixXd state_upper_bound(states, 1);
    state_upper_bound << 5, 5;

    std::vector<double> control_cmd(controls, 0);

    MpcSolver mpc_osqp_solver(A, B, Q, R, initial_state, lower_bound, upper_bound, state_lower_bound, state_upper_bound,
                              reference_state, max_iter, horizon, eps);
    mpc_osqp_solver.Execute(&control_cmd);
    EXPECT_NEAR(0.0, control_cmd[0], 1e-7);
}

}  // namespace control
}  // namespace jiduauto
