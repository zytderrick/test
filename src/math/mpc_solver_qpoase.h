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
#pragma once

#include <Eigen/Core>
#include <memory>
#include <qpOASES.hpp>

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

struct MPCParam {
    int32_t cholesky_refactorisation_freq{0};
    int32_t max_iteration{1000};
    double eps_num{-1e-7};
    double eps_den{-1e-7};
    double eps_iter_ref{-1e-7};
    double termination_tolerance{1.0e-9};
    bool enable_debug{false};
};

/**
 * @brief Solve MPC in standard QP problem form.
 */
class MPCSolverQpoase {
public:
    MPCSolverQpoase() = default;
    virtual ~MPCSolverQpoase() = default;

    bool Solve(const Eigen::MatrixXd& kernel_matrix, const Eigen::MatrixXd& offset,
               const Eigen::MatrixXd& affine_inequality_matrix, const Eigen::MatrixXd& affine_inequality_boundary,
               const Eigen::MatrixXd& affine_equality_matrix, const Eigen::MatrixXd& affine_equality_boundary,
               Eigen::MatrixXd* const solution_ptr);

    void set_mpc_param(const MPCParam& mpc_param);

private:
    MPCParam mpc_param_;

    std::unique_ptr<::qpOASES::SQProblem> qp_problem_{nullptr};
    // hot start
    int32_t last_num_constraint_{0};
    int32_t last_num_param_{0};
    bool last_solver_succeed_{false};
};

}  // namespace control
}  // namespace jiduauto
