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

#include "control/src/math/mpc_osqp_solver.h"

#include <memory>
#include <utility>

#include "pnc_common/src/common/pnc_logger.h"

namespace {
static constexpr double kEpsilon = 1e-6;
}

namespace jiduauto {
namespace control {

MpcOSQPSolver::MpcOSQPSolver(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b,
                             const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r,
                             const Eigen::MatrixXd& matrix_g, const Eigen::MatrixXd& matrix_initial_x,
                             const Eigen::MatrixXd& matrix_u_lower, const Eigen::MatrixXd& matrix_u_upper,
                             const Eigen::MatrixXd& matrix_x_lower, const Eigen::MatrixXd& matrix_x_upper,
                             const std::vector<Eigen::MatrixXd>& matrix_x_ref, const int32_t max_iter,
                             const int32_t horizon, const double eps_abs)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_g_(matrix_g),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),
      max_iteration_(max_iter),
      horizon_(horizon),
      eps_abs_(eps_abs) {
    state_dim_ = matrix_b.rows();
    control_dim_ = matrix_b.cols();
    num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
}

void MpcOSQPSolver::CalculateKernel(std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
                                    std::vector<c_int>* P_indptr) {
    // col1:(row,val),...; col2:(row,val),....; ...
    std::vector<std::vector<std::pair<c_int, c_float>>> columns;
    columns.resize(num_param_);
    std::size_t value_index = 0;
    // state and terminal state
    for (size_t i = 0; i <= horizon_; ++i) {
        for (size_t j = 0; j < state_dim_; ++j) {
            // (row, val)
            columns[i * state_dim_ + j].emplace_back(i * state_dim_ + j, matrix_q_(j, j));
            ++value_index;
        }
    }
    // control
    const size_t state_total_dim = state_dim_ * (horizon_ + 1);
    for (size_t i = 0; i < horizon_; ++i) {
        for (size_t j = 0; j < control_dim_; ++j) {
            // (row, val)
            columns[i * control_dim_ + j + state_total_dim].emplace_back(state_total_dim + i * control_dim_ + j,
                                                                         matrix_r_(j, j));
            ++value_index;
        }
    }
    if (value_index != num_param_) {
        return;
    }

    int32_t ind_p = 0;
    for (size_t i = 0; i < num_param_; ++i) {
        // TODO(zhaobolin): Check this
        P_indptr->emplace_back(ind_p);
        for (const auto& row_data_pair : columns[i]) {
            P_data->emplace_back(row_data_pair.second);    // val
            P_indices->emplace_back(row_data_pair.first);  // row
            ++ind_p;
        }
    }
    P_indptr->emplace_back(ind_p);
}

// reference is changing
void MpcOSQPSolver::CalculateGradient() {
    // populate the gradient vector
    gradient_ = Eigen::VectorXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    for (size_t i = 0; i < horizon_ + 1; ++i) {
        gradient_.block(i * state_dim_, 0, state_dim_, 1) = -1.0 * matrix_q_ * matrix_x_ref_[i];
    }
}

// equality constraints x(k+1) = A*x(k)
void MpcOSQPSolver::CalculateEqualityConstraint(std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
                                                std::vector<c_int>* A_indptr) {
    // block matrix
    Eigen::MatrixXd matrix_constraint =
        Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) + control_dim_ * horizon_,
                              state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);
    Eigen::MatrixXd state_identity_mat =
        Eigen::MatrixXd::Identity(state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1));

    matrix_constraint.block(0, 0, state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1)) = -1 * state_identity_mat;

    Eigen::MatrixXd control_identity_mat = Eigen::MatrixXd::Identity(control_dim_, control_dim_);

    for (size_t i = 0; i < horizon_; ++i) {
        matrix_constraint.block((i + 1) * state_dim_, i * state_dim_, state_dim_,
                                state_dim_) = matrix_a_;  // TODO(zhaobolin): take into account continue-linearization
    }

    for (size_t i = 0; i < horizon_; ++i) {
        matrix_constraint.block((i + 1) * state_dim_, i * control_dim_ + (horizon_ + 1) * state_dim_, state_dim_,
                                control_dim_) = matrix_b_;  // TODO(zhaobolin): take into account continue-linearization
    }

    Eigen::MatrixXd all_identity_mat = Eigen::MatrixXd::Identity(num_param_, num_param_);

    matrix_constraint.block(state_dim_ * (horizon_ + 1), 0, num_param_, num_param_) = all_identity_mat;

    std::vector<std::vector<std::pair<c_int, c_float>>> columns;
    columns.resize(num_param_);
    int32_t value_index = 0;
    // state and terminal state
    for (size_t i = 0; i < num_param_; ++i) {                                  // col
        for (size_t j = 0; j < num_param_ + state_dim_ * (horizon_ + 1); ++j)  // row
            if (std::fabs(matrix_constraint(j, i)) > kEpsilon) {
                // (row, val)
                columns[i].emplace_back(j, matrix_constraint(j, i));
                ++value_index;
            }
    }
    int32_t ind_A = 0;
    for (size_t i = 0; i < num_param_; ++i) {
        A_indptr->emplace_back(ind_A);
        for (const auto& row_data_pair : columns[i]) {
            A_data->emplace_back(row_data_pair.second);    // value
            A_indices->emplace_back(row_data_pair.first);  // row
            ++ind_A;
        }
    }
    A_indptr->emplace_back(ind_A);
}

void MpcOSQPSolver::CalculateConstraintVectors() {
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    for (size_t i = 0; i < horizon_ + 1; ++i) {
        lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_lower_;
        upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_upper_;
    }
    for (size_t i = 0; i < horizon_; ++i) {
        lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0, control_dim_, 1) = matrix_u_lower_;
        upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0, control_dim_, 1) = matrix_u_upper_;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
    Eigen::VectorXd upperEquality;
    for (size_t i = 0; i < horizon_ + 1; ++i) {
        if (i == 0) {
            lowerEquality.block(i, 0, state_dim_, 1) = -1.0 * matrix_initial_x_;
        } else {
            lowerEquality.block(i * state_dim_, 0, state_dim_, 1) = -1.0 * matrix_g_;
        }
    }
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound_ = Eigen::MatrixXd::Zero(2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    lowerBound_ << lowerEquality, lowerInequality;
    upperBound_ = Eigen::MatrixXd::Zero(2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    upperBound_ << upperEquality, upperInequality;
}

OSQPSettings* MpcOSQPSolver::Settings() {
    // default setting
    OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    osqp_set_default_settings(settings);
    settings->polish = true;
    settings->scaled_termination = true;
    settings->verbose = false;
    settings->max_iter = max_iteration_;
    settings->eps_abs = eps_abs_;
    return settings;
}

OSQPData* MpcOSQPSolver::Data() {
    OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    size_t num_affine_constraint = 2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    data->n = kernel_dim;
    data->m = num_affine_constraint;
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices;
    std::vector<c_int> P_indptr;
    CalculateKernel(&P_data, &P_indices, &P_indptr);
    data->P =
        csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data), CopyData(P_indices), CopyData(P_indptr));
    data->q = gradient_.data();
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);
    data->A = csc_matrix(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) + control_dim_ * horizon_,
                         kernel_dim, A_data.size(), CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
    data->l = lowerBound_.data();
    data->u = upperBound_.data();
    return data;
}

void MpcOSQPSolver::FreeData(OSQPData* data) {
    if (data->A) c_free(data->A);
    if (data->P) c_free(data->P);
    if (data) c_free(data);
}

bool MpcOSQPSolver::Execute(std::vector<double>* control_cmd, std::vector<double>* control_state) {
    CalculateGradient();

    CalculateConstraintVectors();

    OSQPData* data = Data();

    OSQPSettings* settings = Settings();
    OSQPWorkspace* osqp_workspace = osqp_setup(data, settings);
    osqp_solve(osqp_workspace);

    auto status = osqp_workspace->info->status_val;

    // check status
    if (status < 0 || (status != 1 && status != 2)) {
        osqp_cleanup(osqp_workspace);
        FreeData(data);
        c_free(settings);
        return false;
    } else if (osqp_workspace->solution == nullptr) {
        osqp_cleanup(osqp_workspace);
        FreeData(data);
        c_free(settings);
        return false;
    }

    for (size_t i = 0; i < state_dim_ * (horizon_ + 1); ++i) {
        control_state->at(i) = osqp_workspace->solution->x[i];
    }

    size_t first_control = state_dim_ * (horizon_ + 1);
    for (size_t i = 0; i < control_dim_ * horizon_; ++i) {
        control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
    }
    LOG_DEBUG("OSQP Solver filled all result.");

    // Cleanup
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);

    return true;
}

}  // namespace control
}  // namespace jiduauto
