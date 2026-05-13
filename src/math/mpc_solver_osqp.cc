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

#include "control/src/math/mpc_solver_osqp.h"

#include <memory>
#include <utility>

#include "pnc_common/src/common/pnc_logger.h"

namespace {
static constexpr double kEpsilon = 1e-6;
}

namespace jiduauto {
namespace control {

MpcSolverOsqp::MpcSolverOsqp(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b,
                             const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r,
                             const Eigen::MatrixXd& matrix_initial_x, const Eigen::MatrixXd& matrix_u_lower,
                             const Eigen::MatrixXd& matrix_u_upper, const Eigen::MatrixXd& matrix_x_lower,
                             const Eigen::MatrixXd& matrix_x_upper, const Eigen::MatrixXd& matrix_x_ref,
                             const int32_t max_iter, const int32_t horizon, const double eps_abs)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
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

    settings_ = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    osqp_set_default_settings(settings_);
    settings_->polish = true;
    settings_->scaled_termination = true;
    settings_->verbose = false;
    settings_->max_iter = max_iteration_;
    settings_->eps_abs = eps_abs_;

    // Populate data
    data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
}

MpcSolverOsqp::~MpcSolverOsqp() { CleanUp(); }

bool MpcSolverOsqp::ResetMatrix(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b,
                                const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r,
                                const Eigen::MatrixXd& matrix_initial_x, const Eigen::MatrixXd& matrix_u_lower,
                                const Eigen::MatrixXd& matrix_u_upper, const Eigen::MatrixXd& matrix_x_lower,
                                const Eigen::MatrixXd& matrix_x_upper, const Eigen::MatrixXd& matrix_x_ref,
                                const int32_t max_iter, const int32_t horizon, const double eps_abs) {
    matrix_a_ = matrix_a;
    matrix_b_ = matrix_b;
    matrix_q_ = matrix_q;
    matrix_r_ = matrix_r;
    matrix_initial_x_ = matrix_initial_x;
    matrix_u_lower_ = matrix_u_lower;
    matrix_u_upper_ = matrix_u_upper;
    matrix_x_lower_ = matrix_x_lower;
    matrix_x_upper_ = matrix_x_upper;
    matrix_x_ref_ = matrix_x_ref;
    max_iteration_ = max_iter;
    horizon_ = horizon;
    eps_abs_ = eps_abs;
    state_dim_ = matrix_b.rows();
    control_dim_ = matrix_b.cols();
    num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;

    settings_ = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    osqp_set_default_settings(settings_);
    settings_->polish = true;
    settings_->scaled_termination = true;
    settings_->verbose = false;
    settings_->max_iter = max_iteration_;
    settings_->eps_abs = eps_abs_;

    // Populate data
    data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    return true;
}

void MpcSolverOsqp::CleanUp() {
    if (work_) osqp_cleanup(work_);
    if (data_) {
        if (data_->A) c_free(data_->A);
        if (data_->P) c_free(data_->P);
        if (data_) c_free(data_);
    }
    if (settings_) {
        c_free(settings_);
    }
}

void MpcSolverOsqp::CalculateKernel(std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
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

// reference is always zero
void MpcSolverOsqp::CalculateGradient() {
    // populate the gradient vector
    gradient_ = Eigen::VectorXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    for (size_t i = 0; i < horizon_ + 1; ++i) {
        gradient_.block(i * state_dim_, 0, state_dim_, 1) = -1.0 * matrix_q_ * matrix_x_ref_;
    }
}

// equality constraints x(k+1) = A*x(k)
void MpcSolverOsqp::CalculateEqualityConstraint(std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
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
        matrix_constraint.block((i + 1) * state_dim_, i * state_dim_, state_dim_, state_dim_) = matrix_a_;
    }

    for (size_t i = 0; i < horizon_; ++i) {
        matrix_constraint.block((i + 1) * state_dim_, i * control_dim_ + (horizon_ + 1) * state_dim_, state_dim_,
                                control_dim_) = matrix_b_;
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

void MpcSolverOsqp::CalculateConstraintVectors() {
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    for (size_t i = 0; i < horizon_; ++i) {
        lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0, control_dim_, 1) = matrix_u_lower_;
        upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0, control_dim_, 1) = matrix_u_upper_;
    }
    for (size_t i = 0; i < horizon_ + 1; ++i) {
        lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_lower_;
        upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_upper_;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, state_dim_, 1) = -1 * matrix_initial_x_;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound_ = Eigen::MatrixXd::Zero(2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    lowerBound_ << lowerEquality, lowerInequality;
    upperBound_ = Eigen::MatrixXd::Zero(2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    upperBound_ << upperEquality, upperInequality;
}

void MpcSolverOsqp::CalculateData() {
    size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    size_t num_affine_constraint = 2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    data_->n = kernel_dim;
    data_->m = num_affine_constraint;
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices;
    std::vector<c_int> P_indptr;
    CalculateKernel(&P_data, &P_indices, &P_indptr);
    data_->P =
        csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data), CopyData(P_indices), CopyData(P_indptr));
    data_->q = gradient_.data();
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);
    data_->A = csc_matrix(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) + control_dim_ * horizon_,
                          kernel_dim, A_data.size(), CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
    data_->l = lowerBound_.data();
    data_->u = upperBound_.data();
    return;
}

bool MpcSolverOsqp::Execute(std::vector<double>* control_cmd) {
    CalculateGradient();

    CalculateConstraintVectors();

    CalculateData();

    work_ = osqp_setup(data_, settings_);

    osqp_solve(work_);

    auto status = work_->info->status_val;

    // check status
    if (status < 0 || (status != 1 && status != 2) || work_->solution == nullptr) {
        LOG_ERROR("ERROR in osqp_solve, solve status: %d", status);
        return false;
    }

    size_t first_control = state_dim_ * (horizon_ + 1);
    for (size_t i = 0; i < control_dim_; ++i) {
        control_cmd->at(i) = work_->solution->x[i + first_control];
    }

    return true;
}

}  // namespace control
}  // namespace jiduauto
