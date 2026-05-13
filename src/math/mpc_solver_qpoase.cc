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

#include "control/src/math/mpc_solver_qpoase.h"

#include <algorithm>

#include "pnc_common/src/common/pnc_logger.h"

namespace jiduauto {
namespace control {

namespace {
const double BOUND_LIMIT = 1e10;
const int32_t max_affine_constraint_size = 24000;
const int32_t max_bound_size = 800;
}  // namespace

void MPCSolverQpoase::set_mpc_param(const MPCParam& mpc_param) { mpc_param_ = mpc_param; }

bool MPCSolverQpoase::Solve(const Eigen::MatrixXd& kernel_matrix, const Eigen::MatrixXd& offset,
                            const Eigen::MatrixXd& affine_inequality_matrix,
                            const Eigen::MatrixXd& affine_inequality_boundary,
                            const Eigen::MatrixXd& affine_equality_matrix,
                            const Eigen::MatrixXd& affine_equality_boundary, Eigen::MatrixXd* const solution_ptr) {
    if (solution_ptr == nullptr) {
        LOG_ERROR("input nullptr");
        return false;
    }
    int32_t num_param = kernel_matrix.rows();
    int32_t num_constraint = affine_equality_matrix.rows() + affine_inequality_matrix.rows();

    if (num_param * num_constraint > max_affine_constraint_size || num_constraint > max_bound_size) {
        LOG_ERROR("kernel affine constraint matrix row num over max set num");
        return false;
    }

    bool use_hotstart = last_solver_succeed_ && (qp_problem_ != nullptr) && (qp_problem_.get() != nullptr) &&
                        (num_param == last_num_param_) && (num_constraint == last_num_constraint_);

    if (!use_hotstart) {
        if ((qp_problem_ == nullptr) || (qp_problem_.get() == nullptr) || (num_param != last_num_param_) ||
            (num_constraint != last_num_constraint_)) {
            qp_problem_ = std::make_unique<qpOASES::SQProblem>(num_param, num_constraint, qpOASES::HST_POSDEF);
        }

        qpOASES::Options myOptions;
        myOptions.enableCholeskyRefactorisation = mpc_param_.cholesky_refactorisation_freq;
        myOptions.epsNum = mpc_param_.eps_num;
        myOptions.epsDen = mpc_param_.eps_den;
        myOptions.epsIterRef = mpc_param_.eps_iter_ref;
        myOptions.terminationTolerance = mpc_param_.termination_tolerance;
        myOptions.enableRegularisation = qpOASES::BT_TRUE;
        qp_problem_->setOptions(myOptions);

        if (!mpc_param_.enable_debug) {
            qp_problem_->setPrintLevel(qpOASES::PL_NONE);
        } else {
            qp_problem_->setPrintLevel(qpOASES::PL_LOW);
        }
    }

    // definition of qpOASESproblem
    const int32_t kKernelMatrixRows = kernel_matrix.rows();
    const int32_t kKernelMatrixCols = kernel_matrix.rows();
    const int32_t kOffsetRows = offset.rows();
    double h_matrix[kKernelMatrixRows * kKernelMatrixCols];
    double g_matrix[kOffsetRows * 1];
    int32_t index = 0;

    for (int32_t r = 0; r < kKernelMatrixRows; ++r) {
        g_matrix[r] = offset(r, 0);

        for (int32_t c = 0; c < kKernelMatrixCols; ++c) {
            h_matrix[index++] = kernel_matrix(r, c);
        }
    }

    // search space lower bound and uppper bound
    const int32_t kNumParam = num_param;
    double lower_bound[kNumParam];
    double upper_bound[kNumParam];

    for (int32_t i = 0; i < kNumParam; ++i) {
        lower_bound[i] = -BOUND_LIMIT;
        upper_bound[i] = BOUND_LIMIT;
    }

    // constraint matrix construction
    static double affine_constraint_matrix[max_affine_constraint_size];
    static double constraint_lower_bound[max_bound_size];
    static double constraint_upper_bound[max_bound_size];
    index = 0;

    for (int32_t r = 0; r < affine_equality_matrix.rows(); ++r) {
        constraint_lower_bound[r] = affine_equality_boundary(r, 0);
        constraint_upper_bound[r] = affine_equality_boundary(r, 0);

        for (int32_t c = 0; c < num_param; ++c) {
            affine_constraint_matrix[index++] = affine_equality_matrix(r, c);
        }
    }

    for (int32_t r = 0; r < affine_inequality_matrix.rows(); ++r) {
        constraint_lower_bound[r + affine_equality_boundary.rows()] = affine_inequality_boundary(r, 0);
        constraint_upper_bound[r + affine_equality_boundary.rows()] = BOUND_LIMIT;

        for (int32_t c = 0; c < num_param; ++c) {
            affine_constraint_matrix[index++] = affine_inequality_matrix(r, c);
        }
    }

    // initialize problem
    int32_t max_iter = std::max(mpc_param_.max_iteration, num_constraint);

    qpOASES::returnValue ret;

    if (use_hotstart) {
        ret = qp_problem_->hotstart(h_matrix, g_matrix, affine_constraint_matrix, lower_bound, upper_bound,
                                    constraint_lower_bound, constraint_upper_bound, max_iter);
    } else {
        ret = qp_problem_->init(h_matrix, g_matrix, affine_constraint_matrix, lower_bound, upper_bound,
                                constraint_lower_bound, constraint_upper_bound, max_iter);
        LOG_INFO("not using hotstart");
    }

    if (ret != qpOASES::SUCCESSFUL_RETURN) {
        LOG_ERROR("qpOASES solver failed due to: (%d) %s", ret, qpOASES::MessageHandling::getErrorCodeMessage(ret));
        last_solver_succeed_ = false;
        return false;
    }

    last_solver_succeed_ = true;
    last_num_param_ = num_param;
    last_num_constraint_ = num_constraint;

    double result[kNumParam];
    qp_problem_->getPrimalSolution(result);

    for (int32_t i = 0; i < kNumParam; ++i) {
        (*solution_ptr)(i, 0) = result[i];
    }

    return qp_problem_->isSolved() == qpOASES::BT_TRUE;
}

}  // namespace control
}  // namespace jiduauto
