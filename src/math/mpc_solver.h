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

#include <osqp.h>

#include <Eigen/Core>
#include <algorithm>
#include <vector>

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

class MpcSolver {
public:
    /**
     * @brief Solver for discrete-time model predictive control problem.
     * @param matrix_a The system dynamic matrix
     * @param matrix_b The control matrix
     * @param matrix_q The cost matrix for control state
     * @param matrix_lower The lower bound control constrain matrix
     * @param matrix_upper The upper bound control constrain matrix
     * @param matrix_initial_state The initial state matrix
     * @param max_iter The maximum iterations
     */
    MpcSolver(const Eigen::MatrixXd& matrix_a, const Eigen::MatrixXd& matrix_b, const Eigen::MatrixXd& matrix_q,
              const Eigen::MatrixXd& matrix_r, const Eigen::MatrixXd& matrix_initial_x,
              const Eigen::MatrixXd& matrix_u_lower, const Eigen::MatrixXd& matrix_u_upper,
              const Eigen::MatrixXd& matrix_x_lower, const Eigen::MatrixXd& matrix_x_upper,
              const Eigen::MatrixXd& matrix_x_ref, const int32_t max_iter, const int32_t horizon, const double eps_abs);

    ~MpcSolver() = default;
    /**
     * @brief execute solver
     * @return bool whether have result
     */
    bool Execute(std::vector<double>* control_cmd);

private:
    /**
     * @brief get osqp settings
     * @return OSQPSettings osqp settings
     */
    OSQPSettings* Settings();

    /**
     * @brief get osqp data
     * @return Data osqp data
     */
    OSQPData* Data();

    /**
     * @brief free memory
     * @param data point of data
     * @param settings point of settings
     */
    void FreeData(OSQPData* data);

    /**
     * @brief calculate osqp P
     * @param P_data data of P
     * @param P_indices indices of P
     * @param P_indptr indptr of P
     */
    void CalculateKernel(std::vector<c_float>* P_data, std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr);
    /**
     * @brief calculate osqp q
     * gradient
     * @param q_data data of q
     */
    void CalculateGradient();

    /**
     * @brief calculate osqp A
     * @param A_data data of A
     * @param A_indices indices of A
     * @param A_indptr indptr of A
     */
    void CalculateEqualityConstraint(std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
                                     std::vector<c_int>* A_indptr);
    /**
     * @brief calculate the constraint of osqp
     * @param lowerBound lowerbound of state
     * @param upperBound upperbound of state
     */
    void CalculateConstraintVectors();

    template <typename T>
    T* CopyData(const std::vector<T>& vec) {
        T* data = new T[vec.size()];
        memcpy(data, vec.data(), sizeof(T) * vec.size());
        return data;
    }

private:
    Eigen::MatrixXd matrix_a_;
    Eigen::MatrixXd matrix_b_;
    Eigen::MatrixXd matrix_q_;
    Eigen::MatrixXd matrix_r_;
    Eigen::MatrixXd matrix_initial_x_;
    const Eigen::MatrixXd matrix_u_lower_;
    const Eigen::MatrixXd matrix_u_upper_;
    const Eigen::MatrixXd matrix_x_lower_;
    const Eigen::MatrixXd matrix_x_upper_;
    const Eigen::MatrixXd matrix_x_ref_;
    int32_t max_iteration_;
    std::size_t horizon_;
    double eps_abs_;
    std::size_t state_dim_;
    std::size_t control_dim_;
    std::size_t num_param_;
    int32_t num_constraint_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd lowerBound_;
    Eigen::VectorXd upperBound_;
};

}  // namespace control
}  // namespace jiduauto
