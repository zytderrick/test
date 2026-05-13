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

/**
 * @file
 * @brief Defines some useful matrix operations.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <cmath>
#include <utility>
#include <vector>

#include "pnc_common/src/common/pnc_logger.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

/**
 * @brief Computes the Moore-Penrose pseudo-inverse of a given square matrix,
 * rounding all eigenvalues with absolute value bounded by epsilon to zero.
 * @param m The square matrix to be pseudo-inverted
 * @param epsilon A small positive real number (optional; default is 1.0e-6).
 * @return Moore-Penrose pseudo-inverse of the given matrix.
 */
template <typename T, unsigned int N>
Eigen::Matrix<T, N, N> PseudoInverse(const Eigen::Matrix<T, N, N>& m, const double epsilon = 1.0e-6) {
    Eigen::JacobiSVD<Eigen::Matrix<T, N, N>> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return static_cast<Eigen::Matrix<T, N, N>>(svd.matrixV() *
                                               (svd.singularValues().array().abs() > epsilon)
                                                   .select(svd.singularValues().array().inverse(), 0)
                                                   .matrix()
                                                   .asDiagonal() *
                                               svd.matrixU().adjoint());
}

/**
 * @brief Computes the Moore-Penrose pseudo-inverse of a given matrix,
 * rounding all eigenvalues with absolute value bounded by epsilon to zero.
 * @param m The matrix to be pseudo-inverted
 * @param epsilon A small positive real number (optional; default is 1.0e-6).
 * @return Moore-Penrose pseudo-inverse of the given matrix.
 */
template <typename T, unsigned int M, unsigned int N>
Eigen::Matrix<T, N, M> PseudoInverse(const Eigen::Matrix<T, M, N>& m, const double epsilon = 1.0e-6) {
    Eigen::Matrix<T, M, M> t = m * m.transpose();
    return static_cast<Eigen::Matrix<T, N, M>>(m.transpose() * PseudoInverse<T, M>(t));
}

/**
 * @brief Computes bilinear transformation of the continuous to discrete form
 * for state space representation This assumes equation format of
 *
 *           dot_x = Ax + Bu
 *           y = Cx + Du
 *
 * @param m_a, m_b, m_c, m_d are the state space matrix control matrix
 * @return true or false.
 */
template <typename T, unsigned int L, unsigned int N, unsigned int O>
bool ContinuousToDiscrete(const Eigen::Matrix<T, L, L>& m_a, const Eigen::Matrix<T, L, N>& m_b,
                          const Eigen::Matrix<T, O, L>& m_c, const Eigen::Matrix<T, O, N>& m_d, const double ts,
                          Eigen::Matrix<T, L, L>* ptr_a_d, Eigen::Matrix<T, L, N>* ptr_b_d,
                          Eigen::Matrix<T, O, L>* ptr_c_d, Eigen::Matrix<T, O, N>* ptr_d_d) {
    if (ts <= 0.0) {
        LOG_ERROR("ContinuousToDiscrete : ts is less than or equal to zero");
        return false;
    }

    // Only matrix_a is mandatory to be non-zeros in matrix
    // conversion.
    if (m_a.rows() == 0) {
        LOG_ERROR("ContinuousToDiscrete: matrix_a size 0 ");
        return false;
    }

    if (m_a.cols() != m_b.rows() || m_b.cols() != m_d.cols() || m_c.rows() != m_d.rows() || m_a.cols() != m_c.cols()) {
        LOG_ERROR("ContinuousToDiscrete: matrix dimensions mismatch");
        return false;
    }
    Eigen::Matrix<T, L, L> m_identity = Eigen::Matrix<T, L, L>::Identity();
    *ptr_a_d = PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) * (m_identity + ts * 0.5 * m_a);

    *ptr_b_d = PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) * m_b * ts;

    *ptr_c_d = m_c * PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a);

    *ptr_d_d = 0.5 * m_c * PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) * m_b * ts + m_d;

    return true;
}

/**
 * @brief convert Dense matrix to CSC matrix, used in osqp.
 */
template <typename T, int M, int N, typename D>
void DenseToCSCMatrix(const Eigen::Matrix<T, M, N>& dense_matrix, std::vector<T>* data, std::vector<D>* indices,
                      std::vector<D>* indptr) {
    constexpr double epsilon = 1e-9;
    int data_count = 0;
    for (int c = 0; c < dense_matrix.cols(); ++c) {
        indptr->emplace_back(data_count);
        for (int r = 0; r < dense_matrix.rows(); ++r) {
            if (std::fabs(dense_matrix(r, c)) < epsilon) {
                continue;
            }
            data->emplace_back(dense_matrix(r, c));
            ++data_count;
            indices->emplace_back(r);
        }
    }
    indptr->emplace_back(data_count);
}

}  // namespace control
}  // namespace jiduauto
