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

#include "control/src/math/matrix_operations.h"

#include <gtest/gtest.h>

namespace jiduauto {
namespace control {

TEST(PseudoInverseTest, PseudoInverseI) {
    Eigen::Matrix<float, 2, 2> A = Eigen::MatrixXf::Identity(2, 2);

    const double epsilon = 1.0e-6;
    Eigen::Matrix<float, 2, 2> B = PseudoInverse<float, 2>(A, epsilon);

    EXPECT_EQ(B(0, 0), 1);
    EXPECT_EQ(B(0, 1), 0);
    EXPECT_EQ(B(1, 0), 0);
    EXPECT_EQ(B(1, 1), 1);

    const Eigen::Matrix<float, 2, 2> C = Eigen::MatrixXf::Zero(2, 2);

    Eigen::Matrix<float, 2, 2> D = PseudoInverse<float, 2>(C, epsilon);
    EXPECT_EQ(D(0, 0), 0);
    EXPECT_EQ(D(0, 1), 0);
    EXPECT_EQ(D(1, 0), 0);
    EXPECT_EQ(D(1, 1), 0);
}

TEST(PseudoInverseTest, PseudoInverseII) {
    const Eigen::Matrix<float, 5, 1> A = Eigen::MatrixXf::Ones(5, 1);

    const double epsilon = 1.0e-6;

    Eigen::Matrix<float, 1, 5> B = PseudoInverse<float, 5, 1>(A, epsilon);

    EXPECT_EQ(B.cols(), 5);
    EXPECT_EQ(B.rows(), 1);

    EXPECT_FLOAT_EQ(B(0, 0), 0.2);
    EXPECT_FLOAT_EQ(B(0, 1), 0.2);
    EXPECT_FLOAT_EQ(B(0, 2), 0.2);
    EXPECT_FLOAT_EQ(B(0, 3), 0.2);
    EXPECT_FLOAT_EQ(B(0, 4), 0.2);

    const Eigen::Matrix<float, 5, 1> C = Eigen::MatrixXf::Zero(5, 1);

    Eigen::Matrix<float, 1, 5> D = PseudoInverse<float, 5, 1>(C, epsilon);

    EXPECT_EQ(D.cols(), 5);
    EXPECT_EQ(D.rows(), 1);
    EXPECT_FLOAT_EQ(D(0, 0), 0);
    EXPECT_FLOAT_EQ(D(0, 1), 0);
    EXPECT_FLOAT_EQ(D(0, 2), 0);
    EXPECT_FLOAT_EQ(D(0, 3), 0);
    EXPECT_FLOAT_EQ(D(0, 4), 0);
}

TEST(ContinuousToDiscreteTest, c2d_fixed_size) {
    double ts = 0.0;

    Eigen::Matrix<float, 2, 2> m_a = Eigen::MatrixXf::Identity(2, 2);

    Eigen::Matrix<float, 2, 1> m_b = Eigen::MatrixXf::Ones(2, 1);

    Eigen::Matrix<float, 1, 2> m_c = Eigen::MatrixXf::Ones(1, 2);

    Eigen::Matrix<float, 1, 1> m_d = Eigen::MatrixXf::Identity(1, 1);

    Eigen::Matrix<float, 2, 2> prt_a_d;

    Eigen::Matrix<float, 2, 1> prt_b_d;

    Eigen::Matrix<float, 1, 2> prt_c_d;

    Eigen::Matrix<float, 1, 1> prt_d_d;

    bool res = ContinuousToDiscrete<float, 2, 1, 1>(m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d, &prt_c_d, &prt_d_d);

    EXPECT_FALSE(res);

    ts = 1;

    res = ContinuousToDiscrete<float, 2, 1, 1>(m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d, &prt_c_d, &prt_d_d);

    EXPECT_TRUE(res);

    EXPECT_FLOAT_EQ(prt_a_d(0, 0), 3);
    EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
    EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
    EXPECT_FLOAT_EQ(prt_a_d(1, 1), 3);

    EXPECT_FLOAT_EQ(prt_b_d(0, 0), 2);
    EXPECT_FLOAT_EQ(prt_b_d(1, 0), 2);

    EXPECT_FLOAT_EQ(prt_c_d(0, 0), 2);
    EXPECT_FLOAT_EQ(prt_c_d(0, 1), 2);

    EXPECT_FLOAT_EQ(prt_d_d(0, 0), 3);

    ts = 0.1;

    res = ContinuousToDiscrete<float, 2, 1, 1>(m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d, &prt_c_d, &prt_d_d);

    EXPECT_TRUE(res);

    EXPECT_FLOAT_EQ(prt_a_d(0, 0), 1.1052631);
    EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
    EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
    EXPECT_FLOAT_EQ(prt_a_d(1, 1), 1.1052631);

    EXPECT_FLOAT_EQ(prt_b_d(0, 0), 0.10526317);
    EXPECT_FLOAT_EQ(prt_b_d(1, 0), 0.10526317);

    EXPECT_FLOAT_EQ(prt_c_d(0, 0), 1.0526316);
    EXPECT_FLOAT_EQ(prt_c_d(0, 1), 1.0526316);

    EXPECT_FLOAT_EQ(prt_d_d(0, 0), 1.1052631);

    ts = 0.01;

    res = ContinuousToDiscrete<float, 2, 1, 1>(m_a, m_b, m_c, m_d, ts, &prt_a_d, &prt_b_d, &prt_c_d, &prt_d_d);

    EXPECT_TRUE(res);

    EXPECT_FLOAT_EQ(prt_a_d(0, 0), 1.0100503);
    EXPECT_FLOAT_EQ(prt_a_d(0, 1), 0);
    EXPECT_FLOAT_EQ(prt_a_d(1, 0), 0);
    EXPECT_FLOAT_EQ(prt_a_d(1, 1), 1.0100503);

    EXPECT_FLOAT_EQ(prt_b_d(0, 0), 0.010050251);
    EXPECT_FLOAT_EQ(prt_b_d(1, 0), 0.010050251);

    EXPECT_FLOAT_EQ(prt_c_d(0, 0), 1.0050251);
    EXPECT_FLOAT_EQ(prt_c_d(0, 1), 1.0050251);

    EXPECT_FLOAT_EQ(prt_d_d(0, 0), 1.0100503);
}

TEST(DENSE_TO_CSC_MATRIX, dense_to_csc_matrix_test) {
    {
        std::vector<double> data;
        std::vector<double> indices;
        std::vector<double> indptr;
        Eigen::MatrixXd dense_matrix(3, 3);
        dense_matrix << 1.2, 0, 2.2, 0, 0, 3.1, 4.8, 5.4, 6.01;
        DenseToCSCMatrix(dense_matrix, &data, &indices, &indptr);

        std::vector<double> golden_data = {1.2, 4.8, 5.4, 2.2, 3.1, 6.01};
        std::vector<double> golden_indices = {0, 2, 2, 0, 1, 2};
        std::vector<double> golden_indptr = {0, 2, 3, 6};

        EXPECT_EQ(data.size(), golden_data.size());
        EXPECT_EQ(indices.size(), golden_indices.size());
        EXPECT_EQ(indptr.size(), golden_indptr.size());

        for (std::size_t i = 0; i < data.size(); i++) EXPECT_DOUBLE_EQ(data[i], golden_data[i]);
        for (std::size_t i = 0; i < indices.size(); i++) EXPECT_DOUBLE_EQ(indices[i], golden_indices[i]);
        for (std::size_t i = 0; i < indptr.size(); i++) EXPECT_DOUBLE_EQ(indptr[i], golden_indptr[i]);
    }

    {
        std::vector<double> data;
        std::vector<double> indices;
        std::vector<double> indptr;
        Eigen::MatrixXd dense_matrix(2, 2);
        dense_matrix << 4.0, 1.0, 1.0, 2.0;
        DenseToCSCMatrix(dense_matrix, &data, &indices, &indptr);

        std::vector<double> golden_data = {4.0, 1.0, 1.0, 2.0};
        std::vector<double> golden_indices = {0, 1, 0, 1};
        std::vector<double> golden_indptr = {0, 2, 4};

        EXPECT_EQ(data.size(), golden_data.size());
        EXPECT_EQ(indices.size(), golden_indices.size());
        EXPECT_EQ(indptr.size(), golden_indptr.size());

        for (std::size_t i = 0; i < data.size(); i++) EXPECT_DOUBLE_EQ(data[i], golden_data[i]);
        for (std::size_t i = 0; i < indices.size(); i++) EXPECT_DOUBLE_EQ(indices[i], golden_indices[i]);
        for (std::size_t i = 0; i < indptr.size(); i++) EXPECT_DOUBLE_EQ(indptr[i], golden_indptr[i]);
    }

    {
        std::vector<double> data;
        std::vector<int> indices;
        std::vector<int> indptr;
        Eigen::MatrixXd dense_matrix(4, 6);

        dense_matrix << 11, 0, 0, 14, 0, 16, 0, 22, 0, 0, 25, 26, 0, 0, 33, 34, 0, 36, 41, 0, 43, 44, 0, 46;
        DenseToCSCMatrix(dense_matrix, &data, &indices, &indptr);
        std::vector<double> golden_data = {11.0, 41.0, 22.0, 33.0, 43.0, 14.0, 34.0,
                                           44.0, 25.0, 16.0, 26.0, 36.0, 46.0};
        std::vector<int> golden_indices = {0, 3, 1, 2, 3, 0, 2, 3, 1, 0, 1, 2, 3};
        std::vector<int> golden_indptr = {0, 2, 3, 5, 8, 9, 13};

        EXPECT_EQ(data.size(), golden_data.size());
        EXPECT_EQ(indices.size(), golden_indices.size());
        EXPECT_EQ(indptr.size(), golden_indptr.size());

        for (size_t i = 0; i < data.size(); ++i) {
            EXPECT_DOUBLE_EQ(data[i], golden_data[i]);
        }
        for (size_t i = 0; i < indices.size(); ++i) {
            EXPECT_EQ(indices[i], golden_indices[i]);
        }
        for (size_t i = 0; i < indptr.size(); ++i) {
            EXPECT_EQ(indptr[i], golden_indptr[i]) << "i = " << i;
        }
    }
}

}  // namespace control
}  // namespace jiduauto
