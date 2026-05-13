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

#include <math.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace jiduauto {
namespace control {

struct DynamicModelState {
    double X;
    double Y;
    double theta;
    double vx;
    double vy;
    double yaw_rate;
    double acc;
    double delta;

    void setZero() {
        X = 0.0;
        Y = 0.0;
        theta = 0.0;
        vx = 0.0;
        vy = 0.0;
        yaw_rate = 0.0;
        acc = 0.0;
        delta = 0.0;
    }
};

struct DynamicModelInput {
    double d_acc = 0.0;
    double d_delta = 0.0;

    void setZero() {
        d_acc = 0.0;
        d_delta = 0.0;
    }
};

typedef Eigen::Matrix<double, NX, 1> StateVector;
typedef Eigen::Matrix<double, NU, 1> InputVector;

typedef Eigen::Matrix<double, NX, NX> A_MPC;
typedef Eigen::Matrix<double, NX, NU> B_MPC;
typedef Eigen::Matrix<double, NX, 1> g_MPC;

typedef Eigen::Matrix<double, NX, 1> q_MPC;
typedef Eigen::Matrix<double, NU, 1> r_MPC;

}  // namespace control
}  // namespace jiduauto
