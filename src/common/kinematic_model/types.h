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

#include <Eigen/Dense>

namespace jiduauto {
namespace control {

struct KinematicModelState {
    const int32_t size = 4;  // X,Y,theta,speed

    double X;
    double Y;
    double theta;
    double speed;
    double delta;

    void setZero() {
        X = 0.0;
        Y = 0.0;
        theta = 0.0;
        speed = 0.0;
        delta = 0.0;
    }
};

struct KinematicModelInput {
    const int32_t size = 2;  // acc delta

    double acc;
    double delta_rate;

    void setZero() {
        acc = 0.0;
        delta_rate = 0.0;
    }
};

const int32_t Nx = 5;
const int32_t Nu = 2;

typedef Eigen::Matrix<double, Nx, 1> KinematicStateVector;
typedef Eigen::Matrix<double, Nu, 1> KinematicInputVector;

typedef Eigen::Matrix<double, Nx, Nx> A_Kinematic;
typedef Eigen::Matrix<double, Nx, Nu> B_Kinematic;
typedef Eigen::Matrix<double, Nx, 1> d_Kinematic;

struct KinLinModelMatrix {
    A_Kinematic A;
    B_Kinematic B;
    d_Kinematic d;
};

}  // namespace control
}  // namespace jiduauto
