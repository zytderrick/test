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

#include "control/src/common/kinematic_model/kinematic_model.h"

#include <unsupported/Eigen/MatrixFunctions>

namespace jiduauto {
namespace control {

// State Evaluate
KinematicStateVector KinematicModel::NonlinearEvalulate(const KinematicModelState& x,
                                                        const KinematicModelInput& u) const {
    // state = {X,Y,theta,speed}
    const double theta = x.theta;
    const double speed = x.speed;
    const double delta = x.delta;

    const double acc = u.acc;
    const double delta_rate = u.delta_rate;

    // nonlinear kinematic bicycle model
    // Reference: Vehicle Dynamic and Control, Page 23, Eqn 2.10-2.12
    double X_dot = speed * std::cos(theta);
    double Y_dot = speed * std::sin(theta);
    double theta_dot = speed * std::tan(delta) / (ll_);
    double speed_dot = acc;
    double delta_dot = delta_rate;

    KinematicStateVector state_dot;
    state_dot(0) = X_dot;
    state_dot(1) = Y_dot;
    state_dot(2) = theta_dot;
    state_dot(3) = speed_dot;
    state_dot(4) = delta_dot;

    return state_dot;
}

KinLinModelMatrix KinematicModel::getModelJacobian(const KinematicModelState& x, const KinematicModelInput& u) const {
    // compute jacobian of the model
    const double theta = x.theta;
    const double speed = x.speed;
    const double delta = x.delta;

    A_Kinematic A_c = A_Kinematic::Zero();
    B_Kinematic B_c = B_Kinematic::Zero();
    d_Kinematic d_c = d_Kinematic::Zero();

    A_c(0, 2) = -speed * std::sin(theta);
    A_c(0, 3) = std::cos(theta);
    A_c(1, 2) = speed * std::cos(theta);
    A_c(1, 3) = std::sin(theta);
    A_c(2, 3) = std::tan(delta) / (ll_);
    A_c(2, 4) = speed / ((ll_) * (std::cos(delta) * std::cos(delta)));

    B_c(3, 0) = 1.0;
    B_c(4, 1) = 1.0;

    const KinematicStateVector state_dot = NonlinearEvalulate(x, u);

    // zero order term
    KinematicStateVector state_cur;
    state_cur(0) = x.X;
    state_cur(1) = x.Y;
    state_cur(2) = x.theta;
    state_cur(3) = x.speed;
    state_cur(4) = x.delta;

    KinematicInputVector input_cur;
    input_cur(0) = u.acc;
    input_cur(1) = u.delta_rate;

    d_c = state_dot - A_c * state_cur - B_c * input_cur;

    return {A_c, B_c, d_c};
}

KinLinModelMatrix KinematicModel::discretizeModel(const KinLinModelMatrix& lin_model_c) const {
    // disctetize the continuous time linear model  x_dot = A x + B u + g using
    // ZHO
    const int NX = 5;  // jiduauto::control::KinematicModelState::size;
    const int NU = 2;  // jiduauto::control::KinematicModelInput::size;

    Eigen::Matrix<double, NX + NU + 1, NX + NU + 1> temp = Eigen::Matrix<double, NX + NU + 1, NX + NU + 1>::Zero();
    // building matrix necessary for expm
    // temp = Ts*[A,B,g;zeros]
    temp.block<NX, NX>(0, 0) = lin_model_c.A;
    temp.block<NX, NU>(0, NX) = lin_model_c.B;
    temp.block<NX, 1>(0, NX + NU) = lin_model_c.d;
    temp = temp * Ts_;
    // take the matrix exponential of temp
    const Eigen::Matrix<double, NX + NU + 1, NX + NU + 1> temp_res = temp.exp();
    // extract dynamics out of big matrix
    // x_{k+1} = Ad x_k + Bd u_k + gd
    // temp_res = [Ad,Bd,gd;zeros]
    const A_Kinematic A_d = temp_res.block<NX, NX>(0, 0);
    const B_Kinematic B_d = temp_res.block<NX, NU>(0, NX);
    const d_Kinematic d_d = temp_res.block<NX, 1>(0, NX + NU);

    return {A_d, B_d, d_d};
}

KinLinModelMatrix KinematicModel::getDiscreteLinModel(const KinematicModelState& x,
                                                      const KinematicModelInput& u) const {
    // compute linearized and discretized model
    const KinLinModelMatrix lin_model_c = getModelJacobian(x, u);
    // discretize the system
    return discretizeModel(lin_model_c);
    // TODO(zhaobolin): add a check in case NaN in matrix
}

}  // namespace control
}  // namespace jiduauto
