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

#include "control/src/common/dynamic_model/dynamic_model.h"

namespace jiduauto {
namespace control {

double DynamicModel::getSlipAngleFront(const DynamicModelState& x) const {
    return -std::atan2(x.vy + x.yaw_rate * param_.lf, x.vx) + x.delta;
}

double DynamicModel::getSlipAngleRear(const DynamicModelState& x) const {
    return -std::atan2(x.vy - x.yaw_rate * param_.lr, x.vx);
}

TireForces DynamicModel::getForceFront(const DynamicModelState& x) const {
    const double alpha_f = getSlipAngleFront(x);
    const double F_y = param_.Df * std::sin(param_.Cf * std::atan(param_.Bf * alpha_f));
    const double F_x = 0.0;  // TODO(bolin.zhao): brake distribution

    return {F_y, F_x};
}

TireForces DynamicModel::getForceRear(const DynamicModelState& x) const {
    const double alpha_r = getSlipAngleRear(x);
    const double F_y = param_.Dr * std::sin(param_.Cr * std::atan(param_.Br * alpha_r));

    const double F_x = param_.m * x.acc;

    return {F_y, F_x};
}

double DynamicModel::getForceFriction(const DynamicModelState& x) const {
    // summary resistance forces, should be negative
    const double F_slope = 0.0;
    const double F_air = 0.0;
    const double F_roll = 0.0;
    const double F_xd = F_slope + F_air + F_roll;
    return F_xd;
}

NormalForces DynamicModel::getForceNormal(const DynamicModelState& x) const {
    const double f_n_front = param_.lr / (param_.lf + param_.lr) * param_.m * param_.g;
    const double f_n_rear = param_.lf / (param_.lf + param_.lr) * param_.m * param_.g;
    return {f_n_front, f_n_rear};
}

TireForcesDerivatives DynamicModel::getForceFrontDerivatives(const DynamicModelState& x) const {
    const double alpha_f = getSlipAngleFront(x);
    const double vx = x.vx;
    const double vy = x.vy;
    const double yaw_rate = x.yaw_rate;

    // F_fx
    const double dF_x_vx = 0.0;
    const double dF_x_vy = 0.0;
    const double dF_x_yaw_rate = 0.0;
    const double dF_x_acc = 0.0;
    const double dF_x_delta = 0.0;
    // F_fy
    const double dF_y_vx = (param_.Bf * param_.Cf * param_.Df * std::cos(param_.Cf * std::atan(param_.Bf * alpha_f))) /
                           (1. + std::pow(param_.Bf, 2) * std::pow(alpha_f, 2)) *
                           ((param_.lf * yaw_rate + vy) / (std::pow((param_.lf * yaw_rate + vy), 2) + std::pow(vx, 2)));
    const double dF_y_vy = (param_.Bf * param_.Cf * param_.Df * std::cos(param_.Cf * std::atan(param_.Bf * alpha_f))) /
                           (1. + std::pow(param_.Bf, 2) * std::pow(alpha_f, 2)) *
                           (-vx / (std::pow((param_.lf * yaw_rate + vy), 2) + std::pow(vx, 2)));
    const double dF_y_yaw_rate =
        (param_.Bf * param_.Cf * param_.Df * std::cos(param_.Cf * std::atan(param_.Bf * alpha_f))) /
        (1. + std::pow(param_.Bf, 2) * std::pow(alpha_f, 2)) *
        ((-param_.lf * vx) / (std::pow((param_.lf * yaw_rate + vy), 2) + std::pow(vx, 2)));
    const double dF_y_acc = 0.0;
    const double dF_y_delta =
        (param_.Bf * param_.Cf * param_.Df * std::cos(param_.Cf * std::atan(param_.Bf * alpha_f))) /
        (1. + std::pow(param_.Bf, 2) * std::pow(alpha_f, 2));

    return {dF_y_vx, dF_y_vy, dF_y_yaw_rate, dF_y_acc, dF_y_delta,
            dF_x_vx, dF_x_vy, dF_x_yaw_rate, dF_x_acc, dF_x_delta};
}

TireForcesDerivatives DynamicModel::getForceRearDerivatives(const DynamicModelState& x) const {
    const double alpha_r = getSlipAngleRear(x);
    const double vx = x.vx;
    const double vy = x.vy;
    const double yaw_rate = x.yaw_rate;
    // const double acc = x.acc;

    // F_rx  Rear wheel drive (Fx = m*acc)
    const double dF_x_vx = 0.0;  // -param_.m * x.vx [?]
    const double dF_x_vy = 0.0;
    const double dF_x_r = 0.0;
    const double dF_x_acc = param_.m;
    const double dF_x_delta = 0.0;
    // F_ry
    const double dF_y_vx =
        ((param_.Br * param_.Cr * param_.Dr * std::cos(param_.Cr * std::atan(param_.Br * alpha_r))) /
         (1. + std::pow(param_.Br, 2) * std::pow(alpha_r, 2))) *
        (-(param_.lr * yaw_rate - vy) / (std::pow((-param_.lr * yaw_rate + vy), 2) + std::pow(vx, 2)));
    const double dF_y_vy = ((param_.Br * param_.Cr * param_.Dr * std::cos(param_.Cr * std::atan(param_.Br * alpha_r))) /
                            (1. + std::pow(param_.Br, 2) * std::pow(alpha_r, 2))) *
                           ((-vx) / (std::pow((-param_.lr * yaw_rate + vy), 2) + std::pow(vx, 2)));
    const double dF_y_r = ((param_.Br * param_.Cr * param_.Dr * std::cos(param_.Cr * std::atan(param_.Br * alpha_r))) /
                           (1. + std::pow(param_.Br, 2) * std::pow(alpha_r, 2))) *
                          ((param_.lr * vx) / (std::pow((-param_.lr * yaw_rate + vy), 2) + std::pow(vx, 2)));
    const double dF_y_acc = 0.0;
    const double dF_y_delta = 0.0;

    return {dF_y_vx, dF_y_vy, dF_y_r, dF_y_acc, dF_y_delta, dF_x_vx, dF_x_vy, dF_x_r, dF_x_acc, dF_x_delta};
}

FrictionForceDerivatives DynamicModel::getForceFrictionDerivatives(const DynamicModelState& x) const {
    return {2.0 * param_.Cr2 * x.vx, 0.0, 0.0, 0.0, 0.0};
}

StateVector DynamicModel::NonlinearEvalulate(const DynamicModelState& x, const DynamicModelInput& u) const {
    const double theta = x.theta;
    const double vx = x.vx;
    const double vy = x.vy;
    const double yaw_rate = x.yaw_rate;
    // const double acc = x.acc;
    const double delta = x.delta;

    const double d_acc = u.d_acc;
    const double d_delta = u.d_delta;

    const TireForces tire_forces_front = getForceFront(x);
    const TireForces tire_forces_rear = getForceRear(x);
    const double friction_force = getForceFriction(x);

    StateVector state_dot;
    state_dot(0) = vx * std::cos(theta) - vy * std::sin(theta);
    state_dot(1) = vy * std::cos(theta) + vx * std::sin(theta);
    state_dot(2) = yaw_rate;
    state_dot(3) =
        1.0 / param_.m *
        (tire_forces_rear.F_x + friction_force - tire_forces_front.F_y * std::sin(delta) + param_.m * vy * yaw_rate);
    state_dot(4) =
        1.0 / param_.m * (tire_forces_rear.F_y + tire_forces_front.F_y * std::cos(delta) - param_.m * vx * yaw_rate);
    state_dot(5) =
        1.0 / param_.Iz * (tire_forces_front.F_y * param_.lf * std::cos(delta) - tire_forces_rear.F_y * param_.lr);
    state_dot(6) = d_acc;
    state_dot(7) = d_delta;

    return state_dot;
}

LinModelMatrix DynamicModel::getModelJacobian(const DynamicModelState& x, const DynamicModelInput& u) const {
    // compute jacobian of the model
    // state values
    const double theta = x.theta;
    const double vx = x.vx;
    const double vy = x.vy;
    const double yaw_rate = x.yaw_rate;
    // const double acc = x.acc;
    const double delta = x.delta;
    const double acc_dot = u.d_acc;
    const double delta_dot = u.d_delta;

    A_MPC A_c = A_MPC::Zero();
    B_MPC B_c = B_MPC::Zero();
    g_MPC g_c = g_MPC::Zero();

    const StateVector state_dot = NonlinearEvalulate(x, u);

    const TireForces F_front = getForceFront(x);
    //    TireForces F_rear  = getForceRear(x);

    const TireForcesDerivatives dF_front = getForceFrontDerivatives(x);
    const TireForcesDerivatives dF_rear = getForceRearDerivatives(x);
    const FrictionForceDerivatives dF_xd = getForceFrictionDerivatives(x);

    // Derivatives of function
    // eq1 X_dot = v_x*cos(theta) - v_y*sin(theta)
    const double df1_dtheta = -vx * std::sin(theta) - vy * std::cos(theta);
    const double df1_dvx = std::cos(theta);
    const double df1_dvy = -std::sin(theta);

    // eq2 Y_dot = v_y*cos(theta) + v_x*sin(theta);
    const double df2_dtheta = -vy * std::sin(theta) + vx * std::cos(theta);
    const double df2_dvx = std::sin(theta);
    const double df2_dvy = std::cos(theta);

    // eq3 theta_dot = yaw_rate;
    const double df3_dyaw_rate = 1.0;

    // eq4 vx_dot = 1/mass*(F_rx - F_xd - F_fy*sin(delta) + mass*v_y*r);
    const double df4_dvx = 1.0 / param_.m * (dF_rear.dF_x_vx - dF_xd.dF_f_vx - dF_front.dF_y_vx * std::sin(delta));
    const double df4_dvy = 1.0 / param_.m * (-dF_front.dF_y_vy * std::sin(delta) + param_.m * yaw_rate);
    const double df4_dyaw_rate = 1.0 / param_.m * (-dF_front.dF_y_yaw_rate * std::sin(delta) + param_.m * vy);
    const double df4_dacc = 1.0 / param_.m * dF_rear.dF_x_acc;
    const double df4_ddelta = 1.0 / param_.m * (-dF_front.dF_y_delta * std::sin(delta) - F_front.F_y * std::cos(delta));

    // eq5 vy_dot = 1/mass*(F_ry + F_fy*cos(delta) - mass*v_x*yaw_rate);
    const double df5_dvx =
        1.0 / param_.m * (dF_rear.dF_y_vx + dF_front.dF_y_vx * std::cos(delta) - param_.m * yaw_rate);
    const double df5_dvy = 1.0 / param_.m * (dF_rear.dF_y_vy + dF_front.dF_y_vy * std::cos(delta));
    const double df5_dyaw_rate =
        1.0 / param_.m * (dF_rear.dF_y_yaw_rate + dF_front.dF_y_yaw_rate * std::cos(delta) - param_.m * vx);
    const double df5_ddelta = 1.0 / param_.m * (dF_front.dF_y_delta * std::cos(delta) - F_front.F_y * std::sin(delta));

    // eq6 yaw_rate_dot= 1/Iz*(F_fy*l_f*cos(delta)- F_ry*l_r)
    const double df6_dvx =
        1.0 / param_.Iz * (dF_front.dF_y_vx * param_.lf * std::cos(delta) - dF_rear.dF_y_vx * param_.lr);
    const double df6_dvy =
        1.0 / param_.Iz * (dF_front.dF_y_vy * param_.lf * std::cos(delta) - dF_rear.dF_y_vy * param_.lr);
    const double df6_dyaw_rate =
        1.0 / param_.Iz * (dF_front.dF_y_yaw_rate * param_.lf * std::cos(delta) - dF_rear.dF_y_yaw_rate * param_.lr);
    const double df6_ddelta =
        1.0 / param_.Iz *
        (dF_front.dF_y_delta * param_.lf * std::cos(delta) - F_front.F_y * param_.lf * std::sin(delta));

    // eq7 a_dot = d_acc
    const double df7_dacc = acc_dot;
    // eq8 delta_dot = d_delta
    const double df8_ddelta = delta_dot;

    // Fill in Jacobians
    // Matrix A
    // Column 1 dX
    // all zero
    // Column 2 dY
    // all zero
    // Column 3 dtheta
    A_c(0, 2) = df1_dtheta;
    A_c(1, 2) = df2_dtheta;
    // Column 4 dvx
    A_c(0, 3) = df1_dvx;
    A_c(1, 3) = df2_dvx;
    A_c(3, 3) = df4_dvx;
    A_c(4, 3) = df5_dvx;
    A_c(5, 3) = df6_dvx;
    // Column 5 dvy
    A_c(0, 4) = df1_dvy;
    A_c(1, 4) = df2_dvy;
    A_c(3, 4) = df4_dvy;
    A_c(4, 4) = df5_dvy;
    A_c(5, 4) = df6_dvy;
    // Column 6 dyaw_rate
    A_c(2, 5) = df3_dyaw_rate;
    A_c(3, 5) = df4_dyaw_rate;
    A_c(4, 5) = df5_dyaw_rate;
    A_c(5, 5) = df6_dyaw_rate;
    // Column 7 dacc
    A_c(3, 6) = df4_dacc;
    A_c(6, 6) = df7_dacc;
    // Column 8 ddelta
    A_c(3, 7) = df4_ddelta;
    A_c(4, 7) = df5_ddelta;
    A_c(5, 7) = df6_ddelta;
    A_c(7, 7) = df8_ddelta;

    // Matrix B
    // Column 1 d_dacc
    B_c(6, 0) = 1.0;
    // Column 2 d_ddelat
    B_c(7, 1) = 1.0;

    // Vector State
    StateVector state_c;
    state_c(0) = x.X;
    state_c(1) = x.Y;
    state_c(2) = x.theta;
    state_c(3) = x.vx;
    state_c(4) = x.vy;
    state_c(5) = x.yaw_rate;
    state_c(6) = x.acc;
    state_c(7) = x.delta;

    // Vector Input
    InputVector input_c;
    input_c(0) = u.d_acc;
    input_c(1) = u.d_delta;

    // zero order term
    g_c = state_dot - A_c * state_c - B_c * input_c;

    return {A_c, B_c, g_c};
}

LinModelMatrix DynamicModel::discretizeModel(const LinModelMatrix& lin_model_c) const {
    // disctetize the continuous time linear model \dot x = A x + B u + g using
    // ZHO
    Eigen::Matrix<double, NX + NU + 1, NX + NU + 1> temp = Eigen::Matrix<double, NX + NU + 1, NX + NU + 1>::Zero();
    // building matrix necessary for expm
    // temp = Ts*[A,B,g;zeros]
    temp.block<NX, NX>(0, 0) = lin_model_c.A;
    temp.block<NX, NU>(0, NX) = lin_model_c.B;
    temp.block<NX, 1>(0, NX + NU) = lin_model_c.g;
    temp = temp * Ts_;
    // take the matrix exponential of temp
    const Eigen::Matrix<double, NX + NU + 1, NX + NU + 1> temp_res = temp.exp();
    // extract dynamics out of big matrix
    // x_{k+1} = Ad x_k + Bd u_k + gd
    // temp_res = [Ad,Bd,gd;zeros]
    const A_MPC A_d = temp_res.block<NX, NX>(0, 0);
    const B_MPC B_d = temp_res.block<NX, NU>(0, NX);
    // const g_MPC g_d = temp_res.block<NX, 1>(0, NX + NU);
    const g_MPC g_d = lin_model_c.g * Ts_;  // To ensure the result same as matlab

    return {A_d, B_d, g_d};
}

LinModelMatrix DynamicModel::getLinModel(const DynamicModelState& x, const DynamicModelInput& u) const {
    // compute linearized and discretized model
    const LinModelMatrix lin_model_c = getModelJacobian(x, u);
    // discretize the system
    return discretizeModel(lin_model_c);
    // TODO(zhaobolin): add a check in case NaN in matrix
}

}  // namespace control
}  // namespace jiduauto
