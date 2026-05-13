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

#include "control/src/common/dynamic_model/params.h"
#include "control/src/common/dynamic_model/types.h"

namespace jiduauto {
namespace control {

struct LinModelMatrix {
    A_MPC A;
    B_MPC B;
    g_MPC g;
};

struct TireForces {
    const double F_y;
    const double F_x;
};

struct NormalForces {
    const double F_N_front;
    const double F_N_rear;
};

struct TireForcesDerivatives {
    const double dF_y_vx;
    const double dF_y_vy;
    const double dF_y_yaw_rate;
    const double dF_y_acc;
    const double dF_y_delta;

    const double dF_x_vx;
    const double dF_x_vy;
    const double dF_x_yaw_rate;
    const double dF_x_acc;
    const double dF_x_delta;
};

struct FrictionForceDerivatives {
    const double dF_f_vx;
    const double dF_f_vy;
    const double dF_f_yaw_rate;
    const double dF_f_acc;
    const double dF_f_delta;
};

class DynamicModel {
public:
    DynamicModel(const double sample_time, const Param model_param) : Ts_(sample_time), param_(model_param) {}

    double getSlipAngleFront(const DynamicModelState& x) const;
    double getSlipAngleRear(const DynamicModelState& x) const;

    TireForces getForceFront(const DynamicModelState& x) const;
    TireForces getForceRear(const DynamicModelState& x) const;
    double getForceFriction(const DynamicModelState& x) const;
    NormalForces getForceNormal(const DynamicModelState& x) const;

    TireForcesDerivatives getForceFrontDerivatives(const DynamicModelState& x) const;
    TireForcesDerivatives getForceRearDerivatives(const DynamicModelState& x) const;
    FrictionForceDerivatives getForceFrictionDerivatives(const DynamicModelState& x) const;

    StateVector NonlinearEvalulate(const DynamicModelState& x, const DynamicModelInput& u) const;

    LinModelMatrix getLinModel(const DynamicModelState& x, const DynamicModelInput& u) const;

private:
    LinModelMatrix getModelJacobian(const DynamicModelState& x, const DynamicModelInput& u) const;
    LinModelMatrix discretizeModel(const LinModelMatrix& lin_model_conti) const;

    const double Ts_;
    Param param_;
};

}  // namespace control
}  // namespace jiduauto
