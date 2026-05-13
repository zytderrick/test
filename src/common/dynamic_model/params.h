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

namespace jiduauto {
namespace control {

#define NX 8  // X_local,Y_local,Theta_local, vx_vrf, vy_vtf,yaw_rate, acc, delta
#define NU 2  // d_acc, d_delta

static constexpr int N = 10;
static constexpr double INF = 1E5;

struct Param {
    double Cr2 = 0.5359;

    double Br = 10.2;
    double Cr = 1.6;
    double Dr = 4400;

    double Bf = 10.2;
    double Cf = 1.6;
    double Df = 4000;

    double m = 1790;
    double Iz = 3890;
    double lf = 1.4;
    double lr = 1.5;

    double car_l = 5;
    double car_w = 2.5;

    double g = 9.81;

    double r_in;
    double r_out;

    double max_dist_proj;

    double e_long;
    double e_eps;

    double max_alpha;

    double initial_velocity;
    double s_trust_region;

    double vx_zero;
};

}  // namespace control
}  // namespace jiduauto
