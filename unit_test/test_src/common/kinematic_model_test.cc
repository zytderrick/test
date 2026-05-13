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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace jiduauto {
namespace control {

TEST(KinematicModel, GetNextState1) {
    const double Ts = 0.1;
    KinematicModel kinematic_model(Ts, 2.5);
    KinematicModelState x;
    x.X = 1.0;
    x.Y = 1.0;
    x.theta = 0.0;
    x.speed = 2.0;
    x.delta = 0.0;
    KinematicModelInput u;
    u.acc = 1.0;
    u.delta_rate = 0.0;

    KinematicStateVector state_dot = kinematic_model.NonlinearEvalulate(x, u);
    EXPECT_EQ(state_dot(0) * Ts + x.X, 1.2);
    EXPECT_EQ(state_dot(1) * Ts + x.Y, 1.0);
    EXPECT_EQ(state_dot(2) * Ts + x.theta, 0.0);
    EXPECT_EQ(state_dot(3) * Ts + x.speed, 2.1);
}

TEST(KinematicModel, GetNextState2) {
    const double Ts = 0.1;
    KinematicModel kinematic_model(Ts, 2.5);
    KinematicModelState x;
    x.X = 3.0;
    x.Y = 1.0;
    x.theta = 0.2;
    x.speed = 2.0;
    x.delta = 0.0;
    KinematicModelInput u;
    u.acc = 1.0;
    u.delta_rate = 0.0;

    KinematicStateVector state_dot = kinematic_model.NonlinearEvalulate(x, u);
    EXPECT_NEAR(state_dot(0) * Ts + x.X, 3.196, 0.01);
    EXPECT_NEAR(state_dot(1) * Ts + x.Y, 1.0397, 0.01);
    EXPECT_EQ(state_dot(2) * Ts + x.theta, 0.2);
    EXPECT_EQ(state_dot(3) * Ts + x.speed, 2.1);
}

TEST(KinematicModel, GetNextState3) {
    const double Ts = 0.1;
    KinematicModel kinematic_model(Ts, 2.5);
    KinematicModelState x;
    x.X = 3.0;
    x.Y = 1.0;
    x.theta = 0.2;
    x.speed = 2.0;
    x.delta = 0.1;
    KinematicModelInput u;
    u.acc = 1.0;
    u.delta_rate = 0.0;

    KinematicStateVector state_dot = kinematic_model.NonlinearEvalulate(x, u);
    EXPECT_NEAR(state_dot(0) * Ts + x.X, 3.196, 0.01);
    EXPECT_NEAR(state_dot(1) * Ts + x.Y, 1.0397, 0.01);
    EXPECT_NEAR(state_dot(2) * Ts + x.theta, 0.208, 0.01);
    EXPECT_EQ(state_dot(3) * Ts + x.speed, 2.1);
}

TEST(KinematicModel, GetLinearModel) {
    double Ts = 0.1;
    KinematicModel kinematic_model(Ts, 2.5);
    KinematicModelState x;
    x.X = 3.0;
    x.Y = 1.0;
    x.theta = 0.2;
    x.speed = 2.0;
    x.delta = 0.1;
    KinematicModelInput u;
    u.acc = 1.0;
    u.delta_rate = 0.0;

    KinLinModelMatrix discrete_model = kinematic_model.getDiscreteLinModel(x, u);
    EXPECT_NEAR(discrete_model.A(0, 2), -0.0397, 0.01);
    EXPECT_NEAR(discrete_model.A(0, 3), 0.0980, 0.01);
    EXPECT_NEAR(discrete_model.A(1, 2), 0.196, 0.01);
    EXPECT_NEAR(discrete_model.A(1, 3), 0.0199, 0.01);
    EXPECT_NEAR(discrete_model.A(2, 3), 0.004, 0.01);
    EXPECT_EQ(discrete_model.A(3, 3), 1.0);

    EXPECT_NEAR(discrete_model.B(3, 0), Ts, 0.01);
    EXPECT_NEAR(discrete_model.B(4, 1), Ts, 0.01);
}

}  // namespace control
}  // namespace jiduauto
