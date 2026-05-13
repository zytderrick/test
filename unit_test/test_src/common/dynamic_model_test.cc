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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "control/src/common/dynamic_model/params.h"

namespace jiduauto {
namespace control {

TEST(DynamicModelTest, GetNextState1) {
    Param model_param;
    const double Ts = 0.05;
    DynamicModel dynamic_model(Ts, model_param);

    DynamicModelState x;
    x.X = 5.0;
    x.Y = 6.0;
    x.theta = 0.2;
    x.vx = 2.0;
    x.vy = 0.2;
    x.yaw_rate = 0.1;
    x.acc = 0.1;
    x.delta = 0.1;
    DynamicModelInput u;
    u.d_acc = -0.1;
    u.d_delta = -0.05;

    StateVector state_dot = dynamic_model.NonlinearEvalulate(x, u);
    EXPECT_NEAR(state_dot(0), 1.9204, 0.02);
    EXPECT_NEAR(state_dot(1), 0.5934, 0.02);
    EXPECT_NEAR(state_dot(2), 0.1, 0.02);
    EXPECT_NEAR(state_dot(3), 0.3040, 0.02);
    EXPECT_NEAR(state_dot(4), -2.9957, 0.02);
    EXPECT_NEAR(state_dot(5), -0.5255, 0.02);
    EXPECT_NEAR(state_dot(6), -0.1, 0.02);
    EXPECT_NEAR(state_dot(7), -0.05, 0.02);
}

TEST(DynamicModel, GetLinearModel) {
    Param model_param;
    const double Ts = 0.05;
    DynamicModel dynamic_model(Ts, model_param);
    DynamicModelState x;
    x.X = 5.0;
    x.Y = 6.0;
    x.theta = 0.2;
    x.vx = 2.0;
    x.vy = 0.2;
    x.yaw_rate = 0.1;
    x.acc = 0.1;
    x.delta = 0.1;
    DynamicModelInput u;
    u.d_acc = -0.1;
    u.d_delta = -0.05;

    LinModelMatrix discrete_model = dynamic_model.getLinModel(x, u);
    EXPECT_NEAR(discrete_model.A(0, 0), 1.0, 0.01);
    EXPECT_NEAR(discrete_model.A(0, 2), -0.0297, 0.01);
    EXPECT_NEAR(discrete_model.A(0, 3), 0.0486, 0.01);
    EXPECT_NEAR(discrete_model.A(0, 4), -0.0053, 0.01);
    EXPECT_NEAR(discrete_model.A(0, 5), -0.0012, 0.01);
    EXPECT_NEAR(discrete_model.A(0, 6), 0.0012, 0.01);
    EXPECT_NEAR(discrete_model.A(0, 7), -0.0019, 0.01);

    EXPECT_NEAR(discrete_model.A(1, 1), 1.0, 0.01);
    EXPECT_NEAR(discrete_model.A(1, 2), 0.096, 0.01);
    EXPECT_NEAR(discrete_model.A(1, 3), 0.0113, 0.01);
    EXPECT_NEAR(discrete_model.A(1, 4), 0.0299, 0.01);
    EXPECT_NEAR(discrete_model.A(1, 5), 0.0106, 0.01);
    EXPECT_NEAR(discrete_model.A(1, 6), 0.0003, 0.01);
    EXPECT_NEAR(discrete_model.A(1, 7), 0.0141, 0.01);

    EXPECT_NEAR(discrete_model.A(2, 3), 0.0005, 0.01);
    EXPECT_NEAR(discrete_model.A(2, 4), 0.0046, 0.05);
    EXPECT_NEAR(discrete_model.A(2, 5), 0.0301, 0.01);
    EXPECT_NEAR(discrete_model.A(2, 7), 0.0092, 0.01);

    EXPECT_NEAR(discrete_model.A(3, 3), 0.9959, 0.01);
    EXPECT_NEAR(discrete_model.A(3, 4), 0.0283, 0.01);
    EXPECT_NEAR(discrete_model.A(3, 5), 0.0409, 0.01);
    EXPECT_NEAR(discrete_model.A(3, 6), 0.0498, 0.01);
    EXPECT_NEAR(discrete_model.A(3, 7), 0.0442, 0.01);

    EXPECT_NEAR(discrete_model.A(4, 3), 0.0478, 0.01);
    EXPECT_NEAR(discrete_model.A(4, 4), 0.3455, 0.01);
    EXPECT_NEAR(discrete_model.A(4, 5), 0.2332, 0.01);
    EXPECT_NEAR(discrete_model.A(4, 6), 0.0014, 0.01);
    EXPECT_NEAR(discrete_model.A(4, 7), 0.4972, 0.01);

    EXPECT_NEAR(discrete_model.A(5, 3), 0.0196, 0.01);
    EXPECT_NEAR(discrete_model.A(5, 4), 0.1216, 0.01);
    EXPECT_NEAR(discrete_model.A(5, 5), 0.3458, 0.01);
    EXPECT_NEAR(discrete_model.A(5, 6), 0.0005, 0.01);
    EXPECT_NEAR(discrete_model.A(5, 7), 0.3334, 0.01);

    EXPECT_NEAR(discrete_model.A(6, 6), 0.9950, 0.01);
    EXPECT_NEAR(discrete_model.A(7, 7), 0.9975, 0.01);

    EXPECT_NEAR(discrete_model.B(0, 0), 0.00002, 0.001);
    EXPECT_NEAR(discrete_model.B(0, 1), -0.0000356, 0.001);
    EXPECT_NEAR(discrete_model.B(1, 1), 0.00024, 0.001);
    EXPECT_NEAR(discrete_model.B(2, 1), 0.00016, 0.01);
    EXPECT_NEAR(discrete_model.B(3, 1), 0.0012, 0.01);
    EXPECT_NEAR(discrete_model.B(4, 1), 0.0139, 0.01);
    EXPECT_NEAR(discrete_model.B(5, 1), 0.0092, 0.01);
    EXPECT_NEAR(discrete_model.B(6, 0), 0.0499, 0.01);
    EXPECT_NEAR(discrete_model.B(7, 1), 0.0499, 0.01);

    EXPECT_NEAR(discrete_model.g(0, 0), 0.0059, 0.01);
    EXPECT_NEAR(discrete_model.g(1, 0), -0.0192, 0.01);
    EXPECT_NEAR(discrete_model.g(3, 0), 0.0062, 0.01);
    EXPECT_NEAR(discrete_model.g(4, 0), -0.1985, 0.01);
    EXPECT_NEAR(discrete_model.g(5, 0), -0.0706, 0.01);
}

}  // namespace control
}  // namespace jiduauto
