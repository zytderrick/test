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

#include "control/src/common/leadlag_controller.h"

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include "control/src/common/control_gflags.h"
#include "control/src/common/file_util.h"
#include "pnc_common/src/utility/pnc_utility.h"
#include "pnc_control_config.pb.h"
#include "pnc_leadlag_conf.pb.h"

namespace jiduauto {
namespace control {

class LeadlagControllerTest : public ::testing::Test {
public:
    virtual void SetUp() {
        CHECK(FileUtil::ReadPnCPath());
        std::string control_conf_file = FLAGS_control_unit_test_data_path + FLAGS_control_unit_test_conf_file;
        CHECK(pnc::util::PnCUtility::LoadTextFile(control_conf_file, &control_conf_));
        pid_speed_controller_conf_ = control_conf_.pid_speed_controller_conf();
    }

protected:
    pnc::control::ControlConfig control_conf_;
    pnc::control::PidSpeedControllerConf pid_speed_controller_conf_;
};

TEST_F(LeadlagControllerTest, StationLeadlagController) {
    double dt = 0.01;
    pnc::control::LeadlagConf leadlag_conf = pid_speed_controller_conf_.reverse_station_leadlag_conf();
    LeadlagController leadlag_controller;
    leadlag_controller.Init(leadlag_conf, dt);
    leadlag_controller.Reset();
    EXPECT_NEAR(leadlag_controller.Control(0.0, dt), 0.0, 1e-6);
    leadlag_controller.Reset();
    EXPECT_NEAR(leadlag_controller.Control(0.1, dt), 0.42, 1e-6);
    leadlag_controller.Reset();
    double control_value = leadlag_controller.Control(-0.1, dt);
    EXPECT_NEAR(control_value, -0.42, 1e-6);
    dt = 0.0;
    EXPECT_EQ(leadlag_controller.Control(100, dt), control_value);
}

TEST_F(LeadlagControllerTest, SpeedLeadlagController) {
    double dt = 0.01;
    pnc::control::LeadlagConf leadlag_conf = pid_speed_controller_conf_.reverse_speed_leadlag_conf();
    LeadlagController leadlag_controller;
    leadlag_controller.Init(leadlag_conf, dt);
    leadlag_controller.Reset();
    EXPECT_NEAR(leadlag_controller.Control(0.0, dt), 0.0, 1e-6);
    leadlag_controller.Reset();
    EXPECT_NEAR(leadlag_controller.Control(0.1, dt), 0.2625, 1e-6);
    leadlag_controller.Reset();
    EXPECT_NEAR(leadlag_controller.Control(-0.1, dt), -0.2625, 1e-6);
    leadlag_controller.Reset();
    EXPECT_NEAR(leadlag_controller.Control(500.0, dt), 6.3, 1e-6);
    EXPECT_EQ(leadlag_controller.InnerstateSaturationStatus(), 1);
    leadlag_controller.Reset();
    EXPECT_NEAR(leadlag_controller.Control(-500.0, dt), -6.3, 1e-6);
    EXPECT_EQ(leadlag_controller.InnerstateSaturationStatus(), -1);
}

}  // namespace control
}  // namespace jiduauto
