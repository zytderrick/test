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

#include <string>

#include "control/src/controller/controller.h"
#include "pnc_chassis_test_config.pb.h"
#include "pnc_common_vehicle_param.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

/**
 * @brief ChassisTestController.
 * Only receive chassis, and send cmd based on pnc::control::ChassisTestConf.
 */
class ChassisTestController : public Controller {
public:
    ChassisTestController();
    ~ChassisTestController() override;

    bool Init(const pnc::control::ControlConfig& control_conf) override;

    /**
     * @brief localization, trajectory invalid input;
     * @param chassis valid input
     * @param cmd valid output
     */
    bool Control(const pnc::localization::LocalizationEstimate* localization, const pnc::chassis::Chassis* chassis,
                 const pnc::planning::ADCTrajectory* trajectory, pnc::control::ControlCommand* const cmd) override;

    bool ControlMain(const ControlInputStream& control_input_stream, ControlCommandStream* const cmd) override;

    /**
     * @brief reset Controller
     * @return true if succeeded; otherwise false;
     */
    bool Reset() override;

    /**
     * @brief controller name
     * @return string controller name in string
     */
    std::string Name() const override;

    /**
     * @brief stop controller
     */
    void Stop() override;

private:
    /**
     * @brief check conf valid or not.
     */
    bool SanityCheck() const;

    /**
     * @brief update latest chassis msg.
     */
    void UpdateChassisStatus();

    /**
     * @brief process LonStandStillTest
     */
    bool LonStandStillTest();

    /**
     * @brief process LonIdleTest
     */
    bool LonIdleSpeedTest();

    /**
     * @brief process LonStepTest
     */
    bool LonStepTest();

    /**
     * @brief process LonSinusoidTest
     */
    bool LonSinusoidTest();

    /**
     * @brief process LatStandStillTest
     */
    bool LatStandStillTest();

    /**
     * @brief process LatStepTest
     */
    bool LatStepTest();

    /**
     * @brief process LatSinusoidTest
     */
    bool LatSinusoidTest();

    /**
     * @brief Init log_file_ handle.
     */
    bool InitLogFile();

private:
    const double kStopAcc_{-1.0};
    double control_period_ = 0.02;
    bool controller_initialized_{false};

    FILE* log_file_{nullptr};
    pnc::control::ChassisTestConf chassis_test_conf_;

    uint64_t prev_time_{0};

    const pnc::chassis::Chassis* chassis_msg_{nullptr};
    const pnc::control::ControlCommand* cmd_msg_{nullptr};

    // conf
    pnc::common::VehicleParam veh_paras_;

    // feedback
    struct feedback {
        pnc::chassis::DrivingMode curr_mode_{pnc::chassis::COMPLETE_MANUAL};
        pnc::chassis::GearPosition curr_gear_{pnc::chassis::GEAR_INVALID};
        float curr_speed_mps_{0.0};
        float prev_speed_mps_{0.0};
        float curr_speed_kph_{0.0};
        float curr_accel_{0.0};  // need to compute based on speed_mps
        float curr_angle_{0.0};
        float prev_angle_{0.0};
        float curr_angle_rate_{0.0};  // need to compute based on steer angle
    } chassis_status_;

    // cmd
    struct cmd {
        pnc::chassis::DrivingMode cmd_mode_{pnc::chassis::COMPLETE_MANUAL};
        pnc::chassis::GearPosition cmd_gear_{pnc::chassis::GEAR_PARKING};
        float cmd_accel_{0.0};
        float cmd_angle_{0.0};
        float cmd_angle_rate_{0.0};
    } control_cmd_;
};

}  // namespace control
}  // namespace jiduauto
