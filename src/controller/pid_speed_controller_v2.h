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

/**
 * @file
 * @brief Defines the PidSpeedControllerV2 class.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "control/src/common/leadlag_controller.h"
#include "control/src/common/pid_controller.h"
#include "control/src/controller/controller.h"
#include "pnc_common/src/math/common/interpolation_1d.h"
#include "pnc_common_vehicle_param.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

enum InterpolationType {
    kVelForseeTime = 0,
    kVelForseeMinDis = 1,
    kVelSpdLoopKp = 2,
    kVelSpdLoopKi = 3,
    kVelSpdLoopKd = 4,
    kVelSpdLoopIntgrSatLvl = 5,
    kVelStatLoopKp = 6,
    kVelStatLoopKi = 7,
    kVelStatLoopKd = 8,
    kVelStatLoopIntgrSatLvl = 9
};

/**
 * @brief PidSpeedControllerV2, using pid for speed control.
 */
class PidSpeedControllerV2 : public Controller {
public:
    PidSpeedControllerV2();
    ~PidSpeedControllerV2() override;

    /**
     * @brief Init pid_speed_controller paras.
     * @return true if succeeded; otherwise false;
     */
    bool Init(const pnc::control::ControlConfig& conf) override;

    /**
     * @brief Main function, compute cmd.
     * @param control_input_stream msg include vehicle state, trajectory and raw topic subscription.
     * @param control_command_stream msg include control command and debug info.
     * @return true if succeeded; otherwise false;
     */
    bool ControlMain(const ControlInputStream& control_input_stream,
                     ControlCommandStream* const control_command_stream) override;

    bool Control(const pnc::localization::LocalizationEstimate* const localization,
                 const pnc::chassis::Chassis* const chassis, const pnc::planning::ADCTrajectory* const trajectory,
                 pnc::control::ControlCommand* const cmd) override;

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
     * @brief check pid controller config valid or not.
     */
    bool PidConfigCheck() const;

    /**
     * @brief Calculate speed command.
     * @param context longitudinal control context.
     * @param trajectory planning info.
     * @param vehicle_state virtual sensors info
     * @param pid_conf pid speed controller config.
     * @return true if succeeded; otherwise false;
     */
    bool ComputeLongitudinalCommand(pnc::control::LongitudinalControlContext* const context,
                                    const pnc::planning::ADCTrajectory& trajectory,
                                    const pnc::common::VehicleState& vehicle_state,
                                    const pnc::control::PidSpeedControllerConf& pid_conf);
    /**
     * @brief Calculate longitudinal station & speed & acceleration error.
     * @param context longitudinal control context.
     * @param trajectory planning info.
     * @param vehicle_state virtual sensors info.
     * @return true if succeeded; otherwise false;
     */
    bool ComputeLongitudinalError(pnc::control::LongitudinalControlContext* const context,
                                  const pnc::planning::ADCTrajectory& trajectory,
                                  const pnc::common::VehicleState& vehicle_state);
    /**
     * @brief Compute compensation value by input error into cascade controller.
     * @param context longitudinal control context.
     * @param pid_conf pid speed controller config.
     * @param ts control period.
     * @return true if succeeded; otherwise false;
     */
    bool ComputeClosedLoopAccel(pnc::control::LongitudinalControlContext* const context,
                                const pnc::control::PidSpeedControllerConf& pid_conf, const double ts);

    /**
     * @brief Generate pedal percentage by input acceleration command.
     * @param context longitudinal control context.
     * @return true if succeeded; otherwise false.
     */
    bool GeneratePedalPercentage(pnc::control::LongitudinalControlContext* const context);

    /**
     * @brief Given linear velocity and get full stop distance in current case.
     * @param context vehicle linear velocity from chassis
     * @param trajectory
     * @param pid_conf
     * @return true if succeeded; otherwise false;
     */
    bool isNearStandstill(pnc::control::LongitudinalControlContext* const context,
                          const pnc::planning::ADCTrajectory& trajectory,
                          const pnc::control::PidSpeedControllerConf& pid_conf);

    /**
     * @brief Load pid calibration table from config file and initialize linear interpolation.
     * @param pid_conf Given speed, calculate Pid paras by applying interpolation.
     * @return true if succeeded; otherwise false;
     */
    bool LoadPIDCalibrationTable(const pnc::control::PidSpeedControllerConf& pid_conf);

    /**
     * @brief Given linear velocity of vehicle, calculate Pid paras by applying interpolation.
     * @param pid_conf
     * @param curr_velocity
     * @param context
     * @return true if succeeded; otherwise false;
     */
    bool UpdatePIDInterpolatedParams(const pnc::control::PidSpeedControllerConf& pid_conf, const double curr_velocity,
                                     pnc::control::LongitudinalControlContext* const context);

    /**
     * @brief Calculate slope acceleration compensation based on gravity and current pitch angle.
     * @param vehicle_state
     * @return acceleration compensation.
     */
    const double SlopeOffsetCompensation(const pnc::common::VehicleState& vehicle_state);

    /**
     * @brief Calculate open loop deceleration by full-stop time and current speed.
     * @param context
     * @return deceleration value.
     */
    const double GetOpenLoopDecel(const pnc::control::LongitudinalControlContext* context);

    /**
     * @brief Init log_file_ handle.
     */
    bool SummaryDebugInfo(const int32_t time_elapsed, const pnc::control::LongitudinalControlContext& context,
                          pnc::control::ControlDebug* const control_debug) const;

private:
    // conf
    pnc::control::ControlConfig control_conf_;
    pnc::common::VehicleParam vehicle_params_;

    // common
    bool is_controller_initialized_{false};
    bool is_current_backward_{false};

    // msg

    // controllers
    PIDController station_pid_controller_;
    PIDController speed_pid_controller_;
    LeadlagController station_leadlag_controller_;
    LeadlagController speed_leadlag_controller_;

    // interpolation
    std::unordered_map<int32_t, std::unique_ptr<pnc::interpolation::Interpolation1D>> interpolator_list_;
};

}  // namespace control
}  // namespace jiduauto
