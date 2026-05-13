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

#include "pnc_pid_config.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

/**
 * @class PIDController
 * @brief A proportional-integral-derivative controller for speed and steering
 using defualt integral hold
 */
class PIDController {
public:
    // PIDController(const PIDController& other) = default;
    // PIDController(PIDController&& other) noexcept = default;
    // PIDController& operator=(const PIDController& other) = default;
    // PIDController& operator=(PIDController&& other) noexcept = default;
    /**
     * @brief initialize pid controller
     * @param pid_conf configuration for pid controller
     */
    void Init(const pnc::control::PIDConfig& pid_conf);

    /**
     * @brief set pid controller coefficients for the proportional,
     * integral, and derivative
     * @param pid_conf configuration for pid controller
     */
    void SetPID(const pnc::control::PIDConfig& pid_conf);

    /**
     * @brief reset variables for pid controller
     */
    void Reset() noexcept;

    /**
     * @brief compute control value based on the error
     * @param error error value, the difference between
     * a desired value and a measured value
     * @param dt sampling time interval
     * @return control value based on PID terms
     */
    virtual double Control(const double error, const double dt);

    virtual ~PIDController() = default;

    /**
     * @brief get saturation status
     * @return saturation status
     */
    inline int32_t IntegratorSaturationStatus() const noexcept { return integrator_saturation_status_; }

    inline int32_t OutputSaturationStatus() const noexcept { return output_saturation_status_; }

    /**
     * @brief get status that if integrator is hold
     * @return if integrator is hold return true
     */
    bool IntegratorHold() const;

    /**
     * @brief set whether to hold integrator component at its current value.
     * @param hold
     */
    void SetIntegratorHold(bool hold) noexcept;

protected:
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double kaw_{0.0};
    double previous_error_{0.0};
    double previous_output_{0.0};
    double integral_{0.0};
    double integrator_saturation_high_{0.0};
    double integrator_saturation_low_{0.0};
    bool first_hit_{false};
    bool integrator_enabled_{false};
    bool integrator_hold_{false};
    int32_t integrator_saturation_status_{0};
    // Only used for pid_BC_controller and pid_IC_controller
    double output_saturation_high_{0.0};
    double output_saturation_low_{0.0};
    int32_t output_saturation_status_{0};
};

}  // namespace control
}  // namespace jiduauto
