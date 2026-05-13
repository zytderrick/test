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

// Virtual Sensors Module
// Refer to https://wiki.jiduauto.com/display/~bolin.zhao/Control+Overall

#pragma once

#include <list>
#include <memory>
#include <mutex>
#include <vector>

#include "control/src/common/control_message_stream.h"
#include "control/src/common/interpolation_1d.h"
#include "pnc_canbus_chassis.pb.h"
#include "pnc_common/src/math/filters/digital_filter.h"
#include "pnc_common/src/math/filters/kalman_filter.h"
#include "pnc_common/src/math/filters/mean_filter.h"
#include "pnc_common_vehicle_param.pb.h"
#include "pnc_common_vehicle_state.pb.h"
#include "pnc_control_config.pb.h"
#include "pnc_control_preprocess_conf.pb.h"
#include "pnc_localization_estimate.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

struct TerminalSteerProcessParams {
    std::vector<double> steer_torque_vec;
    std::list<bool> steady_steer_angle_list;
    std::list<bool> terminal_swa_hold_list;
    bool bflag_steady_steer_torque{false};
    double steer_torque_mean{0.0};
    std::unique_ptr<Interpolation1D> terminal_steer_interpolation = std::make_unique<Interpolation1D>();
};

class VirtualSensors {
public:
    /**
     * @brief Default constructor.
     */
    VirtualSensors() = default;

    /**
     * @brief destructor.
     */
    ~VirtualSensors() = default;

    /**std::mutex chassis_mutex_{};
     * @brief filtered data of input stream.
     */
    const pnc::common::VehicleState& GetSenseView() const;

    bool Reset();

    bool Init(const pnc::control::ControlConfig& control_conf);

    /**
     * @brief Get pretreated information of localization and chassis.
     * @param input_stream information from subscribed from other modules.
     */
    void Update(const InputStream& input_stream, pnc::control::ControlDebug* const debug);

private:
    void ConstructPositionAndLocSpd(const InputStream& input_stream, pnc::common::VehicleState* const sense_view,
                                    const pnc::control::DiagnosisDebugV2 diag_result);

    void ConstructEulerAnglesAndRate(const InputStream& input_stream, pnc::common::VehicleState* const sense_view,
                                     const pnc::control::DiagnosisDebugV2 diag_result);

    void ConstructChassis(const InputStream& input_stream, pnc::common::VehicleState* const sense_view,
                          const pnc::control::DiagnosisDebugV2 diag_result);

    void ConstructTrajectory(const InputStream& input_stream, pnc::common::VehicleState* const sense_view,
                             const pnc::control::DiagnosisDebugV2 diag_result);

    void ConstructHistoryCmd(const InputStream& input_stream, pnc::common::VehicleState* const sense_view,
                             const pnc::control::DiagnosisDebugV2 diag_result);

    void ConstructTimestamp(const InputStream& input_stream, pnc::common::VehicleState* const sense_view,
                            const pnc::control::DiagnosisDebugV2 diag_result);

    void ConstructStandstill(const InputStream& input_stream, pnc::common::VehicleState* const sense_view,
                             const pnc::control::DiagnosisDebugV2 diag_result);

    void ConstructModeAndGear(const InputStream& input_stream, pnc::common::VehicleState* const sense_view,
                              const pnc::control::DiagnosisDebugV2 diag_result);

    const bool IsStandStill(double localization_velocity, double chassis_velocity);

    const bool IsRequireStandStillSteering(double current_steering_angle, double last_cmd, bool standstill,
                                           const pnc::planning::ADCTrajectory& planning_info, double* standingstill_swa,
                                           double* mean_effect_kappa);

    void ConstructTerminalSteering(const InputStream& input_stream, const pnc::control::DiagnosisDebugV2 diag_result,
                                   pnc::common::VehicleState* const sense_view);

    bool LoadSteerTorqueScheduler();

    bool ResetKalmanFilter();

    pnc::control::VirtualSensorsConf virtual_sensors_conf_;
    pnc::common::VehicleParam vehicle_paras_;
    pnc::common::VehicleState sense_view_;
    pnc::control::CommandPostProcessConf post_process_conf_;
    // FILE* state_log_file_{nullptr};

    double previous_timestamp_sec_{0.0};
    bool last_backward_{false};
    bool enable_csv_debug_{false};
    bool is_kf_reset_{false};

    bool last_standstill_swa_reach_flag_ = false;
    int32_t first_reach_hold_cout_ = 0;
    std::list<bool> reach_flag_list_;
    double last_steering_angle_ = 0.0;

    // filter
    jiduauto::pnc::filter::KalmanFilter<double, 6, 6, 3> position_filter_;
    pnc::filter::DigitalFilter standstill_steer_angle_filter_;

    std::list<bool> standstill_list_;

    TerminalSteerProcessParams terminal_steer_process_params_;
};
}  // namespace control
}  // namespace jiduauto
