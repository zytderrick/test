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
 * @brief Defines the ActuatorCheck class.
 */
#pragma once

#include <list>
#include <queue>
#include <string>
#include <vector>

#include "control/src/common/control_message_stream.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common_vehicle_param.pb.h"
#include "pnc_control_config.pb.h"
#include "pnc_debug_control.pb.h"

/**
 * @namespace jiduauto::control
 */

namespace jiduauto {
namespace control {

class ActuatorCheck {
    struct SteerAngleOffsetParas {
        std::queue<double> speed_que;
        std::queue<double> heading_que;
        std::queue<double> steering_angle_que;
        std::queue<double> steering_offset_que;
        double steer_angle_offset_mean{0.0};
        double temp_steer_angle_offset{0.0};
    };

    struct GlobalParas {
        SteerAngleOffsetParas steer_angle_offset_paras;
        std::vector<std::string> check_result_string_vector;
        int actuator_check_code{0};
    };

public:
    ActuatorCheck() = default;
    ~ActuatorCheck() = default;

    /**
     * @brief init
     */
    bool Init(const pnc::control::ControlConfig& control_conf);

    void Reset();

    bool SanityCheck(const pnc::control::ControlConfig& control_conf);

    bool FaultCheck(const ControlInputStream& control_input_stream,
                    pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

private:
    bool SteerAngleOffsetCheck(const jiduauto::pnc::common::VehicleState& vehicle_state,
                               pnc::control::SteerAngleOffsetDebug* const steer_angle_offset_debug_ptr);

    bool LongitudinalCheckPass(const pnc::common::VehicleState& vehicle_state,
                               const pnc::control::ControlCommand& last_cmd,
                               pnc::control::AccelerationPedalDebug* const acc_pedal_debug_ptr);

    bool GearResponseCheckPass(const pnc::common::VehicleState& vehicle_state,
                               const pnc::control::ControlCommand& last_cmd,
                               pnc::control::GearDebug* const gear_debug_ptr);

    bool SteerResponseCheckPass(const pnc::common::VehicleState& vehicle_state,
                                const pnc::control::ControlCommand& last_cmd,
                                pnc::control::SteerDebug* const steer_debug_ptr);

    bool ChassisReadyCheckPass(const pnc::common::VehicleState& vehicle_state,
                               pnc::control::ChassisReadyDebug* const chassis_ready_debug);

    bool PadmsgStartResponseCheckPass(const pnc::common::VehicleState& vehicle_state,
                                      const pnc::control::ControlCommand& last_cmd,
                                      pnc::control::PadmsgResponseDebug* const padmsg_response_debug);

    void ClearQueue(std::queue<double>* q_ptr);

    double GetQueueMean(std::queue<double> q);

    void SetCodeBitHigh(int* const check_code, const int& bit) { (*check_code) |= (0x00000001 << (bit - 1)); }

    void SetCodeBitLow(int* const check_code, const int& bit) { (*check_code) &= ~(0x00000001 << (bit - 1)); }

private:
    // conf
    pnc::control::ControlConfig control_conf_;
    pnc::common::VehicleParam veh_paras_;

    // struct
    GlobalParas global_paras_;
    std::vector<double> acc_cmd_vec_;
    std::vector<double> acc_feedback_vec_;
    std::vector<double> speed_vec_;
    std::vector<double> time_sec_vec_;
    std::list<bool> gear_match_list_;
    std::list<bool> steer_match_list_;
    std::list<bool> manual_driving_list_;

    bool bflag_first_chassis_unready_{true};
    double first_chassis_unready_timestamp_{jiduauto::pnc::util::TimeUtility::GetCurrentTimesecond()};
};
}  // namespace control
}  // namespace jiduauto
