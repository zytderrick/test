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
 * @brief Defines the ChassisCheck class.
 */
#pragma once
#include <queue>
#include <string>
#include <vector>

#include "control/src/common/control_message_stream.h"
#include "control/src/feature/diagnosis/control_dependent_node/node_check.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_control_config.pb.h"
#include "pnc_debug_control.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

class ChassisCheck : public NodeCheck {
    struct GeneralCheckParas {
        int count_timestamp_check{0};
        double last_time_interval{0.0};
        double last_timestamp{jiduauto::pnc::util::TimeUtility::GetCurrentTimesecond()};
    };

    struct SpecificCheckParas {
        std::queue<double> chassis_velocity_queue;
        std::queue<double> chassis2localization_velocity_error_queue;
    };
    struct GlobalParas {
        GeneralCheckParas general_check_paras;
        SpecificCheckParas specific_check_paras;
        pnc::control::DiagnosisReturnCode is_general_check_passed{
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP};
        std::vector<std::string> check_result_string_vector;
        int chassis_check_code{0};
        std::queue<int> chassis_check_code_queue;
    };

public:
    ChassisCheck() = default;
    ~ChassisCheck() = default;

    bool Init(const pnc::control::ControlConfig& control_conf) override;

    bool GeneralCheck(const InputStream& input_stream, pnc::control::DiagnosisDebugV2* const diag_debug_ptr) override;

    bool SpecificCheck(const ControlInputStream& control_input_stream,
                       pnc::control::DiagnosisDebugV2* const diag_debug_ptr) override;

    /**
     * @brief reset NodeCheck
     */
    void Reset() override;

    /**
     * @brief NodeCheck name
     * @return string NodeCheck name in string
     */
    std::string Name() const override;

private:
    bool SanityCheck(const pnc::control::ControlConfig& control_conf);

    pnc::control::DiagnosisReturnCode TimestampCheck(const double timestamp_sec,
                                                     GeneralCheckParas* const general_check_paras_ptr);

    pnc::control::DiagnosisReturnCode VelocityCheck(const ControlInputStream& control_input_stream,
                                                    SpecificCheckParas* const specific_check_paras_ptr);

    double GetQueueMean(const std::queue<double>& q);

    void SetDebugInfo(const GlobalParas& global_param, pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

    void UpdateCodeQueue(GlobalParas* const global_param);

private:
    // conf
    pnc::control::ControlConfig control_conf_;

    // struct
    GlobalParas global_paras_;
};

}  // namespace control
}  // namespace jiduauto
