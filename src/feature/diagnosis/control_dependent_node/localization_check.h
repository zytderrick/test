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
 * @brief Defines the LocalizationCheck class.
 */
#pragma once

#include <queue>
#include <string>
#include <vector>

#include "control/src/common/control_message_stream.h"
#include "control/src/feature/diagnosis/control_dependent_node/node_check.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common_vehicle_param.pb.h"
#include "pnc_control_config.pb.h"
#include "pnc_debug_control.pb.h"
/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

class LocalizationCheck : public NodeCheck {
    struct GeneralCheckParas {
        int count_timestamp_check{0};  // default: 0
        double last_time_interval{0.0};
        double last_timestamp{jiduauto::pnc::util::TimeUtility::GetCurrentTimesecond()};
    };

    enum LocalizationJumpReturnCode {
        NORMAL = 0,
        SKIP = 1,
        FIRST_LEVEL_JUMP = 2,
        SECOND_LEVEL_JUMP = 3,
        ACCUMULATED_ERROR = 4
    };
    struct LocalizationNotJumpParas {
        bool pre_excute_localization_jump_check{false};  // default: false
        bool excute_localization_jump_check{false};      // default: false
        int localization_jump_count{0};                  // default: 0
        double pre_x_pos{0.0};
        double pre_y_pos{0.0};
        double pre_heading{0.0};
        double pre_speed{0.0};
        double pre_acc{0.0};
        double pre_wheel_steer_angle_rad{0.0};
        LocalizationJumpReturnCode localization_jump_return_code{NORMAL};
        std::queue<double> distance_error_queue;
        bool localization_xy_jump{false};
        bool localization_heading_jump{false};
        double accumulated_distance_error{0.0};
    };
    struct SpecificCheckParas {
        LocalizationNotJumpParas localization_not_jump_paras;
    };
    struct GlobalParas {
        GeneralCheckParas general_check_paras;
        SpecificCheckParas specific_check_paras;
        pnc::control::DiagnosisReturnCode is_general_check_passed{
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP};
        std::vector<std::string> check_result_string_vector;
        int localization_check_code{0};
        std::queue<int> localization_check_code_queue;
    };

public:
    LocalizationCheck() = default;
    ~LocalizationCheck() = default;

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

    // general check
    pnc::control::DiagnosisReturnCode TimestampCheck(const double timestamp_sec,
                                                     GeneralCheckParas* const general_check_paras_ptr);

    // specific check
    pnc::control::DiagnosisReturnCode CheckDistanceFromTrajectoryFirstPoint(
        const pnc::localization::LocalizationEstimate* const localization_ptr,
        const pnc::planning::ADCTrajectory* const trajectory_ptr);

    bool LocalizationNotJump(const ControlInputStream& control_input_stream,
                             LocalizationNotJumpParas* const localization_not_jump_paras_ptr);

    // continuous abnormal check
    bool LocalizationContinuousCheck(const std::queue<int>& localization_check_code_queue,
                                     std::vector<std::string>* const continuous_check_result_str_vector);

    // count continuous localization jump
    int LocalizationContinuousJumpCount(const std::queue<int>& localization_check_code_queue);

    bool GetCurrentState(const ControlInputStream& control_input_stream, double* const x, double* const y,
                         double* const heading, double* const speed, double* const acc,
                         double* const wheel_steer_angle_rad);

    void GetPreState(const LocalizationNotJumpParas* const localization_not_jump_paras_ptr, double* const pre_x,
                     double* const pre_y, double* const pre_heading, double* const pre_speed, double* const pre_acc,
                     double* const pre_wheel_steer_angle_rad);

    void GetPredictState(const double& pre_x, const double& pre_y, const double& pre_heading, const double& pre_speed,
                         const double& pre_acc, const double& pre_wheel_steer_angle_rad, double* const expect_x,
                         double* const expect_y, double* const expect_heading);

    void UpdatePrestate(LocalizationNotJumpParas* const localization_not_jump_paras_ptr, const double& x,
                        const double& y, const double& heading, const double& speed, const double& acc,
                        const double& wheel_steer_angle_rad);

    double LocalizationJumpAccumulatedError(const std::queue<double>& error_queue);

    void SetDebugInfo(const GlobalParas& global_param, pnc::control::DiagnosisDebugV2* const diag_debug_ptr);

    void UpdateCodeQueue(GlobalParas* const global_param);

    void CalculateLocalizationError(const double& cur_x, const double& cur_y, const double& cur_heading,
                                    const double& expect_x, const double& expect_y, const double& expect_heading,
                                    double* const station_error, double* const lateral_error,
                                    double* const heading_error, double* const distance_error);

private:
    // conf
    pnc::control::ControlConfig control_conf_;
    pnc::common::VehicleParam veh_paras_;

    // struct
    GlobalParas global_paras_;
};

}  // namespace control
}  // namespace jiduauto
