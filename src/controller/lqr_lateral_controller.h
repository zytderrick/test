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
 * @brief Defines the LqrLateralController class.
 */

#pragma once

#include <Eigen/Core>
#include <string>

#include "control/src/controller/controller.h"
#include "pnc_common/src/math/filters/digital_filter.h"
#include "pnc_common/src/math/filters/mean_filter.h"
#include "pnc_common_vehicle_param.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

struct LQR_Matrix {
    /*forward*/
    // vehicle state matrix
    Eigen::MatrixXd matrix_a;
    // vehicle state matrix (discrete-time)
    Eigen::MatrixXd matrix_ad;
    // vehicle state matrix compound; related to preview
    Eigen::MatrixXd matrix_adc;
    // control matrix
    Eigen::MatrixXd matrix_b;
    // control matrix (discrete-time)
    Eigen::MatrixXd matrix_bd;
    // control matrix compound
    Eigen::MatrixXd matrix_bdc;
    // vehicle state matrix coefficients
    Eigen::MatrixXd matrix_a_coeff;
    // gain
    Eigen::MatrixXd matrix_k;
    // p
    Eigen::MatrixXd matrix_p;

    /* backward */
    Eigen::MatrixXd backward_matrix_a;
    Eigen::MatrixXd backward_matrix_ad;
    Eigen::MatrixXd backward_matrix_adc;
    Eigen::MatrixXd backward_matrix_b;
    Eigen::MatrixXd backward_matrix_bd;
    Eigen::MatrixXd backward_matrix_bdc;
    Eigen::MatrixXd backward_matrix_a_coeff;

    /* state */
    Eigen::MatrixXd matrix_r;  // weighting
    Eigen::MatrixXd matrix_q;  // state weighting
    Eigen::MatrixXd matrix_state;
};

/**
 * @brief LqrLateralController, using lqr method to calculate steer angle.
 */
class LqrLateralController : public Controller {
public:
    LqrLateralController();
    ~LqrLateralController() override;

    /**
     * @brief Init lqr_controller paras.
     * @return true if succeeded; otherwise false;
     */
    bool Init(const pnc::control::ControlConfig& control_conf) override;

    /**
     * @brief Main function, compute cmd.
     * @param localization msg
     * @param chassis msg
     * @param trajectory msg
     * @param cmd set lateral_cmd to cmd
     * @return true if succeeded; otherwise false;
     */
    bool Control(const pnc::localization::LocalizationEstimate* const localization,
                 const pnc::chassis::Chassis* const chassis, const pnc::planning::ADCTrajectory* const trajectory,
                 pnc::control::ControlCommand* const cmd) override {
        return true;
    }

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
     * @brief Init LQR_Matrix.
     */
    bool MatrixInit();

    /**
     * @brief calculate state [lateral error, rate; heading error, rate].
     */
    bool StateUpdate();

    /**
     * @brief Update A based on v.
     */
    void MatrixUpdate();

    /**
     * @brief used for preview matrix.
     * Reserved.
     */
    void MatrixCompound();

    /**
     * @brief calculate steer_angle[lat_compo + dlat_compo+head_compo +
     * dhead_compo].
     */
    void SteerComponentCalc();

    /**
     * @brief check whether is the end of planning.
     */
    bool CheckOverStopPoint();

    /**
     * @brief sanitycheck.
     * @return true if passed; otherwise false;
     */
    bool SanityCheck();

    /**
     * @brief transform localization from mass_center to rear_axis center.
     * @return true if succeeded; otherwise false;
     */
    bool GetRearPos(double* const x, double* const y);

    /**
     * @brief Given speed, calculate Q and R.
     * @param this->current_paras_
     * @param this->lqr_matrix_.matrix_q_
     * @param this->lqr_matrix_.matrix_r_
     */
    void GetParasFromScheduler(const double current_speed);

    /**
     * @brief Given speed, and pnc::control::LqrTable, calculate current_paras_.
     * Using linear interpolation.
     * @param this->current_paras_
     */
    void GainScheduler(const double current_speed,
                       const google::protobuf::RepeatedPtrField<pnc::control::LqrTable>& paras);

    /**
     * @brief log debug info.
     */
    void LogDebugInfo();

    /**
     * @brief Init log_file_ handle.
     */
    bool InitLogFile();

    bool SummaryDebugInfo(pnc::control::ControlDebug* const control_debug) const;

private:
    const int32_t STATE_SIZE{4};
    // debug logger
    bool controller_initialized_{false};
    bool use_debug_csv_file_{false};    // only for debug log
    FILE* steer_log_file_{nullptr};     // only for debug log
    std::size_t log_index_{0};          // only for debug log
    bool is_current_auto_mode_{false};  // only for debug log

    // status
    const pnc::localization::LocalizationEstimate* localization_msg_{nullptr};
    const pnc::chassis::Chassis* chassis_msg_{nullptr};
    const pnc::planning::ADCTrajectory* trajectory_msg_{nullptr};

    // conf
    pnc::control::LqrControllerConf lqr_conf_;
    pnc::common::VehicleParam lat_paras_;
    LQR_Matrix lqr_matrix_;
    pnc::control::LqrTable current_paras_;
    // conf paras
    double ts_{0.02};

    // digital filter
    pnc::filter::DigitalFilter steer_angle_filter_;
    pnc::filter::MeanFilter lateral_error_filter_;
    pnc::filter::MeanFilter lateral_rate_filter_;
    pnc::filter::MeanFilter heading_error_filter_;
    pnc::filter::MeanFilter heading_rate_filter_;
    pnc::filter::MeanFilter curvature_filter_;

    // vehicle state paras
    double current_speed_{0.0};
    bool is_current_backward_{false};

    // lateral error and heading error
    int32_t current_index_{0};
    int32_t target_index_{0};
    double current_lateral_{0.0};
    double current_lateral_error_{0.0};
    double current_ref_heading_{0.0};
    double current_heading_{0.0};
    double current_heading_error_{0.0};
    double current_curvature_{0.0};
    double previous_heading_error_{0.0};
    double previous_lateral_error_{0.0};
    double heading_error_rate_{0.0};
    double lateral_error_rate_{0.0};
    // inner* is error to nearest point
    double inner_lateral_error_{0.0};
    double inner_heading_error_{0.0};
    // record* is target point's info
    double record_x_{0.0};
    double record_y_{0.0};
    double record_head_{0.0};

    // lqr output
    double steer_angle_feedbackterm_{0.0};
    double steer_angle_lateral_contribution_{0.0};
    double steer_angle_lateral_rate_contribution_{0.0};
    double steer_angle_heading_contribution_{0.0};
    double steer_angle_heading_rate_contribution_{0.0};
    double steer_angle_feedforwardterm_{0.0};
    double steer_angle_{0.0};

    double pre_steer_angle_{0.0};
    int32_t num_iterations_{0};
};

}  // namespace control
}  // namespace jiduauto
