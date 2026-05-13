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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "control/src/common/dynamic_model/dynamic_model.h"
#include "control/src/common/interpolation_1d.h"
#include "control/src/controller/controller.h"
#include "control/src/math/trajectory_analyzer.h"
#include "pnc_common/src/math/filters/digital_filter.h"
#include "pnc_common/src/math/filters/mean_filter.h"
#include "pnc_common_vehicle_param.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

struct Dyn_MPC_Matrix {
    // forward
    // vehicle state matrix (discrete-time)
    Eigen::MatrixXd matrix_ad;
    // control matrix (discrete-time)
    Eigen::MatrixXd matrix_bd;

    Eigen::MatrixXd matrix_gd;

    // state
    Eigen::MatrixXd matrix_r;  // weighting
    Eigen::MatrixXd matrix_q;  // state weighting
    Eigen::MatrixXd matrix_state;
};

struct ModelStateIndex {
    static constexpr int32_t kX = 0;
    static constexpr int32_t kY = 1;
    static constexpr int32_t kTheta = 2;
    static constexpr int32_t kVx = 3;
    static constexpr int32_t kVy = 4;
    static constexpr int32_t kYawRate = 5;
    static constexpr int32_t kAcc = 6;
    static constexpr int32_t kDelta = 7;
    static constexpr int32_t kSize = 8;
};

struct ModelInputIndex {
    static constexpr int32_t kAccRate = 0;
    static constexpr int32_t kDeltaRate = 1;
    static constexpr int32_t kSize = 2;
};

// DynMpcController, using nonlinear dynamic model based mpc method to calculate
// steer angle and throttle.

class DynMpcController : public Controller {
public:
    DynMpcController();
    ~DynMpcController() override;
    bool Init(const pnc::control::ControlConfig& control_conf) override;

    bool Control(const pnc::localization::LocalizationEstimate* const localization,
                 const pnc::chassis::Chassis* const chassis, const pnc::planning::ADCTrajectory* const trajectory,
                 pnc::control::ControlCommand* const cmd) override;

    bool ControlMain(const ControlInputStream& control_input_stream, ControlCommandStream* const cmd) override;

    bool Reset() override;
    std::string Name() const override;
    void Stop() override;

private:
    bool MatrixInit();

    // Input Process
    bool LoadMPCGainScheduler();
    bool ResampleTarget();
    bool AdditionalRefStateGenerate();
    bool FinalReferenceCombine();
    bool CurrentStateGenerate();
    const double GetLateralSpeed(const pnc::localization::LocalizationEstimate& localization);

    // MPC config update
    const std::vector<DynamicModelInput> GenerateInputBound() const;
    const std::vector<DynamicModelState> GenerateStateBound() const;
    void GenerateTimeGrid(double step_gain);

    // MPC core update
    void UpdateCurrentState(const DynamicModelState& current_state);
    void UpdateStateMatrix(const LinModelMatrix& linear_dynamic_model);
    void UpdateInputMatrix(const LinModelMatrix& linear_dynamic_model);
    void UpdateLinearizationMatrix(const LinModelMatrix& linear_dynamic_model);
    const std::vector<DynamicModelState> UpdateLocalReferenceState(
        const pnc::planning::ADCTrajectory* target_trajectory_ptr, const DynamicModelState& target_state,
        const DynamicModelState& current_state);
    const std::vector<DynamicModelState> UpdateLocalReferenceState(const DynamicModelState& target_state,
                                                                   const DynamicModelState& current_state);

    // Output process
    void OutputCommandProcess(const DynamicModelState& current_mpc_state);
    bool SummaryDebugInfo(const DynamicModelState& current_state, const DynamicModelInput& current_input,
                          const std::vector<DynamicModelState>& reference_states,
                          const std::vector<DynamicModelState>& pred_states,
                          const std::vector<DynamicModelInput>& cmd_seqs, const std::vector<double>& time_grid,
                          const int32_t cost_time, pnc::control::ControlDebug* const control_debug) const;

    bool InitLogFile();
    void LogDebugFile(const ControlInputStream& control_input_stream,
                      const pnc::control::DynamicMpcDebug& control_debug);

    // Convert
    Eigen::MatrixXd StateToMatrix(const DynamicModelState& mpc_state);
    Eigen::MatrixXd InputToMatrix(const DynamicModelInput& mpc_input);
    DynamicModelState MatrixToState(const Eigen::MatrixXd& state_matrix);
    DynamicModelInput MatrixToInput(const Eigen::MatrixXd& input_matrix);

    bool Solve();

    bool controller_initialized_{false};
    bool use_debug_csv_file_{false};    // only for debug log
    FILE* dyn_mpc_log_file_{nullptr};   // only for debug log
    std::size_t log_index_{0};          // only for debug log
    bool is_current_auto_mode_{false};  // only for debug log

    // status
    const pnc::localization::LocalizationEstimate* localization_msg_ptr_{nullptr};
    const pnc::chassis::Chassis* chassis_msg_ptr_{nullptr};
    const pnc::planning::ADCTrajectory* trajectory_msg_ptr_{nullptr};
    pnc::control::ControlCommand* command_msg_ptr_{nullptr};

    // conf
    pnc::control::DynMPCControllerConf dyn_mpc_conf_;
    Dyn_MPC_Matrix dyn_osqp_model_;
    pnc::common::VehicleParam vehicle_param_;

    // mpc_use_calibration_tab flag
    std::unique_ptr<Interpolation1D> acc_control_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> steer_control_interpolation_ = std::make_unique<Interpolation1D>();

    std::unique_ptr<Interpolation1D> x_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> y_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> theta_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> vx_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> vy_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> yaw_rate_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> acc_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> delta_interpolation_ = std::make_unique<Interpolation1D>();

    TrajectoryAnalyzer analyzer_;

    Eigen::MatrixXd control_cmd_matrix_;
    Eigen::MatrixXd control_state_matrix_;
    std::vector<Eigen::MatrixXd> reference_local_matrixs_;
    std::vector<double> time_grid_;

    double control_sample_time_ = 0.02;
    double ts_ = 0.05;
    int32_t pred_horizon_ = 20;
    bool is_current_backward_ = false;
    double pre_steer_angle_ = 0.0;
    double pre_acc_ = 0.0;
    double pre_delta_steer_angle_ = 0.0;
    double pre_delta_acc_ = 0.0;
    double last_measured_acc_ = 0.0;
    double last_measured_delta_ = 0.0;
};

}  // namespace control
}  // namespace jiduauto
