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

#include <memory>
#include <string>
#include <vector>

#include "control/src/common/interpolation_1d.h"
#include "control/src/common/kinematic_model/kinematic_model.h"
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

struct Kin_MPC_Matrix {
    Eigen::MatrixXd matrix_ad;
    Eigen::MatrixXd matrix_bd;
    Eigen::MatrixXd matrix_gd;

    Eigen::MatrixXd matrix_r;  // control weighting
    Eigen::MatrixXd matrix_q;  // state weighting
    Eigen::MatrixXd matrix_state;
};

struct KinModelStateIndex {
    static constexpr int32_t kX = 0;
    static constexpr int32_t kY = 1;
    static constexpr int32_t kTheta = 2;
    static constexpr int32_t kSpeed = 3;
    static constexpr int32_t kDelta = 4;
    static constexpr int32_t kSize = 5;
};

struct KinModelInputIndex {
    static constexpr int32_t kAcc = 0;
    static constexpr int32_t kDeltaRate = 1;
    static constexpr int32_t kSize = 2;
};

// KinMpcController, using kinematic model based mpc method to calculate
// steer angle and throttle.

class KinMpcController : public Controller {
public:
    KinMpcController();
    ~KinMpcController() override;
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
    pnc::control::ControlModeInfo::ControlScenarioMode RecogniteTrajectoryMode(
        const pnc::planning::ADCTrajectory& target_glb_trajectory);

    // MPC config update
    void InputBoundGenerate();
    void StateBoundGenerate();
    void TimeGridGenerate(double step_gain);

    // MPC core update
    void UpdateCurrentState(const KinematicModelState& current_state);
    void UpdateStateMatrix(const KinLinModelMatrix& linear_model);
    void UpdateInputMatrix(const KinLinModelMatrix& linear_model);
    void UpdateLinearizationMatrix(const KinLinModelMatrix& linear_model);
    void UpdateCostWeightsBySpeed(double current_speed);
    void UpdateCostWeightsByMode(const pnc::control::ControlModeInfo::ControlScenarioMode& mode);
    const std::vector<KinematicModelState> UpdateLocalReferenceState(
        const pnc::planning::ADCTrajectory* target_trajectory_ptr, const KinematicModelState& current_state);

    // Output process
    void OutputCommandProcess(const KinematicModelState& current_mpc_state);
    bool SummaryDebugInfo(const KinematicModelState& current_state,
                          const std::vector<KinematicModelState>& reference_states,
                          const std::vector<KinematicModelState>& pred_states,
                          const std::vector<KinematicModelInput>& cmd_seqs, const std::vector<double>& time_grid,
                          const int32_t to_update_cur_state_ms, const int32_t to_matrix_build_ms,
                          const int32_t to_solve_end_ms, const int32_t to_output_ms, const int32_t cost_time,
                          const pnc::control::ControlModeInfo::ControlScenarioMode& mode,
                          pnc::control::ControlDebug* const control_debug) const;

    // Convert
    Eigen::MatrixXd StateToMatrix(const KinematicModelState& mpc_state);
    Eigen::MatrixXd InputToMatrix(const KinematicModelInput& mpc_input);
    KinematicModelState MatrixToState(const Eigen::MatrixXd& state_matrix);
    KinematicModelInput MatrixToInput(const Eigen::MatrixXd& input_matrix);

    // Mode
    bool PauseLogic(double speed);

    bool Solve();

    bool InitLogFile();
    void LogDebugFile(const ControlInputStream& control_input_stream,
                      const pnc::control::KinematicMpcDebug& control_debug);

    bool controller_initialized_{false};
    bool use_debug_csv_file_{false};    // only for debug log
    FILE* kin_mpc_log_file_{nullptr};   // only for debug log
    bool is_current_auto_mode_{false};  // only for debug log

    // status
    const pnc::localization::LocalizationEstimate* localization_msg_ptr_{nullptr};
    const pnc::chassis::Chassis* chassis_msg_ptr_{nullptr};
    const pnc::planning::ADCTrajectory* trajectory_msg_ptr_{nullptr};
    pnc::control::ControlCommand* command_msg_ptr_{nullptr};

    // conf
    pnc::control::KinMPCControllerConf kin_mpc_conf_;
    Kin_MPC_Matrix kin_osqp_model_;
    pnc::common::VehicleParam vehicle_param_;

    // mpc_use_calibration_tab flag
    std::unique_ptr<Interpolation1D> acc_control_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> steer_gain_control_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> x_state_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> y_state_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> theta_state_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> speed_state_interpolation_ = std::make_unique<Interpolation1D>();
    std::unique_ptr<Interpolation1D> steer_state_interpolation_ = std::make_unique<Interpolation1D>();

    TrajectoryAnalyzer analyzer_;

    Eigen::MatrixXd control_cmd_matrix_;
    Eigen::MatrixXd control_state_matrix_;
    std::vector<Eigen::MatrixXd> reference_local_matrixs_;
    std::vector<double> time_grid_;

    double cur_solution_time_ = 0.0;
    double last_steer_rate_cmd_ = 0.0;
    double last_steer_cmd_ = 0.0;
    double last_acc_cmd_ = 0.0;

    double ts_ = 0.05;
    double control_period_s_ = 0.02;
    int32_t pred_horizon_ = 20;
    double standstill_speed_ = 0.2;
    bool is_current_backward_ = false;
    bool enable_localization_speed_ = false;
    bool enable_compensatory_acceleration_ = false;

    pnc::control::ControlModeInfo::ControlScenarioMode kin_control_mode_;
};

}  // namespace control
}  // namespace jiduauto
