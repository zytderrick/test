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

#include "control/src/math/mpc_solver_qpoase.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

struct MPCModel {
    int32_t state_num{0};
    int32_t input_num{0};
    int32_t output_num{0};
    int32_t horizon_control{0};
    int32_t horizon_prediction{0};
    double ts{0.0};
    // matrix
    Eigen::MatrixXd matrix_a;
    Eigen::MatrixXd matrix_b;
    Eigen::MatrixXd matrix_c;
    Eigen::MatrixXd matrix_d;
    Eigen::MatrixXd matrix_q;
    Eigen::MatrixXd matrix_qp;
    Eigen::MatrixXd matrix_r1;
    Eigen::MatrixXd matrix_r2;
    Eigen::MatrixXd u_lower_boundary;
    Eigen::MatrixXd u_upper_boundary;
    Eigen::MatrixXd du_lower_boundary;
    Eigen::MatrixXd du_upper_boundary;
    Eigen::MatrixXd y_lower_boundary;
    Eigen::MatrixXd y_upper_boundary;
    Eigen::MatrixXd y_lower_terminal_boundary;
    Eigen::MatrixXd y_upper_terminal_boundary;
    Eigen::MatrixXd state_init;
    Eigen::MatrixXd previous_input;
    std::vector<Eigen::MatrixXd> reference;

    MPCModel(const int32_t state_num_t, const int32_t input_num_t, const int32_t output_num_t,
             const int32_t horizon_control_t, const int32_t horizon_prediction_t, const double ts_t)
        : state_num(state_num_t),
          input_num(input_num_t),
          output_num(output_num_t),
          horizon_control(horizon_control_t),
          horizon_prediction(horizon_prediction_t),
          ts(ts_t) {}
};

class MPCKernel {
public:
    MPCKernel();
    ~MPCKernel() = default;

    /**
     * @brief: convert to quadratic programming problem and solve it
     *
     *  min_x:            J(x) = 0.5 * x^T * H * x  + x^T * G
     *  with respect to:  A * x  = b (equality constraint)
     *                    C * x >= d (inequality constraint)
     **/
    bool Solve(const MPCModel& mpc_model, std::vector<Eigen::MatrixXd>* const output);

    void set_mpc_param(const MPCParam& mpc_param);

private:
    /**
     * @brief sanity check of matrix.
     */
    bool MatrixCheck(const MPCModel& mpc_model);

    /**
     * @brief init matrix size.
     */
    void Init(const MPCModel& mpc_model);

    /**
     * @brief continous to discrete.
     * Ad = (I-0.5*ts*A)^-1 * (I+0.5*ts*A).
     * Bd = (I-0.5*ts*A)^-1 * B * ts.
     * Cd = (I-0.5*ts*A)^-1 * C * ts.
     */
    void MatrixC2D(const MPCModel& model, Eigen::MatrixXd* const matrix_a_out, Eigen::MatrixXd* const matrix_b_out,
                   Eigen::MatrixXd* const matrix_c_out);

    /**
     * @brief update LA, LB, LC
     */
    void PrepareMatrixABC(const MPCModel& model, Eigen::MatrixXd* const matrix_la, Eigen::MatrixXd* const matrix_lb,
                          Eigen::MatrixXd* const matrix_lc);

    /**
     * @brief update Q, R
     */
    void PrepareMatrixQR(const MPCModel& model, Eigen::MatrixXd* const matrix_q, Eigen::MatrixXd* const matrix_r1,
                         Eigen::MatrixXd* const matrix_r2);

    /**
     * @brief update Yita.
     */
    void PrepareMatrixYita(const MPCModel& model, Eigen::MatrixXd* const matrix_yita,
                           Eigen::MatrixXd* const matrix_yita_c);

    void PrepareMatrixConstrain(const MPCModel& model, Eigen::MatrixXd* const matrix_u_lb,
                                Eigen::MatrixXd* const matrix_u_ub, Eigen::MatrixXd* const matrix_du_lb,
                                Eigen::MatrixXd* const matrix_du_ub, Eigen::MatrixXd* const matrix_y_lb,
                                Eigen::MatrixXd* const matrix_y_ub);

    /**
     * @brief Debug log matrix.
     */
    void LogKernelMatrix(const Eigen::MatrixXd& matrix_h, const Eigen::MatrixXd& matrix_g,
                         const Eigen::MatrixXd& matrix_constrain_a, const Eigen::MatrixXd& matrix_constrain_b,
                         const Eigen::MatrixXd& matrix_constrain_c, const Eigen::MatrixXd& matrix_constrain_d);

    /**
     * @brief Debug log matrix.
     */
    void LogStateMatrix(const Eigen::MatrixXd& matrix_ad, const Eigen::MatrixXd& matrix_bd,
                        const Eigen::MatrixXd& matrix_cd, const Eigen::MatrixXd& matrix_dd,
                        const Eigen::MatrixXd& matrix_sa, const Eigen::MatrixXd& matrix_sb,
                        const Eigen::MatrixXd& matrix_sc, const Eigen::MatrixXd& matrix_sd);

    /**
     * @brief Debug log matrix.
     */
    void LogQRMatrix(const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r1,
                     const Eigen::MatrixXd& matrix_r2);

    /**
     * @brief Debug log matrix.
     */
    void LogMatrix(const Eigen::MatrixXd& matrix_x, std::string name);

private:
    MPCParam mpc_param_;

    uint32_t sn_{0};     // state number
    uint32_t in_{0};     // input number
    uint32_t on_{0};     // output number
    uint32_t hc_{0};     // control horizon
    uint32_t hp_{0};     // prediction horizon
    uint32_t hp_sn_{0};  // prediction horizon * state number
    uint32_t hp_on_{0};  // prediction horizon * output number
    uint32_t hc_in_{0};  // control horizon * input number

    std::unique_ptr<MPCSolverQpoase> mpc_solver_ptr_{nullptr};
};

}  // namespace control
}  // namespace jiduauto
