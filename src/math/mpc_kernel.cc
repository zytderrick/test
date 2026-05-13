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

#include "control/src/math/mpc_kernel.h"

#include <Eigen/LU>

#include "pnc_common/src/common/pnc_logger.h"

namespace jiduauto {
namespace control {

MPCKernel::MPCKernel() { mpc_solver_ptr_ = std::make_unique<MPCSolverQpoase>(); }

bool MPCKernel::Solve(const MPCModel& model, std::vector<Eigen::MatrixXd>* const output_ptr) {
    if (output_ptr == nullptr) {
        LOG_ERROR("input nullptr");
        return false;
    }
    output_ptr->clear();
    for (int32_t i = 0; i < model.horizon_control; ++i) {
        output_ptr->push_back(Eigen::MatrixXd::Zero(in_, 1));
    }
    if (!MatrixCheck(model)) {
        LOG_ERROR("MatrixCheck failed");
        return false;
    }

    Init(model);

    // update core matrix a, b, c
    Eigen::MatrixXd matrix_la;
    Eigen::MatrixXd matrix_lb;
    Eigen::MatrixXd matrix_lc;
    PrepareMatrixABC(model, &matrix_la, &matrix_lb, &matrix_lc);

    // update weighting matrix Q for output, R1 for input, R2 for delta input
    Eigen::MatrixXd matrix_q = Eigen::MatrixXd::Zero(hp_on_, hp_on_);
    Eigen::MatrixXd matrix_r1 = Eigen::MatrixXd::Zero(hc_in_, hc_in_);
    Eigen::MatrixXd matrix_r2 = Eigen::MatrixXd::Zero(hc_in_, hc_in_);
    PrepareMatrixQR(model, &matrix_q, &matrix_r1, &matrix_r2);

    // update matrix yita, yita_c for delta output
    Eigen::MatrixXd matrix_yita;
    Eigen::MatrixXd matrix_yita_c;
    PrepareMatrixYita(model, &matrix_yita, &matrix_yita_c);

    // update reference output matrix
    Eigen::MatrixXd matrix_ref = Eigen::MatrixXd::Zero(hp_on_, 1);
    for (int32_t i = 0; i < hp_; ++i) {
        matrix_ref.block(i * on_, 0, on_, 1) = model.reference[i];
    }

    // update constrains
    // update constrain matrix Ca, Cb
    Eigen::MatrixXd matrix_u_lb = Eigen::MatrixXd::Zero(hc_in_, 1);
    Eigen::MatrixXd matrix_u_ub = Eigen::MatrixXd::Zero(hc_in_, 1);
    Eigen::MatrixXd matrix_du_lb = Eigen::MatrixXd::Zero(hc_in_, 1);
    Eigen::MatrixXd matrix_du_ub = Eigen::MatrixXd::Zero(hc_in_, 1);
    Eigen::MatrixXd matrix_y_lb = Eigen::MatrixXd::Zero(hp_on_, 1);
    Eigen::MatrixXd matrix_y_ub = Eigen::MatrixXd::Zero(hp_on_, 1);

    PrepareMatrixConstrain(model, &matrix_u_lb, &matrix_u_ub, &matrix_du_lb, &matrix_du_ub, &matrix_y_lb, &matrix_y_ub);

    Eigen::MatrixXd matrix_constrain_a = Eigen::MatrixXd::Zero((2 * hc_in_ + 2 * hc_in_ + 2 * hp_on_), hc_in_);
    Eigen::MatrixXd matrix_constrain_b = Eigen::MatrixXd::Zero((2 * hc_in_ + 2 * hc_in_ + 2 * hp_on_), 1);
    Eigen::MatrixXd matrix_lp = matrix_la * model.state_init + matrix_lc;

    matrix_constrain_a << Eigen::MatrixXd::Identity(hc_in_, hc_in_), -Eigen::MatrixXd::Identity(hc_in_, hc_in_),
        matrix_yita, -matrix_yita, matrix_lb, -matrix_lb;
    matrix_constrain_b << matrix_u_lb, -matrix_u_ub, matrix_du_lb - matrix_yita_c, -matrix_du_ub + matrix_yita_c,
        matrix_y_lb - matrix_lp, -matrix_y_ub + matrix_lp;

    Eigen::MatrixXd matrix_h =
        matrix_lb.transpose() * matrix_q * matrix_lb + matrix_r1 + matrix_yita.transpose() * matrix_r2 * matrix_yita;
    Eigen::MatrixXd matrix_g =
        matrix_lb.transpose() * matrix_q * (matrix_la * model.state_init + matrix_lc - matrix_ref) +
        matrix_yita.transpose() * matrix_r2 * matrix_yita_c;

    // update constrain matrix Cc, Cd
    Eigen::MatrixXd matrix_constrain_c = Eigen::MatrixXd::Zero((2 * hc_in_ + 2 * hc_in_ + 2 * hp_on_), hc_in_);
    Eigen::MatrixXd matrix_constrain_d = Eigen::MatrixXd::Zero((2 * hc_in_ + 2 * hc_in_ + 2 * hp_on_), 1);

    // print matrix
    if (mpc_param_.enable_debug) {
        LogKernelMatrix(matrix_h, matrix_g, matrix_constrain_a, matrix_constrain_b, matrix_constrain_c,
                        matrix_constrain_d);
    }

    // solve qp problem
    Eigen::MatrixXd solution = Eigen::MatrixXd::Zero(hc_in_, 1);
    if (!mpc_solver_ptr_->Solve(matrix_h, matrix_g, matrix_constrain_a, matrix_constrain_b, matrix_constrain_c,
                                matrix_constrain_d, &solution)) {
        LOG_ERROR("Solve failed");
        return false;
    }

    for (int32_t i = 0; i < hc_; ++i) {
        (*output_ptr)[i] = solution.block(i * in_, 0, in_, 1);
    }

    return true;
}

void MPCKernel::set_mpc_param(const MPCParam& mpc_param) {
    mpc_param_ = mpc_param;
    mpc_solver_ptr_->set_mpc_param(mpc_param);
}

// bilinear transfer.
void MPCKernel::MatrixC2D(const MPCModel& model, Eigen::MatrixXd* const matrix_ad, Eigen::MatrixXd* const matrix_bd,
                          Eigen::MatrixXd* const matrix_cd) {
    Eigen::MatrixXd matrix_i = Eigen::MatrixXd::Identity(model.matrix_a.rows(), model.matrix_a.cols());
    Eigen::MatrixXd matrix_tmp = (matrix_i - model.ts * 0.5 * model.matrix_a).inverse();
    *matrix_ad = matrix_tmp * (matrix_i + model.ts * 0.5 * model.matrix_a);
    *matrix_bd = matrix_tmp * model.matrix_b * model.ts;
    // TODO(chi.zhang): *matrix_cd = model.matrix_c * matrix_tmp;
    *matrix_cd = matrix_tmp * model.matrix_c;  /// wrong equations
}

void MPCKernel::PrepareMatrixABC(const MPCModel& model, Eigen::MatrixXd* const matrix_la,
                                 Eigen::MatrixXd* const matrix_lb, Eigen::MatrixXd* const matrix_lc) {
    Eigen::MatrixXd matrix_ad;
    Eigen::MatrixXd matrix_bd;
    Eigen::MatrixXd matrix_cd;
    Eigen::MatrixXd matrix_dd = model.matrix_d;
    MatrixC2D(model, &matrix_ad, &matrix_bd, &matrix_cd);

    // update matrix a power
    std::vector<Eigen::MatrixXd> matrix_a_power(hp_ + 1);
    matrix_a_power[0] = Eigen::MatrixXd::Identity(sn_, sn_);
    for (int32_t i = 1; i <= hp_; ++i) {
        matrix_a_power[i] = matrix_ad * matrix_a_power[i - 1];
    }

    // update matrix Sa
    Eigen::MatrixXd matrix_sa = Eigen::MatrixXd::Zero(hp_sn_, sn_);
    for (int32_t i = 0; i < hp_; ++i) {
        matrix_sa.block(i * sn_, 0, sn_, sn_) = matrix_a_power[i + 1];
    }

    // update matrix Sb
    std::vector<Eigen::MatrixXd> matrix_sb_sum(hp_ - hc_ + 1);
    matrix_sb_sum[0] = matrix_bd;
    for (int32_t i = 1; i < hp_ - hc_ + 1; ++i) {
        matrix_sb_sum[i] = matrix_a_power[i] * matrix_bd + matrix_sb_sum[i - 1];
    }
    Eigen::MatrixXd matrix_sb = Eigen::MatrixXd::Zero(hp_sn_, hc_in_);
    for (int32_t c = 0; c < hc_; ++c) {
        for (int32_t r = 0; r < hp_; ++r) {
            if (r > c) {
                if (c == hc_ - 1) {
                    matrix_sb.block(r * sn_, c * in_, sn_, in_) = matrix_sb_sum[r - c];
                } else {
                    matrix_sb.block(r * sn_, c * in_, sn_, in_) = matrix_a_power[r - c] * matrix_bd;
                }
            } else if (c == r) {
                matrix_sb.block(r * sn_, c * in_, sn_, in_) = matrix_bd;
            }
        }
    }
    // update matrix Sc
    Eigen::MatrixXd matrix_sc = Eigen::MatrixXd::Zero(hp_sn_, 1);
    matrix_sc.block(0, 0, sn_, 1) = matrix_cd;
    for (int32_t i = 1; i < hp_; ++i) {
        matrix_sc.block(i * sn_, 0, sn_, 1) = matrix_sc.block((i - 1) * sn_, 0, sn_, 1) + matrix_a_power[i] * matrix_cd;
    }

    // update matrix Sd
    Eigen::MatrixXd matrix_sd = Eigen::MatrixXd::Zero(hp_on_, hp_sn_);
    for (int32_t i = 0; i < hp_; ++i) {
        matrix_sd.block(i * on_, i * sn_, on_, sn_) = matrix_dd;
    }

    // update matrix La, Lb, Lc for output
    *matrix_la = matrix_sd * matrix_sa;
    *matrix_lb = matrix_sd * matrix_sb;
    *matrix_lc = matrix_sd * matrix_sc;

    // print state matrix
    if (mpc_param_.enable_debug) {
        LogStateMatrix(matrix_ad, matrix_bd, matrix_cd, matrix_dd, matrix_sa, matrix_sb, matrix_sc, matrix_sd);
    }
}

void MPCKernel::PrepareMatrixQR(const MPCModel& model, Eigen::MatrixXd* const matrix_q,
                                Eigen::MatrixXd* const matrix_r1, Eigen::MatrixXd* const matrix_r2) {
    for (int32_t i = 0; i < hp_ - 1; ++i) {
        (*matrix_q).block(i * on_, i * on_, on_, on_) = model.matrix_q;
    }

    (*matrix_q).block((hp_ - 1) * on_, (hp_ - 1) * on_, on_, on_) = model.matrix_qp;

    for (int32_t i = 0; i < hc_; ++i) {
        (*matrix_r1).block(i * in_, i * in_, in_, in_) = model.matrix_r1;
        (*matrix_r2).block(i * in_, i * in_, in_, in_) = model.matrix_r2;
    }

    if (mpc_param_.enable_debug) {
        LogQRMatrix(*matrix_q, *matrix_r1, *matrix_r2);
    }
}

void MPCKernel::PrepareMatrixYita(const MPCModel& model, Eigen::MatrixXd* const matrix_yita,
                                  Eigen::MatrixXd* const matrix_yita_c) {
    *matrix_yita = Eigen::MatrixXd::Identity(hc_in_, hc_in_);
    for (int32_t i = 1; i < hc_; ++i) {
        (*matrix_yita).block(i * in_, (i - 1) * in_, in_, in_) = -Eigen::MatrixXd::Identity(in_, in_);
    }
    *matrix_yita_c = Eigen::MatrixXd::Zero(hc_in_, 1);
    (*matrix_yita_c).block(0, 0, in_, 1) = -model.previous_input;

    if (mpc_param_.enable_debug) {
        LogMatrix(*matrix_yita, "matrix_yita");
        LogMatrix(*matrix_yita_c, "matrix_yita_c");
    }
}

void MPCKernel::PrepareMatrixConstrain(const MPCModel& model, Eigen::MatrixXd* const matrix_u_lb,
                                       Eigen::MatrixXd* const matrix_u_ub, Eigen::MatrixXd* const matrix_du_lb,
                                       Eigen::MatrixXd* const matrix_du_ub, Eigen::MatrixXd* const matrix_y_lb,
                                       Eigen::MatrixXd* const matrix_y_ub) {
    for (int32_t i = 0; i < hc_; ++i) {
        (*matrix_u_lb).block(i * in_, 0, in_, 1) = model.u_lower_boundary;
        (*matrix_u_ub).block(i * in_, 0, in_, 1) = model.u_upper_boundary;
        (*matrix_du_lb).block(i * in_, 0, in_, 1) = model.du_lower_boundary;
        (*matrix_du_ub).block(i * in_, 0, in_, 1) = model.du_upper_boundary;
    }
    for (int32_t i = 0; i < hp_ - 1; ++i) {
        (*matrix_y_lb).block(i * on_, 0, on_, 1) = model.y_lower_boundary;
        (*matrix_y_ub).block(i * on_, 0, on_, 1) = model.y_upper_boundary;
    }
    (*matrix_y_lb).block((hp_ - 1) * on_, 0, on_, 1) = model.y_lower_terminal_boundary;
    (*matrix_y_ub).block((hp_ - 1) * on_, 0, on_, 1) = model.y_upper_terminal_boundary;
}

bool MPCKernel::MatrixCheck(const MPCModel& model) {
    if (model.ts < 0.0001) {
        LOG_ERROR("ts error");
        return false;
    }
    if (model.horizon_control < 1 || model.horizon_control > model.horizon_prediction) {
        LOG_ERROR("horizon_control error");
        return false;
    }

    if (model.matrix_a.rows() != model.state_num || model.matrix_a.cols() != model.state_num) {
        LOG_ERROR("matrix_a error");
        return false;
    }

    if (model.matrix_b.rows() != model.state_num || model.matrix_b.cols() != model.input_num) {
        LOG_ERROR("matrix_b error");
        return false;
    }

    if (model.matrix_c.rows() != model.state_num || model.matrix_c.cols() != 1) {
        LOG_ERROR("matrix_c error");
        return false;
    }

    if (model.matrix_d.rows() != model.output_num || model.matrix_d.cols() != model.state_num) {
        LOG_ERROR("matrix_d error");
        return false;
    }

    if (model.matrix_q.rows() != model.output_num || model.matrix_q.cols() != model.output_num) {
        LOG_ERROR("matrix_q error");
        return false;
    }

    if (model.matrix_r1.rows() != model.input_num || model.matrix_r1.cols() != model.input_num) {
        LOG_ERROR("matrix_r1 error");
        return false;
    }

    if (model.matrix_r2.rows() != model.input_num || model.matrix_r2.cols() != model.input_num) {
        LOG_ERROR("matrix_r2 error");
        return false;
    }

    if (model.u_upper_boundary.rows() != model.input_num || model.u_upper_boundary.cols() != 1) {
        LOG_ERROR("u_upper_boundary error");
        return false;
    }

    if (model.u_lower_boundary.rows() != model.input_num || model.u_lower_boundary.cols() != 1) {
        LOG_ERROR("u_lower_boundary error");
        return false;
    }

    if (model.du_upper_boundary.rows() != model.input_num || model.du_upper_boundary.cols() != 1) {
        LOG_ERROR("du_upper_boundary error");
        return false;
    }

    if (model.du_lower_boundary.rows() != model.input_num || model.du_lower_boundary.cols() != 1) {
        LOG_ERROR("du_lower_boundary error");
        return false;
    }

    if (model.y_upper_boundary.rows() != model.output_num || model.y_upper_boundary.cols() != 1) {
        LOG_ERROR("y_upper_boundary error");
        return false;
    }

    if (model.y_lower_boundary.rows() != model.output_num || model.y_lower_boundary.cols() != 1) {
        LOG_ERROR("y_lower_boundary error");
        return false;
    }

    if (model.state_init.rows() != model.state_num || model.state_init.cols() != 1) {
        LOG_ERROR("state_init error");
        return false;
    }

    if (model.previous_input.rows() != model.input_num || model.previous_input.cols() != 1) {
        LOG_ERROR("previous_input error");
        return false;
    }

    if (model.reference.size() != model.horizon_prediction) {
        LOG_ERROR("reference_.size() error");
        return false;
    }

    for (const auto& matrix_ref : model.reference) {
        if (matrix_ref.rows() != model.output_num || matrix_ref.cols() != 1) {
            LOG_ERROR("reference error");
            return false;
        }
    }
    return true;
}

void MPCKernel::Init(const MPCModel& model) {
    sn_ = model.state_num;
    in_ = model.input_num;
    on_ = model.output_num;
    hc_ = model.horizon_control;
    hp_ = model.horizon_prediction;
    hp_sn_ = model.state_num * model.horizon_prediction;
    hp_on_ = model.output_num * model.horizon_prediction;
    hc_in_ = model.input_num * model.horizon_control;
}

void MPCKernel::LogKernelMatrix(const Eigen::MatrixXd& matrix_h, const Eigen::MatrixXd& matrix_g,
                                const Eigen::MatrixXd& matrix_constrain_a, const Eigen::MatrixXd& matrix_constrain_b,
                                const Eigen::MatrixXd& matrix_constrain_c, const Eigen::MatrixXd& matrix_constrain_d) {
    LogMatrix(matrix_h, "matrix_h");
    LogMatrix(matrix_g, "matrix_g");
    LogMatrix(matrix_constrain_a, "matrix_constrain_a");
    LogMatrix(matrix_constrain_b, "matrix_constrain_b");
    LogMatrix(matrix_constrain_c, "matrix_constrain_c");
    LogMatrix(matrix_constrain_d, "matrix_constrain_d");
}

void MPCKernel::LogStateMatrix(const Eigen::MatrixXd& matrix_ad, const Eigen::MatrixXd& matrix_bd,
                               const Eigen::MatrixXd& matrix_cd, const Eigen::MatrixXd& matrix_dd,
                               const Eigen::MatrixXd& matrix_sa, const Eigen::MatrixXd& matrix_sb,
                               const Eigen::MatrixXd& matrix_sc, const Eigen::MatrixXd& matrix_sd) {
    LogMatrix(matrix_ad, "matrix_ad");
    LogMatrix(matrix_bd, "matrix_bd");
    LogMatrix(matrix_cd, "matrix_cd");
    LogMatrix(matrix_dd, "matrix_dd");
    LogMatrix(matrix_sa, "matrix_sa");
    LogMatrix(matrix_sb, "matrix_sb");
    LogMatrix(matrix_sc, "matrix_sc");
    LogMatrix(matrix_sd, "matrix_sd");
}

void MPCKernel::LogQRMatrix(const Eigen::MatrixXd& matrix_q, const Eigen::MatrixXd& matrix_r1,
                            const Eigen::MatrixXd& matrix_r2) {
    LogMatrix(matrix_q, "matrix_q");
    LogMatrix(matrix_r1, "matrix_r1");
    LogMatrix(matrix_r2, "matrix_r2");
}

void MPCKernel::LogMatrix(const Eigen::MatrixXd& matrix_x, std::string name) {
    LOG_INFO("%s", name.c_str());
    std::string str{""};
    for (int32_t i = 0; i < matrix_x.rows(); ++i) {
        str = "";
        for (int32_t j = 0; j < matrix_x.cols(); ++j) {
            str += std::to_string(matrix_x(i, j)) + ", ";
        }
        LOG_INFO("%s", str.c_str());
    }
}

}  // namespace control
}  // namespace jiduauto
