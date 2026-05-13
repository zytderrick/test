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

#include "control/src/common/interpolation_xy.h"
#include "control/src/controller/controller.h"
#include "pnc_common/src/math/filters/digital_filter.h"
#include "pnc_common_vehicle_param.pb.h"

namespace jiduauto {
namespace control {

/**
 * @brief GeoLateralController, using geometric method to calculate steer angle.
 */
class GeoLateralController : public Controller {
public:
    GeoLateralController();
    ~GeoLateralController() override;

    bool Init(const pnc::control::ControlConfig& control_conf) override;

    bool Control(const pnc::localization::LocalizationEstimate* const localization,
                 const pnc::chassis::Chassis* const chassis, const pnc::planning::ADCTrajectory* const trajectory,
                 pnc::control::ControlCommand* const cmd) override;

    bool Reset() override;
    bool ControlMain(const ControlInputStream& control_input_stream,
                     ControlCommandStream* const control_command_stream) override;

    std::string Name() const override;

    void Stop() override;

private:
    double CalculateLookaheadDistance();

    double CalculateLookaheadDistanceRatio();

    bool CalculateSteerCmd();

    bool CalculateNextTargetPoint(const std::vector<pnc::common::TrajectoryPoint>& trajectory_points,
                                  double lookahead_distance, int32_t* next_target_point_id);

    bool InterpolateNextTargetPoint(int32_t next_target_point_id,
                                    const std::vector<pnc::common::TrajectoryPoint>& trajectory_points,
                                    pnc::common::TrajectoryPoint* target_point);

    double CalculateCurvature(const pnc::common::TrajectoryPoint& target_point);

    pnc::common::TrajectoryPoint ConvertToRelativeCoordinate(const pnc::common::TrajectoryPoint& pt_in_world);

    bool GetLinearEquation(const pnc::common::TrajectoryPoint& start, const pnc::common::TrajectoryPoint& end,
                           double* a, double* b, double* c);

private:
    pnc::common::VehicleParam vehicle_param_;
    pnc::control::GeoLatControllerConf geo_lat_controller_conf_;
    std::shared_ptr<InterpolationXY> ratio_interpolation_ = std::make_shared<InterpolationXY>();
    const pnc::planning::ADCTrajectory* trajectory_msg_{nullptr};
    const pnc::chassis::Chassis* chassis_msg_{nullptr};

    double current_speed_{0.0};
    double lookahead_distance_{0.0};
    const pnc::localization::LocalizationEstimate* localization_msg_{nullptr};
    double current_lateral_error_{0.0};
    double target_steer_angle_{0.0};
    double target_steer_angle_rate_{0.0};
    double pre_target_steer_angle_{0.0};
    pnc::filter::DigitalFilter target_steer_angle_filter_;
    bool controller_initialized_{false};
    bool is_current_backward_{false};
};

}  // namespace control
}  // namespace jiduauto
