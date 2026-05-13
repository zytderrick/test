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

#include "control/src/common/kinematic_model/types.h"

namespace jiduauto {
namespace control {

class KinematicModel {
public:
    KinematicModel(const double sample_time, const double ll) : Ts_(sample_time), ll_(ll) {}

    KinematicStateVector NonlinearEvalulate(const KinematicModelState& x, const KinematicModelInput& u) const;

    KinLinModelMatrix getDiscreteLinModel(const KinematicModelState& x, const KinematicModelInput& u) const;

private:
    KinLinModelMatrix getModelJacobian(const KinematicModelState& x, const KinematicModelInput& u) const;
    KinLinModelMatrix discretizeModel(const KinLinModelMatrix& lin_model_conti) const;

    const double Ts_;
    const double ll_;
};

}  // namespace control
}  // namespace jiduauto
