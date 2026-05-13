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
#include <utility>
#include <vector>

namespace jiduauto {
namespace control {

class InterpolationXY {
public:
    typedef std::vector<std::pair<double, double>> InterpolatData;

    InterpolationXY();

    ~InterpolationXY();

    bool Init(const InterpolatData& xy);

    double Interpolate(const double& x) const;

private:
    double GetInterploateValue(const double& x, const size_t& id) const;

    double x_min_ = 0.0;
    double x_max_ = 0.0;
    double y_start_ = 0.0;
    double y_end_ = 0.0;

    std::vector<double> x_list_;
    std::vector<double> y_list_;
};

}  // namespace control
}  // namespace jiduauto
