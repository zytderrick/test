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

class Interpolation1D {
public:
    typedef std::vector<std::pair<double, double>> DataType;

    Interpolation1D() = default;

    bool Init(const DataType& xy);

    double LinearInterpolate(double x) const;
    double LinearInterExtrapolate(double x) const;
    double GridMin() const { return x_min_; }
    double GridMax() const { return x_max_; }

private:
    double x_min_ = 0.0;
    double x_max_ = 0.0;
    double y_start_ = 0.0;
    double y_end_ = 0.0;
    std::vector<double> xs_;
    std::vector<double> ys_;
};

}  // namespace control
}  // namespace jiduauto
