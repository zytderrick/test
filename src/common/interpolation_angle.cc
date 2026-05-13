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

#include "control/src/common/interpolation_angle.h"

#include <algorithm>

#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/math_utility/math_utils.h"

namespace jiduauto {
namespace control {

bool InterpolationAngle::Init(const DataType& xy) {
    if (xy.empty() || static_cast<int>(xy.size()) <= 1) {
        LOG_ERROR("wrong input length.");
        return false;
    }
    xs_.clear();
    ys_.clear();
    auto data(xy);
    std::sort(data.begin(), data.end());
    for (unsigned i = 0; i < data.size(); ++i) {
        xs_.emplace_back(data[i].first);
        if (i > 0) {
            ys_.emplace_back(data[i - 1].second +
                             jiduauto::pnc::math::NormalizeAngle(data[i].second - data[i - 1].second));
        } else {
            ys_.emplace_back(data[i].second);
        }
    }
    x_min_ = data.front().first;
    x_max_ = data.back().first;
    y_start_ = data.front().second;
    y_end_ = data.back().second;

    return true;
}

double InterpolationAngle::LinearInterpolate(double x) const {
    if (x <= x_min_) {
        return y_start_;
    }
    if (x >= x_max_) {
        return y_end_;
    }
    int i = 0;
    double y;
    while (x > xs_[i]) {
        i++;
    }
    y = jiduauto::pnc::math::NormalizeAngle((x - xs_[i]) * (ys_[i] - ys_[i - 1]) / (xs_[i] - xs_[i - 1]) + ys_[i]);
    return y;
}

double InterpolationAngle::LinearInterExtrapolate(double x) const {
    int i = 1;
    double y;
    while (x > xs_[i] && i < static_cast<int>(xs_.size()) - 1) {
        i++;
    }
    y = jiduauto::pnc::math::NormalizeAngle((x - xs_[i - 1]) * (ys_[i] - ys_[i - 1]) / (xs_[i] - xs_[i - 1]) +
                                            ys_[i - 1]);
    return y;
}

}  // namespace control
}  // namespace jiduauto
