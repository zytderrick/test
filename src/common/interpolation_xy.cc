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

#include "control/src/common/interpolation_xy.h"

#include <algorithm>

#include "pnc_common/src/common/pnc_logger.h"

namespace jiduauto {
namespace control {

InterpolationXY::InterpolationXY() {}

InterpolationXY::~InterpolationXY() {}

bool InterpolationXY::Init(const InterpolatData& xy) {
    if (xy.empty()) {
        LOG_ERROR("empty input");
        return false;
    }

    auto data(xy);
    std::sort(data.begin(), data.end());
    x_list_.clear();
    y_list_.clear();
    for (unsigned i = 0; i < data.size(); ++i) {
        x_list_.emplace_back(data[i].first);
        y_list_.emplace_back(data[i].second);
    }

    x_min_ = data.front().first;
    x_max_ = data.back().first;
    y_start_ = data.front().second;
    y_end_ = data.back().second;

    return true;
}

double InterpolationXY::Interpolate(const double& x) const {
    if (x <= x_min_) {
        return y_start_;
    }
    if (x >= x_max_) {
        return y_end_;
    }

    size_t i = 1;
    while (x > x_list_[i]) {
        i++;
    }
    return GetInterploateValue(x, i);
}

double InterpolationXY::GetInterploateValue(const double& x, const size_t& id) const {
    return y_list_[id - 1] + (x - x_list_[id - 1]) * (y_list_[id] - y_list_[id - 1]) / (x_list_[id] - x_list_[id - 1]);
}

}  // namespace control
}  // namespace jiduauto
