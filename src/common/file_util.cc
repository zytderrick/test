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

#include "control/src/common/file_util.h"

#include <stdlib.h>

#include <sstream>
#include <string>
#include <vector>

#include "control/src/common/control_gflags.h"
#include "pnc_common/src/common/pnc_gflag.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/utility/file.h"
#include "pnc_common/src/utility/pnc_utility.h"

namespace jiduauto {
namespace control {

bool FileUtil::binitialized_ = false;

bool FileUtil::ReadPnCPath() {
    if (binitialized_) {
        return true;
    }
    std::string flag_file_name = pnc::util::PnCUtility::GetPncPath();
    if (flag_file_name == "") {
        return false;
    }

    // test here
    FLAGS_control_gflag_file = flag_file_name + FLAGS_control_gflag_file;
    google::SetCommandLineOption("flagfile", FLAGS_control_gflag_file.c_str());

    FLAGS_control_conf_path = flag_file_name + FLAGS_control_conf_path;
    FLAGS_control_unit_test_data_path = flag_file_name + FLAGS_control_unit_test_data_path;
    binitialized_ = true;

    // glog config
    if (FLAGS_enable_glog_mode) {
        pnc::util::PnCUtility::InitGoogleLoggingDir(FLAGS_control_node_name);
    }
    return true;
}

}  // namespace control
}  // namespace jiduauto
