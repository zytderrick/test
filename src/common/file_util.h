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

/**
 * @file
 * @brief Defines the FileUtil interface class.
 */
#pragma once

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

class FileUtil {
public:
    FileUtil() = delete;
    ~FileUtil() = delete;

    static bool ReadPnCPath();

private:
    static bool binitialized_;
};

}  // namespace control
}  // namespace jiduauto
