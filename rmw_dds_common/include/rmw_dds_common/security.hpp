// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMW_DDS_COMMON__SECURITY_HPP_
#define RMW_DDS_COMMON__SECURITY_HPP_

#include <string>
#include <unordered_map>

#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common {

/// 获取安全围栏中的一组安全文件。(Get the set of security files in a security enclave.)
/**
 * 该函数将查找传入的 'secure_root' 中一组必须在围栏中的所需文件名。
 * (This function will look through the passed in 'secure_root'
 * for a set of required filenames that must be in the enclave.)
 * 如果缺少任何所需的文件名，'result' 将为空，函数将返回 false。
 * (If any of the required filenames are missing, the 'result'
 * will be empty and the function will return false.)
 * 如果所有所需文件名都存在，则此函数
 * 将使用 friendy name -> filename 的键值对填充 'result' 映射。如果前缀不为空，则
 * 将前缀应用于文件名。
 * (If all of the required filenames are present, then this function
 * will fill in the 'result' map with a key-value pair of
 * friendy name -> filename.  If the prefix is not empty, then
 * the prefix will be applied to the filename.)
 *
 * 此函数当前将填充以下友好名称：(The friendly names that this function will currently fill in are:)
 *   IDENTITY_CA
 *   CERTIFICATE
 *   PRIVATE_KEY
 *   PERMISSIONS_CA
 *   GOVERNANCE
 *   PERMISSIONS
 *
 * \param[in]  prefix 存储它们时要应用于文件名的可选前缀。(An optional prefix to apply to the
 filenames when storing them.)
 * \param[in]  secure_root 要查看的安全围栏的路径。(The path to the security enclave to look at.)
 * \param[out] result 存储友好名称 -> 文件名对的映射。(The map where the friendly name -> filename
 pairs are stored.)
 * \return 如果安全围栏中存在所有必需的文件，则为 `true`，否则为 `false`。(Return `true` if all
 required files exist in the security enclave, `false` otherwise.)
 */
RMW_DDS_COMMON_PUBLIC
bool get_security_files(const std::string& prefix,
                        const std::string& secure_root,
                        std::unordered_map<std::string, std::string>& result);

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__SECURITY_HPP_
