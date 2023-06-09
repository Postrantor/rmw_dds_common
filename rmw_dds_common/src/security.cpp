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

#include "rmw_dds_common/security.hpp"

#include <string>
#include <unordered_map>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"

namespace rmw_dds_common {

/**
 * @brief 获取安全文件 (Get security files)
 *
 * @param[in] prefix 文件名前缀 (File name prefix)
 * @param[in] secure_root 安全文件根目录 (Secure root directory)
 * @param[out] result 存储获取到的安全文件的键值对 (A map to store the obtained security files
 * key-value pairs)
 * @return bool 如果成功获取所有必需的安全文件，则返回 true，否则返回 false (Return true if all
 * required security files are successfully obtained, otherwise return false)
 */
bool get_security_files(
    const std::string& prefix,
    const std::string& secure_root,
    std::unordered_map<std::string, std::string>& result) {
  // 必需的安全文件 (Required security files)
  const std::unordered_map<std::string, std::string> required_files{
      {"IDENTITY_CA", "identity_ca.cert.pem"},
      {"CERTIFICATE", "cert.pem"},
      {"PRIVATE_KEY", "key.pem"},
      {"PERMISSIONS_CA", "permissions_ca.cert.pem"},
      {"GOVERNANCE", "governance.p7s"},
      {"PERMISSIONS", "permissions.p7s"},
  };

  // 可选的安全文件 (Optional security files)
  const std::unordered_map<std::string, std::string> optional_files{
      {"CRL", "crl.pem"},
  };

  // 遍历必需的安全文件 (Iterate through the required security files)
  for (const std::pair<const std::string, std::string>& el : required_files) {
    rcpputils::fs::path full_path(secure_root);  // 构建完整路径 (Construct the full path)
    full_path /= el.second;
    if (!full_path.is_regular_file()) {  // 检查文件是否存在 (Check if the file exists)
      result.clear();                    // 清除结果 (Clear the result)
      return false;                      // 返回失败 (Return failure)
    }

    result[el.first] = prefix + full_path.string();  // 将成功找到的文件添加到结果中 (Add the
                                                     // successfully found file to the result)
  }

  // 遍历可选的安全文件 (Iterate through the optional security files)
  for (const std::pair<const std::string, std::string>& el : optional_files) {
    rcpputils::fs::path full_path(secure_root);  // 构建完整路径 (Construct the full path)
    full_path /= el.second;
    if (full_path.is_regular_file()) {  // 检查文件是否存在 (Check if the file exists)
      result[el.first] = prefix + full_path.string();  // 将成功找到的文件添加到结果中 (Add the
                                                       // successfully found file to the result)
    }
  }

  return true;  // 返回成功 (Return success)
}

}  // namespace rmw_dds_common
