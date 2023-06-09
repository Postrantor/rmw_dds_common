// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rmw_dds_common/gid_utils.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

#include "rmw/types.h"
#include "rmw_dds_common/msg/gid.hpp"

using rmw_dds_common::Compare_rmw_gid_t;

/**
 * @brief 比较两个 rmw_gid_t 对象的字典序大小 (Compare two rmw_gid_t objects lexicographically)
 *
 * @param lhs 第一个 rmw_gid_t 对象 (The first rmw_gid_t object)
 * @param rhs 第二个 rmw_gid_t 对象 (The second rmw_gid_t object)
 * @return 如果 lhs 在字典序上小于 rhs，则返回 true，否则返回 false (Return true if lhs is
 * lexicographically less than rhs, false otherwise)
 */
bool Compare_rmw_gid_t::operator()(const rmw_gid_t &lhs, const rmw_gid_t &rhs) const {
  // 使用 std::lexicographical_compare 比较两个 rmw_gid_t 对象的 data 数组的字典序大小 (Use
  // std::lexicographical_compare to compare the lexicographical order of the data arrays of two
  // rmw_gid_t objects)
  return std::lexicographical_compare(
      lhs.data, lhs.data + RMW_GID_STORAGE_SIZE, rhs.data, rhs.data + RMW_GID_STORAGE_SIZE);
}

/**
 * @brief 将 rmw_gid_t 对象输出到 std::ostream (Output an rmw_gid_t object to a std::ostream)
 *
 * @param ostream 输出流 (The output stream)
 * @param gid 要输出的 rmw_gid_t 对象 (The rmw_gid_t object to output)
 * @return 返回输出流 (Return the output stream)
 */
std::ostream &rmw_dds_common::operator<<(std::ostream &ostream, const rmw_gid_t &gid) {
  ostream << std::hex;  // 设置输出流为十六进制格式 (Set the output stream to hexadecimal format)

  size_t i = 0;
  // 遍历 gid.data 数组，输出每个元素的十六进制表示 (Iterate through the gid.data array, outputting
  // the hexadecimal representation of each element)
  for (; i < (RMW_GID_STORAGE_SIZE - 1); i++) {
    ostream << static_cast<int>(gid.data[i]) << ".";
  }
  // 输出最后一个元素 (Output the last element)
  ostream << static_cast<int>(gid.data[i]);

  return ostream << std::dec;  // 设置输出流为十进制格式并返回 (Set the output stream to decimal
                               // format and return)
}

/**
 * @brief 判断两个 rmw_gid_t 对象是否相等 (Determine if two rmw_gid_t objects are equal)
 *
 * @param lhs 第一个 rmw_gid_t 对象 (The first rmw_gid_t object)
 * @param rhs 第二个 rmw_gid_t 对象 (The second rmw_gid_t object)
 * @return 如果两个对象相等，则返回 true，否则返回 false (Return true if the two objects are equal,
 * false otherwise)
 */
bool rmw_dds_common::operator==(const rmw_gid_t &lhs, const rmw_gid_t &rhs) {
  // 使用 std::memcmp 比较两个 rmw_gid_t 对象的 data 数组是否相等 (Use std::memcmp to compare
  // whether the data arrays of two rmw_gid_t objects are equal)
  return std::memcmp(
             reinterpret_cast<const void *>(lhs.data), reinterpret_cast<const void *>(rhs.data),
             RMW_GID_STORAGE_SIZE) == 0;
}

/**
 * @brief 将 rmw_gid_t 对象转换为 Gid 消息对象 (Convert an rmw_gid_t object to a Gid message object)
 *
 * @param gid 输入的 rmw_gid_t 对象 (The input rmw_gid_t object)
 * @param msg_gid 输出的 Gid 消息对象 (The output Gid message object)
 */
void rmw_dds_common::convert_gid_to_msg(const rmw_gid_t *gid, rmw_dds_common::msg::Gid *msg_gid) {
  assert(nullptr != gid);  // 确保输入的 rmw_gid_t 对象不为 nullptr (Ensure the input rmw_gid_t
                           // object is not nullptr)
  assert(nullptr != msg_gid);  // 确保输出的 Gid 消息对象不为 nullptr (Ensure the output Gid message
                               // object is not nullptr)

  // 使用 std::memcpy 将 gid 的 data 数组复制到 msg_gid 的 data 数组中 (Use std::memcpy to copy the
  // data array of gid to the data array of msg_gid)
  std::memcpy(&msg_gid->data, gid->data, RMW_GID_STORAGE_SIZE);
}

/**
 * @brief 将 Gid 消息对象转换为 rmw_gid_t 对象 (Convert a Gid message object to an rmw_gid_t object)
 *
 * @param msg_gid 输入的 Gid 消息对象 (The input Gid message object)
 * @param gid 输出的 rmw_gid_t 对象 (The output rmw_gid_t object)
 */
void rmw_dds_common::convert_msg_to_gid(const rmw_dds_common::msg::Gid *msg_gid, rmw_gid_t *gid) {
  assert(nullptr != msg_gid);  // 确保输入的 Gid 消息对象不为 nullptr (Ensure the input Gid message
                               // object is not nullptr)
  assert(nullptr != gid);  // 确保输出的 rmw_gid_t 对象不为 nullptr (Ensure the output rmw_gid_t
                           // object is not nullptr)

  // 使用 std::memcpy 将 msg_gid 的 data 数组复制到 gid 的 data 数组中 (Use std::memcpy to copy the
  // data array of msg_gid to the data array of gid)
  std::memcpy(const_cast<uint8_t *>(gid->data), &msg_gid->data, RMW_GID_STORAGE_SIZE);
}
