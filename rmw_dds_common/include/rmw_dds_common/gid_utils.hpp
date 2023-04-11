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

#ifndef RMW_DDS_COMMON__GID_UTILS_HPP_
#define RMW_DDS_COMMON__GID_UTILS_HPP_

#include "rmw/types.h"
#include "rmw_dds_common/msg/gid.hpp"
#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common {

/// \file rmw_gid_t_utils.h

/// 比较器 (Comparator) 用于 rmw_gid_t，以便将它们作为 map 的键
/// Comparator for rmw_gid_t, in order to use them as a key of a map
struct RMW_DDS_COMMON_PUBLIC_TYPE Compare_rmw_gid_t {
  /// 比较 lhs 和 rhs。
  /// Compare lhs with rhs.
  bool operator()(const rmw_gid_t& lhs, const rmw_gid_t& rhs) const;
};

/// rmw_gid_t 的流操作符
/// Stream operator for rmw_gid_t
RMW_DDS_COMMON_PUBLIC
std::ostream& operator<<(std::ostream& ostream, const rmw_gid_t& gid);

/// rmw_gid_t 的等于操作符
/// operator== for rmw_gid_t
RMW_DDS_COMMON_PUBLIC
bool operator==(const rmw_gid_t& lhs, const rmw_gid_t& rhs);

/// \internal 将 rmw_gid_t 转换为 rmw_dds_common::msg::Gid
/// \internal Converts from rmw_gid_t to rmw_dds_common::msg::Gid
/**
 * 仅供内部使用，假定两个指针都是有效的。
 * For internal usage, both pointers are assumed to be valid.
 */
RMW_DDS_COMMON_PUBLIC
void convert_gid_to_msg(const rmw_gid_t* gid, rmw_dds_common::msg::Gid* msg_gid);

/// \internal 将 rmw_dds_common::msg::Gid 转换为 rmw_gid_t
/// \internal Converts from rmw_dds_common::msg::Gid to rmw_gid_t
/**
 * 仅供内部使用，假定两个指针都是有效的。
 * For internal usage, both pointers are supposed to be valid.
 */
RMW_DDS_COMMON_PUBLIC
void convert_msg_to_gid(const rmw_dds_common::msg::Gid* msg_gid, rmw_gid_t* gid);

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__GID_UTILS_HPP_
