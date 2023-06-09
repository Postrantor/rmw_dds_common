// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rmw_dds_common/time_utils.hpp"

#include <climits>

#include "rcutils/logging_macros.h"
#include "rmw/types.h"

/**
 * @brief 将 rmw_time_t 时间限制在 DDS 时间范围内 (Clamp rmw_time_t time to DDS time range)
 *
 * @param time 输入的 rmw_time_t 类型时间 (The input rmw_time_t type time)
 * @return rmw_time_t 返回调整后的时间 (The adjusted time after clamping)
 */
rmw_time_t rmw_dds_common::clamp_rmw_time_to_dds_time(const rmw_time_t& time) {
  // 复制输入时间 (Copy the input time)
  rmw_time_t t = time;

  // 规范化 rmw_time_t::nsec 使其小于 1s，以便可以安全地转换为 DDS Duration 或 Time（参见 DDS v1.4
  // 第 2.3.2 节）。 如果总长度（以秒为单位）不能由 DDS 表示（仅限于 INT_MAX 秒 + (10^9 - 1)
  // 纳秒），我们必须将秒数截断为 INT_MAX，同时将纳秒规范化为 < 1s。 (Normalize rmw_time_t::nsec to
  // be < 1s, so that it may be safely converted to a DDS Duration or Time (see DDS v1.4
  // section 2.3.2). If the total length in seconds cannot be represented by DDS (which is limited
  // to INT_MAX seconds + (10^9 - 1) nanoseconds), we must truncate the seconds component at
  // INT_MAX, while also normalizing nanosec to < 1s.)
  constexpr uint64_t sec_to_ns = 1000000000ULL;
  uint64_t ns_sec_adjust = t.nsec / sec_to_ns;
  bool overflow_nsec = false;
  bool overflow_sec = false;

  // 检查纳秒调整值是否超过 INT_MAX (Check if the nanosecond adjustment value exceeds INT_MAX)
  if (ns_sec_adjust > INT_MAX) {
    ns_sec_adjust = INT_MAX;
    overflow_nsec = true;
  }

  // 检查秒数是否超过 INT_MAX 减去纳秒调整值 (Check if the seconds value exceeds INT_MAX minus the
  // nanosecond adjustment value)
  if (t.sec > INT_MAX - ns_sec_adjust) {
    t.sec = INT_MAX;
    overflow_sec = true;
  } else {
    t.sec += ns_sec_adjust;
  }

  // 如果溢出（以纳秒或秒为单位），需要将 nsec 部分设置为 "饱和" 值，即 sec_to_ns - 1
  // (If there is an overflow (in nanoseconds or seconds), the nsec component must be set to the
  // "saturated" value, i.e., sec_to_ns - 1)
  if (overflow_nsec || overflow_sec) {
    t.nsec = sec_to_ns - 1;
    RCUTILS_LOG_DEBUG_NAMED(
        "rmw_dds_common",
        "rmw_time_t length cannot be represented by DDS, truncated at "
        "INT_MAX seconds + (10^9 - 1) nanoseconds");
  } else {
    t.nsec = t.nsec - ns_sec_adjust * sec_to_ns;
  }

  // 返回调整后的时间 (Return the adjusted time)
  return t;
}
