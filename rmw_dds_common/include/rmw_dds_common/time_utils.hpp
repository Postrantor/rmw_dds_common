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

#ifndef RMW_DDS_COMMON__TIME_UTILS_HPP_
#define RMW_DDS_COMMON__TIME_UTILS_HPP_

#include "rmw/types.h"
#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common {

/// 检查rmw_time_t的64位秒/纳秒值是否超过32位限制，并进行相应调整
/// Check if an rmw_time_t's 64-bit seconds/nanoseconds values exceed
/// 32-bit limits and adjust accordingly
/**
 * DDS标准将Time_t和Duration_t类型指定为
 * "long sec" 和 "unsigned long nanosec"，而IDL到C++11映射
 * 规定long和unsigned long的C++类型分别映射到
 * int32_t 和 uint32_t。因此，在将rmw_time_t转换为
 * DDS Duration_t 或 Time_t 时，64位秒和纳秒值将被截断为32位。
 * 此函数将秒值限制为有符号的32位最大值，并尝试将多余的秒数移动到纳秒字段。
 * 如果生成的纳秒值对于无符号32位来说太大，则会饱和并发出警告。
 *
 * The DDS standard specifies the Time_t and Duration_t types with
 * "long sec" and "unsigned long nanosec", and the IDL to C++11 mapping
 * states that the C++ types for long and unsigned long are mapped
 * to int32_t and uint32_t, respectively. So, the 64-bit seconds and
 * nanoseconds values will be truncated to 32-bits when converted
 * from rmw_time_t to either a DDS Duration_t or Time_t. This function
 * limits the seconds value to the signed 32-bit maximum and attempts
 * to move the excess seconds to the nanoseconds field. If the resulting
 * nanoseconds value is too large for unsigned 32-bits it saturates
 * and issues a warning.
 *
 * \param[in] time 要转换的时间
 * \param[in] time to convert
 * \return 转换后的时间值
 * \return converted time value
 */
RMW_DDS_COMMON_PUBLIC
rmw_time_t clamp_rmw_time_to_dds_time(const rmw_time_t& time);

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__TIME_UTILS_HPP_
