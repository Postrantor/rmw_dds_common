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

#include "rmw_dds_common/qos.hpp"

#include <cstdarg>
#include <cstring>
#include <string>
#include <vector>

#include "rcpputils/scope_exit.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/snprintf.h"
#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/qos_profiles.h"
#include "rmw/qos_string_conversions.h"

namespace rmw_dds_common {

// 比较两个 rmw_time_t 对象是否相等
/**
 * @brief 比较两个 rmw_time_t 对象是否相等
 * @param t1 第一个 rmw_time_t 对象
 * @param t2 第二个 rmw_time_t 对象
 * @return 如果两个对象相等，返回 true，否则返回 false
 *
 * @brief Compare two rmw_time_t objects for equality
 * @param t1 The first rmw_time_t object
 * @param t2 The second rmw_time_t object
 * @return Returns true if the two objects are equal, otherwise returns false
 */
static bool operator==(rmw_time_t t1, rmw_time_t t2) {
  // 比较两个对象的 sec 和 nsec 成员是否都相等
  // Compare if both sec and nsec members of the two objects are equal
  return t1.sec == t2.sec && t1.nsec == t2.nsec;
}

// 比较两个 rmw_time_t 对象是否不相等
/**
 * @brief 比较两个 rmw_time_t 对象是否不相等
 * @param t1 第一个 rmw_time_t 对象
 * @param t2 第二个 rmw_time_t 对象
 * @return 如果两个对象不相等，返回 true，否则返回 false
 *
 * @brief Compare two rmw_time_t objects for inequality
 * @param t1 The first rmw_time_t object
 * @param t2 The second rmw_time_t object
 * @return Returns true if the two objects are not equal, otherwise returns false
 */
static bool operator!=(rmw_time_t t1, rmw_time_t t2) { return !(t1 == t2); }

// 比较第一个 rmw_time_t 对象是否小于第二个 rmw_time_t 对象
/**
 * @brief 比较第一个 rmw_time_t 对象是否小于第二个 rmw_time_t 对象
 * @param t1 第一个 rmw_time_t 对象
 * @param t2 第二个 rmw_time_t 对象
 * @return 如果 t1 小于 t2，返回 true，否则返回 false
 *
 * @brief Compare if the first rmw_time_t object is less than the second rmw_time_t object
 * @param t1 The first rmw_time_t object
 * @param t2 The second rmw_time_t object
 * @return Returns true if t1 is less than t2, otherwise returns false
 */
static bool operator<(rmw_time_t t1, rmw_time_t t2) {
  // 如果 t1 的 sec 成员小于 t2 的 sec 成员，则返回 true
  // If the sec member of t1 is less than the sec member of t2, return true
  if (t1.sec < t2.sec) {
    return true;
  } else if (t1.sec == t2.sec && t1.nsec < t2.nsec) {
    // 如果两个对象的 sec 成员相等，且 t1 的 nsec 成员小于 t2 的 nsec 成员，则返回 true
    // If the sec members of both objects are equal and the nsec member of t1 is less than the nsec
    // member of t2, return true
    return true;
  }
  // 其他情况返回 false
  // In other cases, return false
  return false;
}

// 定义 deadline_default 常量为 RMW_QOS_DEADLINE_DEFAULT
// Define the deadline_default constant as RMW_QOS_DEADLINE_DEFAULT
static const rmw_time_t deadline_default = RMW_QOS_DEADLINE_DEFAULT;

// 定义 deadline_best_available 常量为 RMW_QOS_DEADLINE_BEST_AVAILABLE
// Define the deadline_best_available constant as RMW_QOS_DEADLINE_BEST_AVAILABLE
static const rmw_time_t deadline_best_available = RMW_QOS_DEADLINE_BEST_AVAILABLE;

// 定义 liveliness_lease_duration_default 常量为 RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT
// Define the liveliness_lease_duration_default constant as
// RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT
static const rmw_time_t liveliness_lease_duration_default =
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;

// 定义 liveliness_lease_duration_best_available 常量为
// RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE Define the
// liveliness_lease_duration_best_available constant as
// RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE
static const rmw_time_t liveliness_lease_duration_best_available =
    RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE;

// 将格式化字符串添加到缓冲区中（Append a formatted string to the buffer）
/**
 * @brief 将格式化字符串添加到缓冲区中（Append a formatted string to the buffer）
 *
 * @param[in] buffer 缓冲区指针（Pointer to the buffer）
 * @param[in] buffer_size 缓冲区的大小（Size of the buffer）
 * @param[in] format 格式化字符串（Formatted string）
 * @param[in] ... 可变参数列表，与格式化字符串匹配（Variable argument list, matching the formatted
 * string）
 *
 * @return RMW_RET_OK 如果成功或没有提供缓冲区（if successful or no buffer was provided）
 * @return RMW_RET_ERROR 如果在将消息复制到缓冲区时出现错误（if there was an error copying the
 * message to the buffer）
 */
static rmw_ret_t _append_to_buffer(char* buffer, size_t buffer_size, const char* format, ...) {
  // 只在提供缓冲区时写入（Only write if a buffer is provided）
  if (!buffer || buffer_size == 0u) {
    return RMW_RET_OK;
  }
  // 确定缓冲区中剩余的可用空间（Determine available space left in buffer）
  size_t offset = strnlen(buffer, buffer_size);
  size_t write_size = buffer_size - offset;

  // 初始化可变参数列表（Initialize variable argument list）
  std::va_list args;
  va_start(args, format);

  // 将格式化字符串追加到缓冲区（Append the formatted string to the buffer）
  int snprintf_ret = rcutils_vsnprintf(buffer + offset, write_size, format, args);

  // 清理可变参数列表（Clean up variable argument list）
  va_end(args);

  // 如果追加失败，设置错误消息并返回 RMW_RET_ERROR（If appending fails, set error message and
  // return RMW_RET_ERROR）
  if (snprintf_ret < 0) {
    RMW_SET_ERROR_MSG("failed to append to character buffer");
    return RMW_RET_ERROR;
  }

  // 成功完成操作，返回 RMW_RET_OK（Successfully completed operation, return RMW_RET_OK）
  return RMW_RET_OK;
}

// 检查发布者和订阅者的 QoS 配置是否兼容 (Check if the publisher and subscriber QoS configurations
// are compatible)
/**
 * @brief 检查发布者和订阅者的 QoS 配置是否兼容 (Check if the publisher and subscriber QoS
 * configurations are compatible)
 *
 * @param[in] publisher_qos 发布者的 QoS 配置 (Publisher's QoS configuration)
 * @param[in] subscription_qos 订阅者的 QoS 配置 (Subscriber's QoS configuration)
 * @param[out] compatibility 兼容性结果，指示两个 QoS 配置是否兼容 (Compatibility result, indicating
 * whether the two QoS configurations are compatible)
 * @param[out] reason 如果不兼容，提供原因 (Reason for incompatibility, if any)
 * @param[in] reason_size 原因缓冲区的大小 (Size of the reason buffer)
 * @return rmw_ret_t 返回操作状态 (Return operation status)
 */
rmw_ret_t qos_profile_check_compatible(
    const rmw_qos_profile_t publisher_qos,
    const rmw_qos_profile_t subscription_qos,
    rmw_qos_compatibility_type_t* compatibility,
    char* reason,
    size_t reason_size) {
  // 检查兼容性参数是否为空 (Check if the compatibility parameter is null)
  if (!compatibility) {
    // 设置错误消息 (Set error message)
    RMW_SET_ERROR_MSG("compatibility parameter is null");
    // 返回无效参数错误 (Return invalid argument error)
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 当 reason 参数为空且 reason_size 不为零时设置错误消息
  // Set error message when reason parameter is null and reason_size is not zero
  if (!reason && reason_size != 0u) {
    RMW_SET_ERROR_MSG("reason parameter is null, but reason_size parameter is not zero");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 假设配置文件默认兼容，除非证明有问题
  // Presume profiles are compatible until proven otherwise
  *compatibility = RMW_QOS_COMPATIBILITY_OK;

  // 初始化 reason 缓冲区
  // Initialize reason buffer
  if (reason && reason_size != 0u) {
    reason[0] = '\0';
  }

  // 最佳尝试发布者和可靠订阅者
  // Best effort publisher and reliable subscription
  if (publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
      subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
        reason, reason_size, "ERROR: Best effort publisher and reliable subscription;");
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  // 易失性发布者和本地瞬态订阅者
  // Volatile publisher and transient local subscription
  if (publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
      subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
        reason, reason_size, "ERROR: Volatile publisher and transient local subscription;");
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  const rmw_time_t& pub_deadline = publisher_qos.deadline;
  const rmw_time_t& sub_deadline = subscription_qos.deadline;

  // 发布者没有截止日期，订阅者有截止日期
  // No deadline for publisher and deadline for subscription
  if (pub_deadline == deadline_default && sub_deadline != deadline_default) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t ret = _append_to_buffer(
        reason, reason_size, "ERROR: Subscription has a deadline, but publisher does not;");
    if (RMW_RET_OK != ret) {
      return ret;
    }
  }

  // 订阅者的 deadline 小于发布者的 deadline (Subscription deadline is less than publisher deadline)
  if (pub_deadline != deadline_default && sub_deadline != deadline_default) {
    if (sub_deadline < pub_deadline) {
      *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
      // 添加错误原因到缓冲区 (Append error reason to the buffer)
      rmw_ret_t append_ret = _append_to_buffer(
          reason, reason_size, "ERROR: Subscription deadline is less than publisher deadline;");
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    }
  }

  // 发布者的活跃度为自动，订阅者的活跃度为按主题手动 (Automatic liveliness for publisher and manual
  // by topic for subscription)
  if (publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_AUTOMATIC &&
      subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    // 添加错误原因到缓冲区 (Append error reason to the buffer)
    rmw_ret_t append_ret = _append_to_buffer(
        reason, reason_size,
        "ERROR: Publisher's liveliness is automatic and subscription's is manual by topic;");
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  const rmw_time_t& pub_lease = publisher_qos.liveliness_lease_duration;
  const rmw_time_t& sub_lease = subscription_qos.liveliness_lease_duration;

  // 发布者没有租约持续时间，订阅者有租约持续时间 (No lease duration for publisher and lease
  // duration for subscription)
  if (pub_lease == liveliness_lease_duration_default &&
      sub_lease != liveliness_lease_duration_default) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    // 添加错误原因到缓冲区 (Append error reason to the buffer)
    rmw_ret_t append_ret = _append_to_buffer(
        reason, reason_size,
        "ERROR: Subscription has a liveliness lease duration, but publisher does not;");
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  // 订阅者的租约持续时间小于发布者的租约持续时间 (Subscription lease duration is less than
  // publisher lease duration)
  if (pub_lease != liveliness_lease_duration_default &&
      sub_lease != liveliness_lease_duration_default) {
    if (sub_lease < pub_lease) {
      *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
      // 添加错误原因到缓冲区 (Append error reason to the buffer)
      rmw_ret_t append_ret = _append_to_buffer(
          reason, reason_size,
          "ERROR: Subscription liveliness lease duration is less than publisher;");
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    }
  }

  // 只在没有错误的情况下检查警告 (Only check for warnings if there are no errors)
  if (RMW_QOS_COMPATIBILITY_OK == *compatibility) {
    // 如果值为 "system default" 或 "unknown"，我们不知道策略 (We don't know the policy if the value
    // is "system default" or "unknown") 检查发布者可靠性是否未知 (Check if publisher reliability is
    // unknown)
    const bool pub_reliability_unknown =
        publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ||
        publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    // 检查订阅者可靠性是否未知 (Check if subscriber reliability is unknown)
    const bool sub_reliability_unknown =
        subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ||
        subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    // 检查发布者持久性是否未知 (Check if publisher durability is unknown)
    const bool pub_durability_unknown =
        publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ||
        publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    // 检查订阅者持久性是否未知 (Check if subscriber durability is unknown)
    const bool sub_durability_unknown =
        subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ||
        subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    // 检查发布者生命周期是否未知 (Check if publisher liveliness is unknown)
    const bool pub_liveliness_unknown =
        publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
        publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    // 检查订阅者生命周期是否未知 (Check if subscriber liveliness is unknown)
    const bool sub_liveliness_unknown =
        subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
        subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN;

    // 将发布者可靠性策略转换为字符串 (Convert publisher reliability policy to string)
    const char* pub_reliability_str = rmw_qos_reliability_policy_to_str(publisher_qos.reliability);
    if (!pub_reliability_str) {
      pub_reliability_str = "unknown";
    }
    // 将订阅者可靠性策略转换为字符串 (Convert subscriber reliability policy to string)
    const char* sub_reliability_str =
        rmw_qos_reliability_policy_to_str(subscription_qos.reliability);
    if (!sub_reliability_str) {
      sub_reliability_str = "unknown";
    }
    // 将发布者持久性策略转换为字符串 (Convert publisher durability policy to string)
    const char* pub_durability_str = rmw_qos_durability_policy_to_str(publisher_qos.durability);
    if (!pub_durability_str) {
      pub_durability_str = "unknown";
    }
    // 将订阅者持久性策略转换为字符串 (Convert subscriber durability policy to string)
    const char* sub_durability_str = rmw_qos_durability_policy_to_str(subscription_qos.durability);
    if (!sub_durability_str) {
      sub_durability_str = "unknown";
    }
    // 将发布者生命周期策略转换为字符串 (Convert publisher liveliness policy to string)
    const char* pub_liveliness_str = rmw_qos_liveliness_policy_to_str(publisher_qos.liveliness);
    if (!pub_liveliness_str) {
      pub_liveliness_str = "unknown";
    }
    // 将订阅者生命周期策略转换为字符串 (Convert subscriber liveliness policy to string)
    const char* sub_liveliness_str = rmw_qos_liveliness_policy_to_str(subscription_qos.liveliness);
    if (!sub_liveliness_str) {
      sub_liveliness_str = "unknown";
    }

    // 可靠性警告 Reliability warnings
    if (pub_reliability_unknown && sub_reliability_unknown) {
      // 发布者和订阅者的可靠性均未知 Reliability for publisher and subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
          reason, reason_size,
          "WARNING: Publisher reliability is %s and subscription reliability is %s;",
          pub_reliability_str, sub_reliability_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else if (
        pub_reliability_unknown &&  // NOLINT
        subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      // 发布者的可靠性未知，订阅者的可靠性为可靠 Reliability for publisher is unknown and
      // subscription is reliable
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
          reason, reason_size, "WARNING: Reliable subscription, but publisher is %s;",
          pub_reliability_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else if (
        publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&  // NOLINT
        sub_reliability_unknown) {
      // 发布者的可靠性为尽力而为，订阅者的可靠性未知 Reliability for publisher is best effort and
      // subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
          reason, reason_size, "WARNING: Best effort publisher, but subscription is %s;",
          sub_reliability_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    }

    // 持久性警告 Durability warnings
    if (pub_durability_unknown && sub_durability_unknown) {
      // 发布者和订阅者的持久性均未知 Durability for publisher and subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
          reason, reason_size,
          "WARNING: Publisher durabilty is %s and subscription durability is %s;",
          pub_durability_str, sub_durability_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else if (
        pub_durability_unknown &&  // NOLINT
        subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      // 发布者的持久性未知，订阅者的持久性为瞬态本地 Durability for publisher is unknown and
      // subscription is transient local
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
          reason, reason_size, "WARNING: Transient local subscription, but publisher is %s;",
          pub_durability_str);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    } else if (
        publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&  // NOLINT
        sub_durability_unknown) {
      // 发布者的持久性为易失，订阅者的持久性未知 Durability for publisher is volatile and
      // subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
          reason, reason_size, "WARNING: Volatile publisher, but subscription is %s;",
          sub_durability_str);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    }

    // Liveliness warnings (活跃度警告)
    if (pub_liveliness_unknown && sub_liveliness_unknown) {
      // Liveliness for publisher and subscription is unknown (发布者和订阅者的活跃度未知)
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
          reason, reason_size,
          // 警告：发布者活跃度为 %s，订阅者活跃度为 %s；
          "WARNING: Publisher liveliness is %s and subscription liveliness is %s;",
          pub_liveliness_str, sub_liveliness_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else if (
        pub_liveliness_unknown &&  // NOLINT
        subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC) {
      // Unknown liveliness for publisher and manual by topic for subscription
      // (发布者活跃度未知且订阅者活跃度为按主题手动设置)
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
          reason, reason_size,
          "WARNING: Subscription's liveliness is manual by topic, but publisher's is %s;",  // 警告：订阅者活跃度为按主题手动设置，但发布者活跃度为
                                                                                            // %s；
          pub_liveliness_str);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    } else if (
        publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_AUTOMATIC &&  // NOLINT
        sub_liveliness_unknown) {
      // Automatic liveliness for publisher and unknown for subscription
      // (发布者活跃度为自动且订阅者活跃度未知)
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
          reason, reason_size,
          "WARNING: Publisher's liveliness is automatic, but subscription's is %s;",  // 警告：发布者活跃度为自动，但订阅者活跃度为
                                                                                      // %s；
          sub_liveliness_str);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    }
  }

  return RMW_RET_OK;
}

/**
 * @brief 获取订阅端最佳可用的 QoS 配置 (Get the best available QoS profile for subscription)
 *
 * @param[in] publishers_info 发布者信息数组 (Array of publisher information)
 * @param[out] subscription_profile 订阅端 QoS 配置 (QoS profile for subscription)
 * @return rmw_ret_t 返回操作结果 (Return operation result)
 */
rmw_ret_t qos_profile_get_best_available_for_subscription(
    const rmw_topic_endpoint_info_array_t* publishers_info,
    rmw_qos_profile_t* subscription_profile) {
  // 检查输入参数是否为空 (Check if input parameters are null)
  if (!publishers_info) {
    RMW_SET_ERROR_MSG("publishers_info parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (!subscription_profile) {
    RMW_SET_ERROR_MSG("subscription_profile parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 只有所有发布者配置均为 reliable 时，才使用 "reliable" 可靠性
  // (Only use "reliable" reliability if all publisher profiles are reliable)
  // 只有所有发布者配置均为 transient local 时，才使用 "transient local" 持久化
  // (Only use "transient local" durability if all publisher profiles are transient local)
  // 只有所有发布者配置均为 manual by topic 时，才使用 "manual by topic" 活跃度
  // (Only use "manual by topic" liveliness if all publisher profiles are manual by topic)
  // 如果所有发布者都有默认截止日期，则使用默认截止日期，否则使用最大截止日期
  // (Use default deadline if all publishers have default deadline, otherwise use largest deadline)
  // 如果所有发布者都有默认租约，则使用默认租约，否则使用最大租约
  // (Use default lease duration if all publishers have default lease, otherwise use largest lease)
  size_t number_of_reliable = 0u;
  size_t number_of_transient_local = 0u;
  size_t number_of_manual_by_topic = 0u;
  bool use_default_deadline = true;
  rmw_time_t largest_deadline = {0u, 0u};
  bool use_default_liveliness_lease_duration = true;
  rmw_time_t largest_liveliness_lease_duration = {0u, 0u};

  // 遍历发布者信息数组，统计各个 QoS 策略的数量
  // (Iterate through the publisher info array, counting the number of each QoS policy)
  for (size_t i = 0u; i < publishers_info->size; ++i) {
    const rmw_qos_profile_t& profile = publishers_info->info_array[i].qos_profile;
    if (RMW_QOS_POLICY_RELIABILITY_RELIABLE == profile.reliability) {
      number_of_reliable++;
    }
    if (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL == profile.durability) {
      number_of_transient_local++;
    }
    if (RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC == profile.liveliness) {
      number_of_manual_by_topic++;
    }
    if (profile.deadline != deadline_default) {
      use_default_deadline = false;
      if (largest_deadline < profile.deadline) {
        largest_deadline = profile.deadline;
      }
    }
    if (profile.liveliness_lease_duration != liveliness_lease_duration_default) {
      use_default_liveliness_lease_duration = false;
      if (largest_liveliness_lease_duration < profile.liveliness_lease_duration) {
        largest_liveliness_lease_duration = profile.liveliness_lease_duration;
      }
    }
  }

  // 根据统计结果设置订阅端 QoS 配置
  // (Set subscription QoS profile based on the statistics)
  if (RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE == subscription_profile->reliability) {
    if (number_of_reliable == publishers_info->size) {
      subscription_profile->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    } else {
      subscription_profile->reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    }
  }

  if (RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE == subscription_profile->durability) {
    if (number_of_transient_local == publishers_info->size) {
      subscription_profile->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    } else {
      subscription_profile->durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    }
  }

  if (RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE == subscription_profile->liveliness) {
    if (number_of_manual_by_topic == publishers_info->size) {
      subscription_profile->liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    } else {
      subscription_profile->liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    }
  }

  if (deadline_best_available == subscription_profile->deadline) {
    if (use_default_deadline) {
      subscription_profile->deadline = RMW_QOS_DEADLINE_DEFAULT;
    } else {
      subscription_profile->deadline = largest_deadline;
    }
  }

  if (liveliness_lease_duration_best_available == subscription_profile->liveliness_lease_duration) {
    if (use_default_liveliness_lease_duration) {
      subscription_profile->liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    } else {
      subscription_profile->liveliness_lease_duration = largest_liveliness_lease_duration;
    }
  }

  // 返回操作成功
  // (Return operation success)
  return RMW_RET_OK;
}

/**
 * @brief 获取最佳可用的发布者 QoS 配置文件
 *        Get the best available QoS profile for a publisher
 *
 * @param[in] subscriptions_info 订阅者信息数组
 *                   Subscription information array
 * @param[out] publisher_profile 发布者 QoS 配置文件
 *                    Publisher QoS profile
 * @return rmw_ret_t 返回状态，成功或错误代码
 *                   Return status, success or error code
 */
rmw_ret_t qos_profile_get_best_available_for_publisher(
    const rmw_topic_endpoint_info_array_t* subscriptions_info,
    rmw_qos_profile_t* publisher_profile) {
  // 检查传入的订阅者信息参数是否为空
  // Check if the passed subscriptions_info parameter is null
  if (!subscriptions_info) {
    RMW_SET_ERROR_MSG("subscriptions_info parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  // 检查传入的发布者配置文件参数是否为空
  // Check if the passed publisher_profile parameter is null
  if (!publisher_profile) {
    RMW_SET_ERROR_MSG("publisher_profile parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 始终使用 "reliable" 可靠性和 "transient_local"
  // 持久性，因为这两种策略都与所有订阅兼容，并具有最高的服务级别 Always use "reliable" reliability
  // and "transient_local" durability since both policies are compatible with all subscriptions and
  // have the highest level of service
  if (RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE == publisher_profile->reliability) {
    publisher_profile->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if (RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE == publisher_profile->durability) {
    publisher_profile->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  }

  // 只有当至少有一个订阅者使用 "manual by topic" 活跃性时才使用它
  // 使用默认的 deadline，如果所有订阅者都有默认的 deadline，否则使用最小的
  // 使用默认的 lease duration，如果所有订阅者都有默认的 lease，否则使用最小的
  // Only use "manual by topic" liveliness if at least one  subscription is using manual by topic
  // Use default deadline if all subscriptions have default deadline, otherwise use smallest
  // Use default lease duration if all subscriptions have default lease, otherwise use smallest
  bool use_manual_by_topic = false;
  bool use_default_deadline = true;
  rmw_time_t smallest_deadline = RMW_DURATION_INFINITE;
  bool use_default_liveliness_lease_duration = true;
  rmw_time_t smallest_liveliness_lease_duration = RMW_DURATION_INFINITE;
  for (size_t i = 0u; i < subscriptions_info->size; ++i) {
    const rmw_qos_profile_t& profile = subscriptions_info->info_array[i].qos_profile;
    if (RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC == profile.liveliness) {
      use_manual_by_topic = true;
    }
    if (profile.deadline != deadline_default) {
      use_default_deadline = false;
      if (profile.deadline < smallest_deadline) {
        smallest_deadline = profile.deadline;
      }
    }
    if (profile.liveliness_lease_duration != liveliness_lease_duration_default) {
      use_default_liveliness_lease_duration = false;
      if (profile.liveliness_lease_duration < smallest_liveliness_lease_duration) {
        smallest_liveliness_lease_duration = profile.liveliness_lease_duration;
      }
    }
  }

  // 如果发布者配置文件的活跃性为 "best available"，则根据是否使用 "manual by topic" 设置活跃性
  // If the publisher_profile's liveliness is "best available", set the liveliness based on whether
  // "manual by topic" is used
  if (RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE == publisher_profile->liveliness) {
    if (use_manual_by_topic) {
      publisher_profile->liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    } else {
      publisher_profile->liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    }
  }

  // 如果发布者配置文件的 deadline 为 "best available"，则根据是否使用默认 deadline 设置 deadline
  // If the publisher_profile's deadline is "best available", set the deadline based on whether the
  // default deadline is used
  if (deadline_best_available == publisher_profile->deadline) {
    if (use_default_deadline) {
      publisher_profile->deadline = RMW_QOS_DEADLINE_DEFAULT;
    } else {
      publisher_profile->deadline = smallest_deadline;
    }
  }

  // 如果发布者配置文件的 liveliness_lease_duration 为 "best available"，则根据是否使用默认 lease
  // duration 设置 liveliness_lease_duration If the publisher_profile's liveliness_lease_duration is
  // "best available", set the liveliness_lease_duration based on whether the default lease duration
  // is used
  if (liveliness_lease_duration_best_available == publisher_profile->liveliness_lease_duration) {
    if (use_default_liveliness_lease_duration) {
      publisher_profile->liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    } else {
      publisher_profile->liveliness_lease_duration = smallest_liveliness_lease_duration;
    }
  }

  // 返回成功状态
  // Return success status
  return RMW_RET_OK;
}

/**
 * @brief 检查给定的QoS配置是否包含最佳可用策略 (Check if the given QoS profile has the best
 * available policy)
 *
 * @param[in] qos_profile 要检查的QoS配置 (The QoS profile to check)
 * @return 如果QoS配置包含最佳可用策略，则返回true，否则返回false (Returns true if the QoS profile
 * has the best available policy, false otherwise)
 */
static bool _qos_profile_has_best_available_policy(const rmw_qos_profile_t& qos_profile) {
  // 检查可靠性策略是否为最佳可用策略 (Check if the reliability policy is the best available policy)
  if (RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE == qos_profile.reliability) {
    return true;
  }
  // 检查持久性策略是否为最佳可用策略 (Check if the durability policy is the best available policy)
  if (RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE == qos_profile.durability) {
    return true;
  }
  // 检查活跃性策略是否为最佳可用策略 (Check if the liveliness policy is the best available policy)
  if (RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE == qos_profile.liveliness) {
    return true;
  }
  // 检查截止日期策略是否为最佳可用策略 (Check if the deadline policy is the best available policy)
  if (deadline_best_available == qos_profile.deadline) {
    return true;
  }
  // 检查活跃性租期持续时间策略是否为最佳可用策略 (Check if the liveliness lease duration policy is
  // the best available policy)
  if (liveliness_lease_duration_best_available == qos_profile.liveliness_lease_duration) {
    return true;
  }
  return false;
}

/**
 * @brief 获取订阅主题的最佳可用QoS配置 (Get the best available QoS profile for topic subscription)
 *
 * @param[in] node ROS2节点 (The ROS2 node)
 * @param[in] topic_name 主题名称 (The topic name)
 * @param[out] qos_profile 存储最佳可用QoS配置的指针 (Pointer to store the best available QoS
 * profile)
 * @param[in] get_endpoint_info 根据主题获取端点信息的函数 (Function to get endpoint info by topic)
 * @return 成功时返回RMW_RET_OK，否则返回相应的错误代码 (Returns RMW_RET_OK on success, appropriate
 * error code otherwise)
 */
rmw_ret_t qos_profile_get_best_available_for_topic_subscription(
    const rmw_node_t* node,
    const char* topic_name,
    rmw_qos_profile_t* qos_profile,
    const GetEndpointInfoByTopicFunction& get_endpoint_info) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, RMW_RET_INVALID_ARGUMENT);

  // 如果给定的QoS配置包含最佳可用策略，则执行以下操作
  // (If the given QoS profile has the best available policy, perform the following actions)
  if (_qos_profile_has_best_available_policy(*qos_profile)) {
    // 获取分配器 (Get the allocator)
    rcutils_allocator_t& allocator = node->context->options.allocator;
    // 初始化发布者信息数组 (Initialize the publishers info array)
    rmw_topic_endpoint_info_array_t publishers_info =
        rmw_get_zero_initialized_topic_endpoint_info_array();
    // 获取发布者端点信息 (Get the publisher endpoint info)
    rmw_ret_t ret = get_endpoint_info(node, &allocator, topic_name, false, &publishers_info);
    if (RMW_RET_OK != ret) {
      return ret;
    }
    // 获取订阅的最佳可用QoS配置 (Get the best available QoS profile for subscription)
    ret = qos_profile_get_best_available_for_subscription(&publishers_info, qos_profile);
    // 清理发布者信息数组 (Clean up the publishers info array)
    rmw_ret_t fini_ret = rmw_topic_endpoint_info_array_fini(&publishers_info, &allocator);
    if (RMW_RET_OK != fini_ret) {
      return fini_ret;
    }
    if (RMW_RET_OK != ret) {
      return ret;
    }
  }
  return RMW_RET_OK;
}

/**
 * @brief 获取发布主题的最佳可用QoS配置 (Get the best available QoS profile for topic publisher)
 *
 * @param[in] node ROS2节点 (The ROS2 node)
 * @param[in] topic_name 主题名称 (The topic name)
 * @param[out] qos_profile 存储最佳可用QoS配置的指针 (Pointer to store the best available QoS
 * profile)
 * @param[in] get_endpoint_info 根据主题获取端点信息的函数 (Function to get endpoint info by topic)
 * @return 成功时返回RMW_RET_OK，否则返回相应的错误代码 (Returns RMW_RET_OK on success, appropriate
 * error code otherwise)
 */
rmw_ret_t qos_profile_get_best_available_for_topic_publisher(
    const rmw_node_t* node,
    const char* topic_name,
    rmw_qos_profile_t* qos_profile,
    const GetEndpointInfoByTopicFunction& get_endpoint_info) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, RMW_RET_INVALID_ARGUMENT);

  // 如果给定的QoS配置包含最佳可用策略，则执行以下操作 (If the given QoS profile has the best
  // available policy, perform the following actions)
  if (_qos_profile_has_best_available_policy(*qos_profile)) {
    // 获取分配器 (Get the allocator)
    rcutils_allocator_t& allocator = node->context->options.allocator;
    // 初始化订阅者信息数组 (Initialize the subscriptions info array)
    rmw_topic_endpoint_info_array_t subscriptions_info =
        rmw_get_zero_initialized_topic_endpoint_info_array();
    // 获取订阅者端点信息 (Get the subscriber endpoint info)
    rmw_ret_t ret = get_endpoint_info(node, &allocator, topic_name, false, &subscriptions_info);
    if (RMW_RET_OK != ret) {
      return ret;
    }
    // 获取发布的最佳可用QoS配置 (Get the best available QoS profile for publisher)
    ret = qos_profile_get_best_available_for_publisher(&subscriptions_info, qos_profile);
    // 清理订阅者信息数组 (Clean up the subscriptions info array)
    rmw_ret_t fini_ret = rmw_topic_endpoint_info_array_fini(&subscriptions_info, &allocator);
    if (RMW_RET_OK != fini_ret) {
      return fini_ret;
    }
    if (RMW_RET_OK != ret) {
      return ret;
    }
  }
  return RMW_RET_OK;
}

/**
 * @brief 更新服务的最佳可用QoS配置 (Update the best available QoS profile for services)
 *
 * @param[in] qos_profile 要更新的QoS配置 (The QoS profile to update)
 * @return 更新后的QoS配置 (The updated QoS profile)
 */
rmw_qos_profile_t qos_profile_update_best_available_for_services(
    const rmw_qos_profile_t& qos_profile) {
  // 创建一个结果变量并将其设置为输入的QoS配置 (Create a result variable and set it to the input QoS
  // profile)
  rmw_qos_profile_t result = qos_profile;
  // 如果可靠性策略是最佳可用策略，则使用服务默认配置更新它 (If the reliability policy is the best
  // available policy, update it with the service default configuration)
  if (RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE == result.reliability) {
    result.reliability = rmw_qos_profile_services_default.reliability;
  }
  // 如果持久性策略是最佳可用策略，则使用服务默认配置更新它 (If the durability policy is the best
  // available policy, update it with the service default configuration)
  if (RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE == result.durability) {
    result.durability = rmw_qos_profile_services_default.durability;
  }
  // 如果活跃性策略是最佳可用策略，则使用服务默认配置更新它 (If the liveliness policy is the best
  // available policy, update it with the service default configuration)
  if (RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE == result.liveliness) {
    result.liveliness = rmw_qos_profile_services_default.liveliness;
  }
  // 如果截止日期策略是最佳可用策略，则使用服务默认配置更新它 (If the deadline policy is the best
  // available policy, update it with the service default configuration)
  if (deadline_best_available == result.deadline) {
    result.deadline = rmw_qos_profile_services_default.deadline;
  }
  // 如果活跃性租期持续时间策略是最佳可用策略，则使用服务默认配置更新它 (If the liveliness lease
  // duration policy is
  if (liveliness_lease_duration_best_available == result.liveliness_lease_duration) {
    result.liveliness_lease_duration = rmw_qos_profile_services_default.liveliness_lease_duration;
  }
  return result;
}

/**
 * @brief 解析用户数据中的类型哈希值 (Parse type hash from user data)
 *
 * @param[in] user_data 用户数据指针 (Pointer to user data)
 * @param[in] user_data_size 用户数据大小 (Size of user data)
 * @param[out] type_hash_out 解析出的类型哈希值 (Parsed type hash)
 * @return rmw_ret_t 返回操作状态 (Return operation status)
 */
rmw_ret_t parse_type_hash_from_user_data(
    const uint8_t* user_data, size_t user_data_size, rosidl_type_hash_t& type_hash_out) {
  // 检查 user_data 是否为空 (Check if user_data is NULL)
  RMW_CHECK_ARGUMENT_FOR_NULL(user_data, RMW_RET_INVALID_ARGUMENT);

  // 将 user_data 转换为 std::vector (Convert user_data to std::vector)
  std::vector<uint8_t> udvec(user_data, user_data + user_data_size);

  // 解析键值对 (Parse key-value pairs)
  auto key_value = rmw::impl::cpp::parse_key_value(udvec);

  // 查找 "typehash" 键 (Find the "typehash" key)
  auto typehash_it = key_value.find("typehash");

  // 如果未找到 "typehash" 键，返回零初始化的类型哈希值 (If "typehash" key not found, return
  // zero-initialized type hash)
  if (typehash_it == key_value.end()) {
    type_hash_out = rosidl_get_zero_initialized_type_hash();
    return RMW_RET_OK;
  }

  // 获取类型哈希字符串 (Get the type hash string)
  std::string type_hash_str(typehash_it->second.begin(), typehash_it->second.end());

  // 解析类型哈希字符串 (Parse the type hash string)
  if (RCUTILS_RET_OK != rosidl_parse_type_hash_string(type_hash_str.c_str(), &type_hash_out)) {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

/**
 * @brief 为用户数据的 QoS 编码类型哈希值 (Encode type hash for user data QoS)
 *
 * @param[in] type_hash 类型哈希值 (Type hash)
 * @param[out] string_out 编码后的类型哈希字符串 (Encoded type hash string)
 * @return rmw_ret_t 返回操作状态 (Return operation status)
 */
rmw_ret_t encode_type_hash_for_user_data_qos(
    const rosidl_type_hash_t& type_hash, std::string& string_out) {
  // 如果类型哈希版本未设置，清除输出字符串并返回成功 (If type hash version is unset, clear output
  // string and return success)
  if (type_hash.version == ROSIDL_TYPE_HASH_VERSION_UNSET) {
    string_out.clear();
    return RMW_RET_OK;
  }

  // 获取默认分配器 (Get default allocator)
  auto allocator = rcutils_get_default_allocator();

  // 分配类型哈希 C 字符串 (Allocate type hash C string)
  char* type_hash_c_str = nullptr;

  // 将类型哈希转换为字符串 (Convert type hash to string)
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(&type_hash, allocator, &type_hash_c_str);

  // 检查内存分配错误 (Check for memory allocation error)
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    return RMW_RET_BAD_ALLOC;
  }

  // 检查其他错误 (Check for other errors)
  if (RCUTILS_RET_OK != stringify_ret) {
    return RMW_RET_ERROR;
  }

  // 释放分配的内存 (Release allocated memory)
  RCPPUTILS_SCOPE_EXIT(allocator.deallocate(type_hash_c_str, &allocator.state));

  // 设置输出字符串 (Set output string)
  string_out = "typehash=" + std::string(type_hash_c_str) + ";";

  return RMW_RET_OK;
}

}  // namespace rmw_dds_common
