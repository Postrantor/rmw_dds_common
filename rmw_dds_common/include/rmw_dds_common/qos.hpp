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

#ifndef RMW_DDS_COMMON__QOS_HPP_
#define RMW_DDS_COMMON__QOS_HPP_

#include <functional>
#include <string>

#include "rmw/qos_profiles.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw/types.h"
#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common {

/// 检查两个 QoS 配置文件是否兼容(Check if two QoS profiles are compatible)
/**
 * 如果使用这些 QoS 策略的发布者和订阅者可以相互通信，则两个 QoS 配置文件是兼容的。
 * (Two QoS profiles are compatible if a publisher and subscription
 * using the QoS policies can communicate with each other.)
 *
 * 此函数实现了 rmw API \ref rmw_qos_profile_check_compatible()。
 * 有关更多信息，请参阅 \ref rmw_qos_profile_check_compatible()。
 * (This implements the rmw API \ref rmw_qos_profile_check_compatible().)
 * (See \ref rmw_qos_profile_check_compatible() for more information.)
 *
 * \param[in] publisher_qos: 用于发布者的 QoS 配置文件。(The QoS profile used for a publisher.)
 * \param[in] subscription_qos: 用于订阅者的 QoS 配置文件。(The QoS profile used for a
 *subscription.)
 * \param[out] compatibility: 如果 QoS 配置文件兼容，则为 `RMW_QOS_COMPATIBILITY_OK`，或
 *   如果 QoS 配置文件可能兼容，则为 `RMW_QOS_COMPATIBILITY_WARNING`，或
 *   如果 QoS 配置文件不兼容，则为 `RMW_QOS_COMPATIBILITY_ERROR`。
 *   (`RMW_QOS_COMPATIBILITY_OK` if the QoS profiles are compatible, or)
 *   (`RMW_QOS_COMPATIBILITY_WARNING` if the QoS profiles might be compatible, or)
 *   (`RMW_QOS_COMPATIBILITY_ERROR` if the QoS profiles are not compatible.)
 * \param[out] reason: QoS 不兼容或可能不兼容的详细原因。
 *   必须由调用者预先分配。
 *   如果不需要原因信息，此参数是可选的，可以将其设置为 `nullptr`。
 *   (A detailed reason for a QoS incompatibility or potential incompatibility.)
 *   (Must be pre-allocated by the caller.)
 *   (This parameter is optional and may be set to `nullptr` if the reason information is not
 *   desired.)
 * \param[in] reason_size: 如果提供了字符串缓冲区 `reason`，则为 `reason` 的大小。
 *   如果 `reason` 为 `nullptr`，则此参数必须为零。
 *   (Size of the string buffer `reason`, if one is provided.)
 *   (If `reason` is `nullptr`, then this parameter must be zero.)
 * \return 如果检查成功，则返回 `RMW_RET_OK`，或
 * \return 如果 `compatiblity` 为 `nullptr`，则返回 `RMW_RET_INVALID_ARGUMENT`，或
 * \return 如果 `reason` 为 `nullptr` 且 `reason_size` 不为零，则返回 `RMW_RET_INVALID_ARGUMENT`，或
 * \return 如果出现意外错误，则返回 `RMW_RET_ERROR`。
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t qos_profile_check_compatible(
    const rmw_qos_profile_t publisher_qos,
    const rmw_qos_profile_t subscription_qos,
    rmw_qos_compatibility_type_t *compatibility,
    char *reason,
    size_t reason_size);

/// 获取订阅的最佳可用 QoS 策略。
/// Get the best available QoS policies for a subscription.
/**
 * 根据零个或多个发布者端点，更新订阅 QoS 配置文件中的任何 BEST_AVAILABLE 策略，
 * 以便在保持最高服务水平的同时匹配所有发布者。
 * Given zero or more publisher endpoints, update any BEST_AVAILABLE policies in a subscription QoS
 * profile such that is matches all publishers while maintaining the highest level of service.
 *
 * 如果可靠性是 BEST_AVAILABLE，则当所有发布者都是 RELIABLE 时，将其设置为 RELIABLE。
 * If reliability is BEST_AVAILABLE, then it will be set to RELIABLE if all publishers are
 * RELIABLE.
 * 否则，可靠性将设置为 BEST_EFFORT。
 * Otherwise, reliability will be set to BEST_EFFORT.
 *
 * 如果持久性是 BEST_AVAILABLE，则当所有发布者都是 TRANSIENT_LOCAL 时，将其设置为 TRANSIENT_LOCAL。
 * If durability is BEST_AVAILABLE, then it will be set to TRANSIENT_LOCAL if all publishers are
 * TRANSIENT_LOCAL.
 * 否则，持久性将设置为 VOLATILE。
 * Otherwise, durability will be set to VOLATILE.
 *
 * 如果生命周期是 BEST_AVAILABLE，则当所有发布者都是 MANUAL_BY_TOPIC 时，将其设置为
 * MANUAL_BY_TOPIC。 If liveliness is BEST_AVAILABLE, then it will be set to MANUAL_BY_TOPIC if all
 * publishers are MANUAL_BY_TOPIC. 否则，生命周期将设置为 AUTOMATIC。 Otherwise, liveliness will be
 * set to AUTOMATIC.
 *
 * 如果截止日期是 BEST_AVAILABLE，则当所有发布者都是 DEFAULT 时，将其设置为 DEFAULT。
 * If deadline is BEST_AVAILABLE, then it will be set to DEFAULT if all publishers are DEFAULT.
 * 否则，截止日期将设置为所有发布者的最大截止日期。
 * Otherwise, deadline will be set to the maximum deadline of all publishers.
 *
 * 如果生命周期租期是 BEST_AVAILABLE，则当所有发布者都是 DEFAULT 时，将其设置为 DEFAULT。
 * If liveliness lease duration is BEST_AVAILABLE, then it will be set to DEFAULT if all
 * publishers are DEFAULT.
 * 否则，生命周期租期将设置为所有发布者的最大截止日期。
 * Otherwise, liveliness lease duration will be set to the maximum deadline of all publishers.
 *
 * 历史记录、历史深度和生命周期策略不会被此函数更改。
 * History, history depth, and lifespan policies are not changed by this function.
 *
 * \param[in] publishers_info: 发布者端点信息。
 * \param[in] publishers_info: Endpoint information for publishers.
 * \param[out] subscription_profile: 与大多数输入发布者兼容的 QoS 配置文件。
 * \param[out] subscription_profile: QoS profile that is compatible with the majority of
 *   the input publishers.
 * \return `RMW_RET_OK` 如果操作成功，或
 * \return `RMW_RET_OK` if the operation was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `publishers_info` 是 `nullptr`，或
 * \return `RMW_RET_INVALID_ARGUMENT` if `publishers_info` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `subscription_profile` 是 `nullptr`，或
 * \return `RMW_RET_INVALID_ARGUMENT` if `subscription_profile` is `nullptr`, or
 * \return `RMW_RET_ERROR` 如果出现意外错误。
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t qos_profile_get_best_available_for_subscription(
    const rmw_topic_endpoint_info_array_t *publishers_info,
    rmw_qos_profile_t *subscription_profile);

/// 获取发布者的最佳可用 QoS 策略。
/// Get the best available QoS policies for a publisher.
/**
 * 给定零个或多个订阅端点，更新发布者 QoS 配置文件中的任何 BEST_AVAILABLE 策略，
 * 以便在保持最高服务水平的同时匹配所有订阅。
 * Given zero or more subscription endpoints, update any BEST_AVAILABLE policies in a publisher QoS
 * profile such that is matches all subscriptions while maintaining the highest level of service.
 *
 * 如果可靠性是 BEST_AVAILABLE，则将其设置为 RELIABLE。
 * If reliability is BEST_AVAILABLE, then it will be set to RELIABLE.
 *
 * 如果持久性是 BEST_AVAILABLE，则将其设置为 TRANSIENT_LOCAL。
 * If durability is BEST_AVAILABLE, then it will be set to TRANSIENT_LOCAL.
 *
 * 如果活跃度是 BEST_AVAILABLE，那么如果至少有一个订阅是 MANUAL_BY_TOPIC，则将其设置为
 * MANUAL_BY_TOPIC。 否则，活跃度将设置为 AUTOMATIC。 If liveliness is BEST_AVAILABLE, then it will
 * be set to MANUAL_BY_TOPIC if at least one subscription is MANUAL_BY_TOPIC. Otherwise, liveliness
 * will be set to AUTOMATIC.
 *
 * 如果截止日期是 BEST_AVAILABLE，那么如果所有订阅都是 DEFAULT，则将其设置为 DEFAULT。
 * 否则，截止日期将设置为所有订阅的最小截止日期。
 * If deadline is BEST_AVAILABLE, then it will be set to DEFAULT if all subscriptions are DEFAULT.
 * Otherwise, deadline will be set to the minimum deadline of all subscriptions.
 *
 * 如果活跃度租期持续时间是 BEST_AVAILABLE，那么如果所有订阅都是 DEFAULT，则将其设置为 DEFAULT。
 * 否则，活跃度租期持续时间将设置为所有订阅的最小截止日期。
 * If liveliness lease duration is BEST_AVAILABLE, then it will be set to DEFAULT if all
 * subscriptions are DEFAULT.
 * Otherwise, liveliness lease duration will be set to the mininum deadline of all subscriptions.
 *
 * 此功能不会更改历史记录、历史记录深度和生命周期策略。
 * History, history depth, and lifespan policies are not changed by this function.
 *
 * \param[in] subscriptions_info: 订阅端点信息。
 * \param[in] subscriptions_info: Endpoint information for subscriptions.
 * \param[out] publisher_profile: 与大多数输入订阅兼容的 QoS 配置文件。
 * \param[out] publisher_profile: QoS profile that is compatible with the majority of
 *   the input subscriptions.
 * \return `RMW_RET_OK` 如果操作成功，或
 * \return `RMW_RET_OK` if the operation was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `subscriptions_info` 是 `nullptr`，或
 * \return `RMW_RET_INVALID_ARGUMENT` if `subscriptions_info` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `publisher_profile` 是 `nullptr`，或
 * \return `RMW_RET_INVALID_ARGUMENT` if `publisher_profile` is `nullptr`, or
 * \return `RMW_RET_ERROR` 如果有意外错误。
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t qos_profile_get_best_available_for_publisher(
    const rmw_topic_endpoint_info_array_t *subscriptions_info,
    rmw_qos_profile_t *publisher_profile);

/// 签名匹配 rmw_get_publishers_info_by_topic 和 rmw_get_subscriptions_info_by_topic
/// Signature matching rmw_get_publishers_info_by_topic and rmw_get_subscriptions_info_by_topic
using GetEndpointInfoByTopicFunction = std::function<rmw_ret_t(
    const rmw_node_t *,
    rcutils_allocator_t *,
    const char *,
    bool,
    rmw_topic_endpoint_info_array_t *)>;

/// 更新订阅的 QoS 配置文件，使其与发现的发布者兼容。
/// Update a subscription QoS profile so that it is compatible with discovered publishers.
/**
 * 如果 `qos_profile` 中的任何策略设置为 BEST_AVAILABLE，则将调用 `get_endpoint_info`
 * 来查询用于适应 QoS 策略的端点信息。
 * If any policies in `qos_profile` are set to BEST_AVAILABLE, then `get_endpoint_info` will
 * be called to query endpoint information for adapting the QoS policies.
 *
 * 有关适应 'best available' 策略的规则，请参见
 * \ref `qos_profile_get_best_available_for_subscription`。
 * For rules related to adapting 'best available' policies, see
 * \ref `qos_profile_get_best_available_for_subscription`.
 *
 * 此函数使用节点的上下文分配器分配内存。
 * This function allocates memory with the node's context allocator.
 *
 * \param[in] node: 用于查询图形的节点。
 * \param[in] node: Node used to query the graph.
 * \param[in] topic_name: 查询此主题名称上的发布者的信息。
 * \param[in] topic_name: Query info for publishers on this topic name.
 * \param[inout] qos_profile: 设置为 'best available' 的任何策略将根据发布者端点 QoS 策略进行更新。
 * \param[inout] qos_profile: Any policies that are set to 'best available' will be updated based
 *   on publisher endpoint QoS policies.
 * \param[in] get_endpoint_info: 用于查询发布者端点信息的函数。
 *   即 `rmw_get_publishers_info_by_topic` 的实现。
 * \param[in] get_endpoint_info: The function used to query for publisher endpoint information.
 *   I.e. an implementation of `rmw_get_publishers_info_by_topic`.
 * \return `RMW_RET_OK` 如果操作成功，或
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `node` 是 `nullptr`，或
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `topic_name` 是 `nullptr`，或
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `qos_profile` 是 `nullptr`，或
 * \return `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` 如果 `node` 实现
 *   标识符与此实现不匹配，或
 * \return `RMW_RET_BAD_ALLOC` 如果内存分配失败，或
 * \return `RMW_RET_ERROR` 如果有意外错误。
 * \return `RMW_RET_OK` if the operation was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `node` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `topic_name` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `qos_profile` is `nullptr`, or
 * \return `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` if the `node` implementation
 *   identifier does not match this implementation, or
 * \return `RMW_RET_BAD_ALLOC` if memory allocation fails, or
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t qos_profile_get_best_available_for_topic_subscription(
    const rmw_node_t *node,
    const char *topic_name,
    rmw_qos_profile_t *qos_profile,
    const GetEndpointInfoByTopicFunction &get_endpoint_info);

/// 更新发布者 QoS 配置文件，使其与已发现的订阅兼容。
/// Update a publisher QoS profile so that it is compatible with discovered subscriptions.
/**
 * 如果 `qos_profile` 中的任何策略设置为 'best available'，那么将调用 `get_endpoint_info`
 * 来查询端点信息以适应 QoS 策略。
 * If any policies in `qos_profile` are set to 'best available', then `get_endpoint_info` will
 * be called to query endpoint information for adapting the QoS policies.
 *
 * 有关适应 'best available' 策略的规则，请参见
 * \ref `qos_profile_get_best_available_for_publisher`。
 * For rules related to adapting 'best available' policies, see
 * \ref `qos_profile_get_best_available_for_publisher`.
 *
 * 此函数使用节点的上下文分配器分配内存。
 * This function allocates memory with the node's context allocator.
 *
 * \param[in] node: 用于查询图形的节点。
 * \param[in] node: Node used to query the graph.
 * \param[in] topic_name: 查询此主题名称的订阅信息。
 * \param[in] topic_name: Query info for subscriptions on this topic name.
 * \param[inout] qos_profile: 设置为 'best available' 的任何策略都将根据订阅端点 QoS 策略进行更新。
 * \param[inout] qos_profile: Any policies that are set to 'best available' will by updated based
 *   on subscription endpoint QoS policies.
 * \param[in] get_endpoint_info: 用于查询订阅端点信息的函数。
 *   即 `rmw_get_subscriptions_info_by_topic` 的实现。
 * \param[in] get_endpoint_info: The function used to query for subscription endpoint information.
 *   I.e. an implementation of `rmw_get_subscriptions_info_by_topic`.
 * \return `RMW_RET_OK` 如果操作成功，或
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `node` 是 `nullptr`，或
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `topic_name` 是 `nullptr`，或
 * \return `RMW_RET_INVALID_ARGUMENT` 如果 `qos_profile` 是 `nullptr`，或
 * \return `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` 如果 `node` 实现
 *   标识符与此实现不匹配，或
 * \return `RMW_RET_BAD_ALLOC` 如果内存分配失败，或
 * \return `RMW_RET_ERROR` 如果出现意外错误。
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t qos_profile_get_best_available_for_topic_publisher(
    const rmw_node_t *node,
    const char *topic_name,
    rmw_qos_profile_t *qos_profile,
    const GetEndpointInfoByTopicFunction &get_endpoint_info);

/// 更新服务和客户端的最佳可用 QoS 策略。
/// Update best available QoS policies for services and clients.
/**
 * 给定 QoS 策略，返回一个新的策略集，其中任何 BEST_AVAILABLE 策略都被替换为服务的默认策略。
 * Give QoS policies, return a new set of policies that have any BEST_AVAILABLE policies replaced
 * with default policies for services.
 *
 * 有关默认策略值，请参见 `rmw_qos_profile_services_default`。
 * See `rmw_qos_profile_services_default` for default policy values.
 *
 * \param[in] qos_profile: 要复制和更新的 QoS 配置文件。
 * \param[in] qos_profile: QoS profile to copy and update.
 * \return 输入 QoS 配置文件的副本，其中任何 BEST_AVAILABLE 策略都被覆盖为
 *   默认服务策略。
 * \return A copy of the input QoS profile with any BEST_AVAILABLE policies overwritten with
 *   default service policies.
 */
RMW_DDS_COMMON_PUBLIC
rmw_qos_profile_t qos_profile_update_best_available_for_services(
    const rmw_qos_profile_t &qos_profile);

/// 解析 USER_DATA "key=value;key=value;" 编码，找到 key "typehash" 的值
/// Parse USER_DATA "key=value;key=value;" encoding, finding value of key "typehash"
/**
 * \param[in] user_data 用户数据 QoS 的原始字节
 * \param[in] user_data_size user_data 的长度
 * \param[out] type_hash_out 如果找到则填充类型哈希数据，如果未找到键，则为零值
 * \return RMW_RET_OK 如果成功解析键，或者未找到键
 * \return RMW_RET_INVALID_ARGUMENT 如果 user_data 为空
 * \return RMW_RET_ERROR 如果找到 typehash 键，但无法解析值
 *
 * \param[in] user_data USER_DATA qos raw bytes
 * \param[in] user_data_size Length of user_data
 * \param[out] type_hash_out Filled with type hash data if found, or to zero value if key not found
 * \return RMW_RET_OK if key parsed successfully, or if key not found
 * \return RMW_RET_INVALID_ARGUMENT if user_data is null
 * \return RMW_RET_ERROR if typehash key found, but value could not be parsed
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t parse_type_hash_from_user_data(
    const uint8_t *user_data, size_t user_data_size, rosidl_type_hash_t &type_hash_out);

/// 将类型哈希编码为 "typehash=hash_string;" 以在 USER_DATA QoS 中使用
/// Encode type hash as "typehash=hash_string;" for use in USER_DATA QoS
/**
 * \param[in] type_hash 要编码的类型哈希值
 * \param[out] string_out 成功时，将设置为 "typehash=stringified_type_hash;"
 *   如果 type_hash 的版本为 0，则将 string_out 设置为空
 * \return RMW_RET_OK 成功时，包括未设置版本的空字符串
 * \return RMW_RET_BAD_ALLOC 如果内存分配失败
 *
 * \param[in] type_hash Type hash value to encode
 * \param[out] string_out On success, will be set to "typehash=stringified_type_hash;"
 *   If type_hash's version is 0, string_out will be set to empty
 * \return RMW_RET_OK on success, including empty string for unset version
 * \return RMW_RET_BAD_ALLOC if memory allocation fails
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t encode_type_hash_for_user_data_qos(
    const rosidl_type_hash_t &type_hash, std::string &string_out);

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__QOS_HPP_
