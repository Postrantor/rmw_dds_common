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

#ifndef RMW_DDS_COMMON__GRAPH_CACHE_HPP_
#define RMW_DDS_COMMON__GRAPH_CACHE_HPP_

#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rmw/names_and_types.h"
#include "rmw/topic_endpoint_info.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw/types.h"
#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/msg/gid.hpp"
#include "rmw_dds_common/msg/node_entities_info.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"
#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common {

// Forward-declaration, defined at end of file.
struct EntityInfo;
struct ParticipantInfo;

/// 图形缓存数据结构 (Graph cache data structure).
/**
 * 管理参与者、节点和主题之间的关系 (Manages relationships between participants, nodes, and topics).
 */
class GraphCache {
  // 声明一个友元函数，用于输出 GraphCache 的内容到输出流中 (Declare a friend function to output the
  // content of GraphCache to the output stream)
  friend RMW_DDS_COMMON_PUBLIC std::ostream &operator<<(
      std::ostream &ostream, const GraphCache &topic_cache);

public:
  /// 设置一个回调函数，在对象状态改变时会被调用 (Set a callback that will be called when the state
  /// of the object changes).
  /**
   * \param callback 要调用的回调函数 (callback to be called).
   */
  template <typename CallbackT>
  void set_on_change_callback(CallbackT &&callback) {
    // 使用互斥锁保护对 on_change_callback_ 的访问 (Use a mutex lock to protect access to
    // on_change_callback_)
    std::lock_guard<std::mutex> lock(mutex_);
    // 将传入的回调函数赋值给 on_change_callback_ (Assign the passed-in callback function to
    // on_change_callback_)
    on_change_callback_ = callback;
  }

  /// 清除之前注册的 "on change" 回调。
  /// Clear previously registered "on change" callback.
  RMW_DDS_COMMON_PUBLIC
  void clear_on_change_callback();

  /**
   * \defgroup dds_discovery_api dds_discovery_api
   * Methods used to update the Graph Cache based on DDS discovery.
   * @{
   */

  /// 基于发现添加数据写入器。
  /// Add a data writer based on discovery.
  /**
   * \param writer_gid 数据写入器的 GUID。
   * \param topic_name 此数据写入器的 DDS 主题名称。
   * \param type_name 此数据写入器的 DDS 主题类型名称。
   * \param type_hash 主题类型描述的哈希值。
   * \param participant_gid 参与者的 GUID。
   * \param qos 数据写入器的 QoS 配置文件。
   * \return 如果缓存已更新，则返回 `true`，如果数据写入器已存在，则返回 `false`。
   */
  RMW_DDS_COMMON_PUBLIC
  bool add_writer(
      const rmw_gid_t &writer_gid,
      const std::string &topic_name,
      const std::string &type_name,
      const rosidl_type_hash_t &type_hash,
      const rmw_gid_t &participant_gid,
      const rmw_qos_profile_t &qos);

  /// 基于发现添加数据写入器。
  /// Add a data writer based on discovery.
  /**
   * 参见带有 rosidl_type_hash_t 的 add_reader，其他参数与这些相匹配。
   * See add_reader with rosidl_type_hash_t, whose other parameters match these.
   */
  RMW_DDS_COMMON_PUBLIC
  RCUTILS_DEPRECATED_WITH_MSG("Migrate to using the version of this function taking a type hash.")
  bool add_writer(
      const rmw_gid_t &writer_gid,
      const std::string &topic_name,
      const std::string &type_name,
      const rmw_gid_t &participant_gid,
      const rmw_qos_profile_t &qos);

  /// 添加一个基于发现的数据读取器。
  /**
   * \param reader_gid 数据读取器的GUID。
   * \param topic_name 此数据读取器的DDS主题名称。
   * \param type_name 此数据读取器的DDS主题类型名称。
   * \param type_hash 主题类型描述的哈希值。
   * \param participant_gid 参与者的GUID。
   * \param qos 数据读取器的QoS配置文件。
   * \return 如果缓存已更新，则为`true`；如果数据读取器已存在，则为`false`。
   */
  RMW_DDS_COMMON_PUBLIC
  bool add_reader(
      const rmw_gid_t &reader_gid,
      const std::string &topic_name,
      const std::string &type_name,
      const rosidl_type_hash_t &type_hash,
      const rmw_gid_t &participant_gid,
      const rmw_qos_profile_t &qos);

  /// 添加一个基于发现的数据读取器。
  /**
   * 请参阅具有rosidl_type_hash_t的add_reader，其其他参数与这些匹配。
   */
  RMW_DDS_COMMON_PUBLIC
  RCUTILS_DEPRECATED_WITH_MSG("Migrate to using the version of this function taking a type hash.")
  bool add_reader(
      const rmw_gid_t &reader_gid,
      const std::string &topic_name,
      const std::string &type_name,
      const rmw_gid_t &participant_gid,
      const rmw_qos_profile_t &qos);

  /// 添加一个数据读取器或写入器。
  /**
   * \param gid 实体的GUID。
   * \param topic_name 此数据读取器的DDS主题名称。
   * \param type_name 此实体的DDS主题类型名称。
   * \param type_hash 主题类型描述的哈希值。
   * \param participant_gid 参与者的GUID。
   * \param qos 实体的QoS配置文件。
   * \param is_reader 实体是数据读取器还是写入器。
   * \return 如果缓存已更新，则为`true`；如果实体已存在，则为`false`。
   */
  RMW_DDS_COMMON_PUBLIC
  bool add_entity(
      const rmw_gid_t &gid,
      const std::string &topic_name,
      const std::string &type_name,
      const rosidl_type_hash_t &type_hash,
      const rmw_gid_t &participant_gid,
      const rmw_qos_profile_t &qos,
      bool is_reader);

  /// 添加数据读取器或写入器。 (Add a data reader or writer.)
  /**
   * 参见带有rosidl_type_hash_t的add_entity，其它参数与这些相匹配。
   * (See add_entity with rosidl_type_hash_t, whose other parameters match these.)
   */
  RMW_DDS_COMMON_PUBLIC
  RCUTILS_DEPRECATED_WITH_MSG("Migrate to using the version of this function taking a type hash.")
  bool add_entity(
      const rmw_gid_t &gid,
      const std::string &topic_name,
      const std::string &type_name,
      const rmw_gid_t &participant_gid,
      const rmw_qos_profile_t &qos,
      bool is_reader);

  /// 移除数据写入器。 (Remove a data writer.)
  /**
   * \param gid 数据写入器的GUID。 (GUID of the data writer.)
   * \return 如果缓存已更新，则为`true`；如果数据写入器不存在，则为`false`。 ( `true` if the cache
   * was updated, `false` if the data writer was not present.)
   */
  RMW_DDS_COMMON_PUBLIC
  bool remove_writer(const rmw_gid_t &gid);

  /// 移除数据读取器。 (Remove a data reader.)
  /**
   * \param gid 数据读取器的GUID。 (GUID of the The data reader.)
   * \return 如果缓存已更新，则为`true`；如果数据读取器不存在，则为`false`。 (`true` if the cache
   * was updated, `false` if the data reader was not present.)
   */
  RMW_DDS_COMMON_PUBLIC
  bool remove_reader(const rmw_gid_t &gid);

  /// 移除数据读取器或写入器。 (Remove a data reader or writer.)
  /**
   * \param gid 实体的GUID。 (GUID of the entity.)
   * \param is_reader 实体是数据读取器还是写入器。 (Whether the entity is a data reader or a
   * writer.) \return 当缓存更新时为`true`，如果实体不存在则为`false`。 (`true` when the cache was
   * updated, `false` if the entity was not present.)
   */
  RMW_DDS_COMMON_PUBLIC
  bool remove_entity(const rmw_gid_t &gid, bool is_reader);

  /**
   * @}
   * \defgroup common_api common_api
   * 用于更新图形缓存的方法。 (Methods used to update the Graph Cache.)
   * @{
   */

  /// 添加参与者。 (Add a participant.)
  /**
   * \param participant_gid 参与者的 GUID。 (GUID of the participant.)
   * \param enclave Enclave 的名称。 (Name of the enclave.)
   */
  RMW_DDS_COMMON_PUBLIC
  void add_participant(const rmw_gid_t &participant_gid, const std::string &enclave);

  /// 移除参与者。 (Remove a participant.)
  /**
   * \param participant_gid 参与者的 GUID。 (GUID of the participant.)
   * \return 当发生变化时返回 `true`。 (`true` when a change took place.)
   */
  RMW_DDS_COMMON_PUBLIC
  bool remove_participant(const rmw_gid_t &participant_gid);

  /**
   * @}
   * \defgroup ros_discovery_api ros_discovery_api
   * 用于根据从远程参与者接收到的 ROS 2 特定消息更新 Graph Cache 的方法。 (Methods used to update
   * the Graph Cache based on ROS 2 specific messages received from remote participants.)
   * @{
   */

  /// 从 `ParticipantEntitiesInfo` 消息更新缓存的参与者信息。 (Update cached participant info from a
  /// `ParticipantEntitiesInfo` message.)
  /**
   * \param msg 用于更新缓存的参与者信息。 (participant info to update cache from.)
   */
  RMW_DDS_COMMON_PUBLIC
  void update_participant_entities(const rmw_dds_common::msg::ParticipantEntitiesInfo &msg);

  /**
   * @}
   * \defgroup local_api local_api
   * 用于根据本地实体的构建和销毁更新 Graph Cache 的方法。这些方法都返回一个描述本地参与者状态的
   * `ParticipantEntitiesInfo` 消息。此消息可用于更新由远程参与者保留的远程缓存。请参阅
   * rmw_dds_common::GraphCache::update_participant_entities。 (Methods used to update the Graph
   * Cache, based on local construction and destruction of entities. All of these methods return a
   * `ParticipantEntitiesInfo` message describing the local participant state. This message can be
   * used to update remote caches kept by remote participants. \see
   * rmw_dds_common::GraphCache::update_participant_entities.)
   * @{
   */

  /// 添加一个节点到图中。
  /// Add a node to the graph.
  /**
   * \param participant_gid 参与者的 GUID。
   * \param node_name 要添加的节点的名称。
   * \param node_namespace 要添加的节点的命名空间。
   * \return 更新其他缓存的消息。
   *
   * \param participant_gid GUID of the participant.
   * \param node_name Name of the node to be added.
   * \param node_namespace Namespace of the node to be added.
   * \return Message to update other caches.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo add_node(
      const rmw_gid_t &participant_gid,
      const std::string &node_name,
      const std::string &node_namespace);

  /// 从图中移除一个节点。
  /// Remove a node from the graph.
  /**
   * \param participant_gid 参与者的 GUID。
   * \param node_name 要移除的节点的名称。
   * \param node_namespace 要移除的节点的命名空间。
   * \return 更新其他缓存的消息。
   *
   * \param participant_gid GUID of the participant.
   * \param node_name Name of the node to be removed.
   * \param node_namespace Namespace of the node to be removed.
   * \return Message to update other caches.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo remove_node(
      const rmw_gid_t &participant_gid,
      const std::string &node_name,
      const std::string &node_namespace);

  /// 将数据写入器与节点关联。
  /// Associate a data writer with a node.
  /**
   * \param writer_gid 数据写入器的 GUID。
   * \param participant_gid 参与者的 GUID。
   * \param node_name 目标节点的名称。
   * \param node_namespace 目标节点的命名空间。
   * \return 更新其他缓存的消息。
   *
   * \param writer_gid GUID of the data writer.
   * \param participant_gid GUID of the participant.
   * \param node_name Name of the target node.
   * \param node_namespace Namespace of the target node.
   * \return Message to update other caches.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo associate_writer(
      const rmw_gid_t &writer_gid,
      const rmw_gid_t &participant_gid,
      const std::string &node_name,
      const std::string &node_namespace);

  /// 将数据写入器与节点解除关联。
  /// Dissociate a data writer from a node.
  /**
   * \param writer_gid 数据写入器的 GUID。
   * \param participant_gid 参与者的 GUID。
   * \param node_name 目标节点的名称。
   * \param node_namespace 目标节点的命名空间。
   * \return 更新其他缓存的消息。
   *
   * \param writer_gid GUID of the data writer.
   * \param participant_gid GUID of the participant.
   * \param node_name Name of the target node.
   * \param node_namespace Namespace of the target node.
   * \return Message to update other caches.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo dissociate_writer(
      const rmw_gid_t &writer_gid,
      const rmw_gid_t &participant_gid,
      const std::string &node_name,
      const std::string &node_namespace);

  /// 将数据读取器与节点关联。
  /// Associate a data reader with a node.
  /**
   * \param reader_gid 数据读取器的 GUID。
   * \param participant_gid 参与者的 GUID。
   * \param node_name 目标节点的名称。
   * \param node_namespace 目标节点的命名空间。
   * \return 用于更新其他缓存的消息。
   *
   * \param reader_gid GUID of the data reader.
   * \param participant_gid GUID of the participant.
   * \param node_name Name of the target node.
   * \param node_namespace Namespace of target node.
   * \return Message to update other caches.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo associate_reader(
      const rmw_gid_t &reader_gid,
      const rmw_gid_t &participant_gid,
      const std::string &node_name,
      const std::string &node_namespace);

  /// 将数据读取器与节点解除关联。
  /// Dissociate a data reader from a node.
  /**
   * \param reader_gid 数据读取器的 GUID。
   * \param participant_gid 参与者的 GUID。
   * \param node_name 目标节点的名称。
   * \param node_namespace 目标节点的命名空间。
   * \return 用于更新其他缓存的消息。
   *
   * \param reader_gid GUID of the data reader.
   * \param participant_gid GUID of the participant.
   * \param node_name Name of the target node.
   * \param node_namespace Namespace of target node.
   * \return Message to update other caches.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo dissociate_reader(
      const rmw_gid_t &reader_gid,
      const rmw_gid_t &participant_gid,
      const std::string &node_name,
      const std::string &node_namespace);

  /**
   * @}
   * \defgroup introspection_api introspection_api
   * 用于内省 GraphCache 的方法。
   * Methods used to introspect the GraphCache.
   * @{
   */

  /// 获取 DDS 主题的数据写入器数量。
  /// Get the number of data writers for a DDS topic.
  /**
   * \param[in] topic_name DDS 主题的名称。
   * \param[out] count 数据写入器的数量。
   *
   * \return RMW_RET_INVALID_ARGUMENT 如果 count 是 `nullptr`，或者
   * \return RMW_RET_ERROR 如果发生意外错误，或者
   * \return RMW_RET_OK。
   *
   * \param[in] topic_name Name of the DDS topic.
   * \param[out] count Number of data writers.
   *
   * \return RMW_RET_INVALID_ARGUMENT if count is `nullptr`, or
   * \return RMW_RET_ERROR if an unexpected error take place, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t get_writer_count(const std::string &topic_name, size_t *count) const;

  /// 获取 DDS 主题的数据读取器数量。
  /// Get the number of data readers for a DDS topic.
  /**
   * \param[in] topic_name DDS 主题的名称。
   * \param[out] count 数据读取器的数量。
   *
   * \return RMW_RET_INVALID_ARGUMENT 如果 count 是 `nullptr`，或者
   * \return RMW_RET_ERROR 如果发生意外错误，或者
   * \return RMW_RET_OK。
   *
   * \param[in] topic_name Name of the DDS topic.
   * \param[out] count Number of data readers.
   *
   * \return RMW_RET_INVALID_ARGUMENT if count is `nullptr`, or
   * \return RMW_RET_ERROR if an unexpected error take place, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t get_reader_count(const std::string &topic_name, size_t *count) const;

  /// 可调用的函数，用于对名称进行解扰。
  /// Callable used to demangle a name.
  using DemangleFunctionT = std::function<std::string(const std::string &)>;

  /// 获取一个数组，其中包含有关 DDS 主题的数据写入器的信息。
  /// Get an array with information about the data writers for a DDS topic.
  /**
   * \param[in] topic_name DDS 主题的名称。
   * \param[in] topic_name Name of the DDS topic.
   * \param[in] demangle_type 用于解扰 DDS 主题类型名称的函数。
   * \param[in] demangle_type Function to demangle DDS topic type names.
   * \param[in] allocator 在填充 `endpoints_info` 时分配内存。
   * \param[in] allocator To allocate memory when populating `endpoints_info`.
   * \param[out] endpoints_info 要使用数据写入器的信息填充的零初始化主题端点信息数组。
   * \param[out] endpoints_info A zero initialized topic endpoint information
   *   array to be populated with data writers' info.
   *
   * \return RMW_RET_INVALID_ARGUMENT 如果 count 是 `nullptr`，或
   * \return RMW_RET_INVALID_ARGUMENT if count is `nullptr`, or
   * \return RMW_RET_ERROR 如果发生意外错误，或
   * \return RMW_RET_ERROR if an unexpected error take place, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t get_writers_info_by_topic(
      const std::string &topic_name,
      DemangleFunctionT demangle_type,
      rcutils_allocator_t *allocator,
      rmw_topic_endpoint_info_array_t *endpoints_info) const;

  /// 获取一个数组，其中包含有关 DDS 主题的数据读取器的信息。
  /// Get an array with information about the data readers for a DDS topic.
  /**
   * \param[in] topic_name DDS 主题的名称。
   * \param[in] topic_name Name of the DDS topic.
   * \param[in] demangle_type 用于解扰 DDS 主题类型名称的函数。
   * \param[in] demangle_type Function to demangle DDS topic type names.
   * \param[in] allocator 在填充 `endpoints_info` 时分配内存。
   * \param[in] allocator To allocate memory when populating `endpoints_info`.
   * \param[out] endpoints_info 要使用数据读取器的信息填充的零初始化主题端点信息数组。
   * \param[out] endpoints_info A zero initialized topic endpoint information
   *   array to be populated with data readers' info.
   *
   * \return RMW_RET_INVALID_ARGUMENT 如果 count 是 `nullptr`，或
   * \return RMW_RET_INVALID_ARGUMENT if count is `nullptr`, or
   * \return RMW_RET_ERROR 如果发生意外错误，或
   * \return RMW_RET_ERROR if an unexpected error take place, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t get_readers_info_by_topic(
      const std::string &topic_name,
      DemangleFunctionT demangle_type,
      rcutils_allocator_t *allocator,
      rmw_topic_endpoint_info_array_t *endpoints_info) const;

  /// 获取所有主题名称和类型。
  /// Get all topic names and types.
  /**
   * \param[in] demangle_topic 用于解扰 DDS 主题名称的函数。
   * \param[in] demangle_topic Function to demangle DDS topic names.
   * \param[in] demangle_type 用于解扰 DDS 主题类型名称的函数。
   * \param[in] demangle_type Function to demangle DDS topic type names.
   * \param[in] allocator 在填充 `topic_names_and_types` 时分配内存。
   * \param[in] allocator To allocate memory when populating `topic_names_and_types`.
   * \param[inout] topic_names_and_types 要使用结果填充的零初始化名称和类型集合。
   * \param[inout] topic_names_and_types A zero initialized names and types collection
   *   to be populated with the result.
   *
   * \return RMW_RET_INVALID_ARGUMENT 如果参数无效，或
   * \return RMW_RET_INVALID_ARGUMENT if an argument is invalid, or
   * \return RMW_RET_BAD_ALLOC 如果分配失败，或
   * \return RMW_RET_BAD_ALLOC if an allocation failed, or
   * \return RMW_RET_ERROR 如果发生意外错误，或
   * \return RMW_RET_ERROR if an unexpected error happened, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t get_names_and_types(
      DemangleFunctionT demangle_topic,
      DemangleFunctionT demangle_type,
      rcutils_allocator_t *allocator,
      rmw_names_and_types_t *topic_names_and_types) const;

  /// 获取与节点关联的所有数据写入器的主题名称和类型。 (Get topic names and types for all data
  /// writers associated to a node.)
  /**
   * \param[in] node_name 节点的名称。(Name of the node.)
   * \param[in] namespace_ 节点的命名空间。(Namespace of the node.)
   * \param[in] demangle_topic 用于对 DDS 主题名称进行解析的函数。(Function to demangle DDS topic
   * names.) \param[in] demangle_type 用于对 DDS 主题类型名称进行解析的函数。(Function to demangle
   * DDS topic type names.) \param[in] allocator 在填充 `topic_names_and_types` 时分配内存。(To
   * allocate memory when populating `topic_names_and_types`.) \param[inout] topic_names_and_types
   * 一个要用结果填充的零初始化名称和类型集合。(A zero initialized names and types collection to be
   * populated with the result.)
   *
   * \return RMW_RET_NODE_NAME_NON_EXISTENT 如果节点不存在，或 (if the node doesn't exist, or)
   * \return RMW_RET_INVALID_ARGUMENT 如果参数无效，或 (if an argument is invalid, or)
   * \return RMW_RET_BAD_ALLOC 如果分配失败，或 (if an allocation failed, or)
   * \return RMW_RET_ERROR 如果发生意外错误，或 (if an unexpected error happened, or)
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t get_writer_names_and_types_by_node(
      const std::string &node_name,
      const std::string &namespace_,
      DemangleFunctionT demangle_topic,
      DemangleFunctionT demangle_type,
      rcutils_allocator_t *allocator,
      rmw_names_and_types_t *topic_names_and_types) const;

  /// 获取与节点关联的所有数据读取器的主题名称和类型。 (Get the topic names and types for all data
  /// readers associated to a node.)
  /**
   * \param[in] node_name 节点的名称。(Name of the node.)
   * \param[in] namespace_ 节点的命名空间。(Namespace of the node.)
   * \param[in] demangle_topic 用于对 DDS 主题名称进行解析的函数。(Function to demangle DDS topic
   * names.) \param[in] demangle_type 用于对 DDS 主题类型名称进行解析的函数。(Function to demangle
   * DDS topic type names.) \param[in] allocator 在填充 `topic_names_and_types` 时分配内存。(To
   * allocate memory when populating `topic_names_and_types`.) \param[inout] topic_names_and_types
   * 一个要用结果填充的零初始化名称和类型集合。(A zero initialized names and types collection to be
   * populated with the result.)
   *
   * \return RMW_RET_NODE_NAME_NON_EXISTENT 如果节点不存在，或 (if the node doesn't exist, or)
   * \return RMW_RET_INVALID_ARGUMENT 如果参数无效，或 (if an argument is invalid, or)
   * \return RMW_RET_BAD_ALLOC 如果分配失败，或 (if an allocation failed, or)
   * \return RMW_RET_ERROR 如果发生意外错误，或 (if an unexpected error happened, or)
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t get_reader_names_and_types_by_node(
      const std::string &node_name,
      const std::string &namespace_,
      DemangleFunctionT demangle_topic,
      DemangleFunctionT demangle_type,
      rcutils_allocator_t *allocator,
      rmw_names_and_types_t *topic_names_and_types) const;

  /// 获取已发现节点的数量。 (Get the number of nodes that have been discovered.)
  RMW_DDS_COMMON_PUBLIC
  size_t get_number_of_nodes() const;

  /// 获取所有节点的名称、命名空间和围场（enclaves）。
  /// Get the names, namespaces, and enclaves of all nodes.
  /**
   * \param[inout] node_names 一个用于填充节点名称的零初始化字符串数组。
   * \param[inout] node_names A zero initialized string array to be populated with node names.
   * \param[inout] node_namespaces
   * 一个用于填充节点命名空间的零初始化字符串数组。此数组中的每个项目对应于`node_names`中相同位置的项目。
   * \param[inout] node_namespaces A zero initialized string array to be populated with node
   *   namespaces. Each item in this array corresponds to the item at the same position in
   *   `node_names`.
   * \param[inout] enclaves
   * 一个用于填充节点围场（enclaves）的零初始化字符串数组。此数组中的每个项目对应于`node_names`中相同位置的项目。如果为`nullptr`，则将被忽略。
   * \param[inout] enclaves A zero initialized string array to be populated with node
   *   enclaves. Each item in this array corresponds to the item at the same position in
   *   `node_names`. If `nullptr`, it will be ignored.
   * \param[in] allocator
   * 分配内存时使用的分配器，用于填充`node_names`、`node_namespaces`和`enclaves`。 \param[in]
   * allocator To allocate memory when populating `node_names`, `node_namespaces`, and `enclaves`.
   * \return RMW_RET_INVALID_ARGUMENT 如果参数无效，或
   * \return RMW_RET_INVALID_ARGUMENT if an argument is invalid, or
   * \return RMW_RET_BAD_ALLOC 如果分配失败，或
   * \return RMW_RET_BAD_ALLOC if an allocation failed, or
   * \return RMW_RET_ERROR 如果发生意外错误，或
   * \return RMW_RET_ERROR if an unexpected error occurred, or
   * \return RMW_RET_OK.
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t get_node_names(
      rcutils_string_array_t *node_names,
      rcutils_string_array_t *node_namespaces,
      rcutils_string_array_t *enclaves,
      rcutils_allocator_t *allocator) const;

  /**
   * @}
   */

  /// \internal
  /// NodeEntitiesInfo 消息序列。
  /// Sequence of NodeEntitiesInfo messages.
  using NodeEntitiesInfoSeq =
      decltype(std::declval<rmw_dds_common::msg::ParticipantEntitiesInfo>().node_entities_info_seq);
  /// \internal
  /// 端点 gid 到端点发现信息的映射。
  /// Map from endpoint gids to endpoints discovery info.
  using EntityGidToInfo = std::map<rmw_gid_t, EntityInfo, Compare_rmw_gid_t>;
  /// \internal
  /// 参与者 gid 到参与者发现信息的映射。
  /// Map from participant gids to participant discovery info.
  using ParticipantToNodesMap = std::map<rmw_gid_t, ParticipantInfo, Compare_rmw_gid_t>;
  /// \internal
  /// 端点 gid 序列。
  /// Sequence of endpoints gids.
  using GidSeq = decltype(std::declval<rmw_dds_common::msg::NodeEntitiesInfo>().writer_gid_seq);

private:
  // 数据写入器（data writers）的映射。
  // Map of data writers.
  EntityGidToInfo data_writers_;
  // 数据读取器（data readers）的映射。
  // Map of data readers.
  EntityGidToInfo data_readers_;
  // 参与者（participants）的映射。
  // Map of participants.
  ParticipantToNodesMap participants_;
  // 更改回调函数。
  // Change callback function.
  std::function<void()> on_change_callback_ = nullptr;

  // 互斥锁，用于保护共享数据。
  // Mutex to protect shared data.
  mutable std::mutex mutex_;
};

// 输出运算符重载，用于将 GraphCache 对象的信息输出到给定的输出流中
// Overloaded output operator, used to output the information of a GraphCache object to the given
// output stream
RMW_DDS_COMMON_PUBLIC
std::ostream &operator<<(std::ostream &ostream, const GraphCache &topic_cache);

/// 代表参与者发现数据的结构
/// Structure representing the discovery data of a Participant
struct ParticipantInfo {
  /// 从参与者创建的每个节点的发现信息
  /// Discovery information of each Node created from a Participant
  GraphCache::NodeEntitiesInfoSeq node_entities_info_seq;

  /// 隔离区名称
  /// Name of the enclave
  std::string enclave;
};

/// 代表端点（数据读取器或写入器）发现数据的结构
struct EntityInfo {
  /// 主题名称
  std::string topic_name;
  /// 主题类型名称
  std::string topic_type;
  /// 主题类型哈希
  rosidl_type_hash_t topic_type_hash;
  /// 参与者 gid
  rmw_gid_t participant_gid;
  /// 主题的服务质量
  rmw_qos_profile_t qos;
  /// 简单构造函数
  EntityInfo(
      const std::string &topic_name,
      const std::string &topic_type,
      const rosidl_type_hash_t &topic_type_hash,
      const rmw_gid_t &participant_gid,
      const rmw_qos_profile_t &qos)
      : topic_name(topic_name),
        topic_type(topic_type),
        topic_type_hash(topic_type_hash),
        participant_gid(participant_gid),
        qos(qos) {}
};

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__GRAPH_CACHE_HPP_
