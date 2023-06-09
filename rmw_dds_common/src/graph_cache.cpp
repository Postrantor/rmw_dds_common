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

#include "rmw_dds_common/graph_cache.hpp"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcutils/strdup.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/sanity_checks.h"
#include "rmw/topic_endpoint_info.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw_dds_common/gid_utils.hpp"

using rmw_dds_common::GraphCache;
using rmw_dds_common::operator<<;

static const char log_tag[] = "rmw_dds_common";  //!< 定义日志标签 (Define log tag)

// 定义宏，用于在满足条件时调用 graph_cache_ptr 的 on_change_callback_ 成员函数
// (Define a macro to call the on_change_callback_ member function of graph_cache_ptr when the
// condition is met)
#define GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK_IF(graph_cache_ptr, condition) \
  do {                                                                     \
    if (graph_cache_ptr->on_change_callback_ && condition) {               \
      graph_cache_ptr->on_change_callback_();                              \
    }                                                                      \
  } while (0);

// 定义宏，用于调用 graph_cache_ptr 的 on_change_callback_ 成员函数
// (Define a macro to call the on_change_callback_ member function of graph_cache_ptr)
#define GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(graph_cache_ptr) \
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK_IF(graph_cache_ptr, true)

/**
 * @brief 清除图缓存的更改回调 (Clear the change callback of the graph cache)
 */
void GraphCache::clear_on_change_callback() {
  std::lock_guard<std::mutex> lock(mutex_);  // 对互斥锁进行加锁 (Lock the mutex)
  on_change_callback_ = nullptr;  // 将更改回调设置为空 (Set the change callback to null)
}

/**
 * @brief 添加数据写入器 (Add data writer)
 *
 * @param[in] gid 写入器全局唯一标识符 (Global unique identifier for the writer)
 * @param[in] topic_name 主题名称 (Topic name)
 * @param[in] type_name 类型名称 (Type name)
 * @param[in] type_hash 类型哈希 (Type hash)
 * @param[in] participant_gid 参与者全局唯一标识符 (Global unique identifier for the participant)
 * @param[in] qos 服务质量配置文件 (Quality of Service profile)
 *
 * @return 是否成功添加写入器 (Whether the writer was added successfully)
 */
bool GraphCache::add_writer(
    const rmw_gid_t &gid,
    const std::string &topic_name,
    const std::string &type_name,
    const rosidl_type_hash_t &type_hash,
    const rmw_gid_t &participant_gid,
    const rmw_qos_profile_t &qos) {
  std::lock_guard<std::mutex> guard(mutex_);  // 对互斥锁进行加锁 (Lock the mutex)
  auto pair = data_writers_.emplace(  // 尝试将数据写入器插入容器中 (Try to insert the data writer
                                      // into the container)
      std::piecewise_construct, std::forward_as_tuple(gid),
      std::forward_as_tuple(topic_name, type_name, type_hash, participant_gid, qos));
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK_IF(
      this,
      pair.second);  // 如果插入成功，调用更改回调 (Call the change callback if insertion is
                     // successful)
  return pair.second;  // 返回是否成功插入 (Return whether the insertion was successful)
}

/**
 * @brief 添加数据写入器 (Add data writer)
 *
 * @param[in] gid 写入器全局唯一标识符 (Global unique identifier for the writer)
 * @param[in] topic_name 主题名称 (Topic name)
 * @param[in] type_name 类型名称 (Type name)
 * @param[in] participant_gid 参与者全局唯一标识符 (Global unique identifier for the participant)
 * @param[in] qos 服务质量配置文件 (Quality of Service profile)
 *
 * @return 是否成功添加写入器 (Whether the writer was added successfully)
 */
bool GraphCache::add_writer(
    const rmw_gid_t &gid,
    const std::string &topic_name,
    const std::string &type_name,
    const rmw_gid_t &participant_gid,
    const rmw_qos_profile_t &qos) {
  // 使用 rosidl_get_zero_initialized_type_hash() 函数初始化类型哈希，并调用另一个 add_writer 函数
  // (Initialize the type hash using the rosidl_get_zero_initialized_type_hash() function and call
  // the other add_writer function)
  return this->add_writer(
      gid, topic_name, type_name, rosidl_get_zero_initialized_type_hash(), participant_gid, qos);
}

/**
 * @brief 添加一个数据读取器 (Add a data reader)
 *
 * @param[in] gid 读取器的全局唯一ID (Global unique ID of the reader)
 * @param[in] topic_name 读取器订阅的主题名称 (Topic name the reader subscribes to)
 * @param[in] type_name 读取器处理的数据类型名称 (Data type name the reader handles)
 * @param[in] type_hash 数据类型的哈希值 (Hash value of the data type)
 * @param[in] participant_gid 参与者的全局唯一ID (Global unique ID of the participant)
 * @param[in] qos 读取器的服务质量配置 (Quality of service configuration for the reader)
 * @return bool 添加成功返回true，否则返回false (Return true if added successfully, otherwise return
 * false)
 */
bool GraphCache::add_reader(
    const rmw_gid_t &gid,
    const std::string &topic_name,
    const std::string &type_name,
    const rosidl_type_hash_t &type_hash,
    const rmw_gid_t &participant_gid,
    const rmw_qos_profile_t &qos) {
  // 使用互斥锁保护数据结构 (Protect data structure with a mutex lock)
  std::lock_guard<std::mutex> guard(mutex_);
  // 在data_readers_中插入新的读取器 (Insert the new reader into data_readers_)
  auto pair = data_readers_.emplace(
      std::piecewise_construct,  //
      std::forward_as_tuple(gid),
      std::forward_as_tuple(topic_name, type_name, type_hash, participant_gid, qos));
  // 如果添加成功，则调用回调函数 (Call the callback function if added successfully)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK_IF(this, pair.second);
  // 返回添加结果 (Return the result of the addition)
  return pair.second;
}

/**
 * @brief 添加一个数据读取器（不包含类型哈希）(Add a data reader without type hash)
 *
 * @param[in] gid 读取器的全局唯一ID (Global unique ID of the reader)
 * @param[in] topic_name 读取器订阅的主题名称 (Topic name the reader subscribes to)
 * @param[in] type_name 读取器处理的数据类型名称 (Data type name the reader handles)
 * @param[in] participant_gid 参与者的全局唯一ID (Global unique ID of the participant)
 * @param[in] qos 读取器的服务质量配置 (Quality of service configuration for the reader)
 * @return bool 添加成功返回true，否则返回false (Return true if added successfully, otherwise return
 * false)
 */
bool GraphCache::add_reader(
    const rmw_gid_t &gid,
    const std::string &topic_name,
    const std::string &type_name,
    const rmw_gid_t &participant_gid,
    const rmw_qos_profile_t &qos) {
  // 调用带有类型哈希的add_reader函数 (Call the add_reader function with type hash)
  return this->add_reader(
      gid, topic_name, type_name, rosidl_get_zero_initialized_type_hash(), participant_gid, qos);
}

/**
 * @brief 添加一个实体（根据is_reader参数选择添加读取器或写入器） (Add an entity (choose to add a
 * reader or writer based on the is_reader parameter))
 *
 * @param[in] gid 实体的全局唯一ID (Global unique ID of the entity)
 * @param[in] topic_name 实体订阅或发布的主题名称 (Topic name the entity subscribes to or publishes)
 * @param[in] type_name 实体处理的数据类型名称 (Data type name the entity handles)
 * @param[in] type_hash 数据类型的哈希值 (Hash value of the data type)
 * @param[in] participant_gid 参与者的全局唯一ID (Global unique ID of the participant)
 * @param[in] qos 实体的服务质量配置 (Quality of service configuration for the entity)
 * @param[in] is_reader 如果为true，则添加读取器，否则添加写入器 (If true, add a reader, otherwise
 * add a writer)
 * @return bool 添加成功返回true，否则返回false (Return true if added successfully, otherwise return
 * false)
 */
bool GraphCache::add_entity(
    const rmw_gid_t &gid,
    const std::string &topic_name,
    const std::string &type_name,
    const rosidl_type_hash_t &type_hash,
    const rmw_gid_t &participant_gid,
    const rmw_qos_profile_t &qos,
    bool is_reader) {
  // 根据is_reader参数选择添加读取器或写入器 (Choose to add a reader or writer based on the
  // is_reader parameter)
  if (is_reader) {
    return this->add_reader(gid, topic_name, type_name, type_hash, participant_gid, qos);
  }
  return this->add_writer(gid, topic_name, type_name, type_hash, participant_gid, qos);
}

/**
 * @brief 添加一个实体（不包含类型哈希） (Add an entity without type hash)
 *
 * @param[in] gid 实体的全局唯一ID (Global unique ID of the entity)
 * @param[in] topic_name 实体订阅或发布的主题名称 (Topic name the entity subscribes to or publishes)
 * @param[in] type_name 实体处理的数据类型名称 (Data type name the entity handles)
 * @param[in] participant_gid 参与者的全局唯一ID (Global unique ID of the participant)
 * @param[in] qos 实体的服务质量配置 (Quality of service configuration for the entity)
 * @param[in] is_reader 如果为true，则添加读取器，否则添加写入器 (If true, add a reader, otherwise
 * add a writer)
 * @return bool 添加成功返回true，否则返回false (Return true if added successfully, otherwise return
 * false)
 */
bool GraphCache::add_entity(
    const rmw_gid_t &gid,
    const std::string &topic_name,
    const std::string &type_name,
    const rmw_gid_t &participant_gid,
    const rmw_qos_profile_t &qos,
    bool is_reader) {
  // 调用带有类型哈希的add_entity函数 (Call the add_entity function with type hash)
  return this->add_entity(
      gid, topic_name, type_name, rosidl_get_zero_initialized_type_hash(), participant_gid, qos,
      is_reader);
}

/**
 * @brief 删除一个数据写入者 (Remove a data writer)
 *
 * @param[in] gid 要删除的数据写入者的全局唯一标识符 (Global unique identifier of the data writer to
 * remove)
 * @return 如果成功删除，则返回 true，否则返回 false (Returns true if the removal is successful,
 * otherwise returns false)
 */
bool GraphCache::remove_writer(const rmw_gid_t &gid) {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> guard(mutex_);

  // 从 data_writers_ 中删除 gid 对应的实体，并检查是否删除成功 (Erase the entity corresponding to
  // gid from data_writers_, and check if the deletion is successful)
  bool ret = data_writers_.erase(gid) > 0;

  // 如果删除成功，调用回调函数 (If the deletion is successful, call the callback function)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK_IF(this, ret);

  return ret;
}

/**
 * @brief 删除一个数据读取者 (Remove a data reader)
 *
 * @param[in] gid 要删除的数据读取者的全局唯一标识符 (Global unique identifier of the data reader to
 * remove)
 * @return 如果成功删除，则返回 true，否则返回 false (Returns true if the removal is successful,
 * otherwise returns false)
 */
bool GraphCache::remove_reader(const rmw_gid_t &gid) {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> guard(mutex_);

  // 从 data_readers_ 中删除 gid 对应的实体，并检查是否删除成功 (Erase the entity corresponding to
  // gid from data_readers_, and check if the deletion is successful)
  bool ret = data_readers_.erase(gid) > 0;

  // 如果删除成功，调用回调函数 (If the deletion is successful, call the callback function)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK_IF(this, ret);

  return ret;
}

/**
 * @brief 删除一个实体，根据给定的参数判断是读取者还是写入者 (Remove an entity, determining whether
 * it is a reader or writer based on the given parameter)
 *
 * @param[in] gid 要删除的实体的全局唯一标识符 (Global unique identifier of the entity to remove)
 * @param[in] is_reader 如果为 true，则表示要删除的实体是数据读取者；否则为数据写入者 (If true, the
 * entity to be removed is a data reader; otherwise, it is a data writer)
 * @return 如果成功删除，则返回 true，否则返回 false (Returns true if the removal is successful,
 * otherwise returns false)
 */
bool GraphCache::remove_entity(const rmw_gid_t &gid, bool is_reader) {
  // 根据 is_reader 参数决定调用 remove_reader 或 remove_writer 函数 (Call remove_reader or
  // remove_writer function depending on the is_reader parameter)
  if (is_reader) {
    return this->remove_reader(gid);
  }
  return this->remove_writer(gid);
}

/**
 * @brief 更新参与者实体信息 (Update participant entities information)
 *
 * @param[in] msg 包含参与者实体信息的消息 (Message containing participant entities information)
 */
void GraphCache::update_participant_entities(
    const rmw_dds_common::msg::ParticipantEntitiesInfo &msg) {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> guard(mutex_);

  // 将 msg 的 gid 转换为 rmw_gid_t 类型 (Convert msg's gid to rmw_gid_t type)
  rmw_gid_t gid;
  rmw_dds_common::convert_msg_to_gid(&msg.gid, &gid);

  // 查找参与者，如果不存在，则创建一个新的参与者实体 (Find the participant, if it does not exist,
  // create a new participant entity)
  auto it = participants_.find(gid);
  if (participants_.end() == it) {
    auto ret = participants_.emplace(
        std::piecewise_construct, std::forward_as_tuple(gid), std::forward_as_tuple());
    it = ret.first;
    assert(ret.second);
  }

  // 更新参与者的节点实体信息序列 (Update the participant's node entities information sequence)
  it->second.node_entities_info_seq = msg.node_entities_info_seq;

  // 调用回调函数 (Call the callback function)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(this);
}

/**
 * @brief 删除一个参与者实体 (Remove a participant entity)
 *
 * @param[in] participant_gid 要删除的参与者实体的全局唯一标识符 (Global unique identifier of the
 * participant entity to remove)
 * @return 如果成功删除，则返回 true，否则返回 false (Returns true if the removal is successful,
 * otherwise returns false)
 */
bool GraphCache::remove_participant(const rmw_gid_t &participant_gid) {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> guard(mutex_);

  // 从 participants_ 中删除 participant_gid 对应的实体，并检查是否删除成功 (Erase the entity
  // corresponding to participant_gid from participants_, and check if the deletion is successful)
  bool ret = participants_.erase(participant_gid) > 0;

  // 如果删除成功，调用回调函数 (If the deletion is successful, call the callback function)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK_IF(this, ret);

  return ret;
}

/**
 * @brief 创建参与者信息消息
 * @param gid 参与者的全局唯一标识符（GID）
 * @param info 节点实体信息序列
 * @return 返回一个包含参与者 GID 和节点实体信息序列的 ParticipantEntitiesInfo 消息
 *
 * @brief Create a participant info message
 * @param gid The Global IDentifier (GID) of the participant
 * @param info Node entities information sequence
 * @return Returns a ParticipantEntitiesInfo message containing the participant GID and node
 * entities info sequence
 */
static rmw_dds_common::msg::ParticipantEntitiesInfo __create_participant_info_message(
    const rmw_gid_t &gid, const GraphCache::NodeEntitiesInfoSeq &info) {
  // 创建一个空的参与者实体信息消息
  // Create an empty ParticipantEntitiesInfo message
  rmw_dds_common::msg::ParticipantEntitiesInfo msg;

  // 将参与者的 GID 转换为消息类型并存储在 msg.gid 中
  // Convert the participant's GID to a message type and store it in msg.gid
  rmw_dds_common::convert_gid_to_msg(&gid, &msg.gid);

  // 设置节点实体信息序列
  // Set the node entities information sequence
  msg.node_entities_info_seq = info;

  // 返回参与者实体信息消息
  // Return the ParticipantEntitiesInfo message
  return msg;
}

/**
 * @brief 添加参与者
 * @param participant_gid 参与者的全局唯一标识符（GID）
 * @param enclave 参与者所属的安全域
 */
void GraphCache::add_participant(const rmw_gid_t &participant_gid, const std::string &enclave) {
  // 使用互斥锁保护数据结构
  // Use a mutex lock to protect the data structure
  std::lock_guard<std::mutex> guard(mutex_);

  // 查找参与者 GID 是否已存在于 participants_ 映射中
  // Check if the participant GID already exists in the participants_ map
  auto it = participants_.find(participant_gid);

  // 如果参与者 GID 不存在，则将其添加到映射中
  // If the participant GID does not exist, add it to the map
  if (participants_.end() == it) {
    auto ret = participants_.emplace(
        std::piecewise_construct, std::forward_as_tuple(participant_gid), std::forward_as_tuple());
    it = ret.first;
    assert(ret.second);
  }

  // 设置参与者的安全域
  // Set the participant's enclave
  it->second.enclave = enclave;

  // 调用回调函数，通知图缓存发生了变化
  // Call the callback function to notify that the graph cache has changed
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(this);
}

/**
 * @brief 添加节点
 * @param participant_gid 参与者的全局唯一标识符（GID）
 * @param node_name 节点名称
 * @param node_namespace 节点命名空间
 * @return 返回一个包含参与者 GID 和节点实体信息序列的 ParticipantEntitiesInfo 消息
 */
rmw_dds_common::msg::ParticipantEntitiesInfo GraphCache::add_node(
    const rmw_gid_t &participant_gid,
    const std::string &node_name,
    const std::string &node_namespace) {
  // 使用互斥锁保护数据结构
  // Use a mutex lock to protect the data structure
  std::lock_guard<std::mutex> guard(mutex_);

  // 查找参与者 GID 是否已存在于 participants_ 映射中
  // Check if the participant GID already exists in the participants_ map
  auto it = participants_.find(participant_gid);
  assert(it != participants_.end());

  // 创建一个节点实体信息对象
  // Create a NodeEntitiesInfo object
  rmw_dds_common::msg::NodeEntitiesInfo node_info;

  // 设置节点名称和命名空间
  // Set the node name and namespace
  node_info.node_name = node_name;
  node_info.node_namespace = node_namespace;

  // 将节点实体信息添加到参与者的节点实体信息序列中
  // Add the node entities info to the participant's node entities info sequence
  it->second.node_entities_info_seq.emplace_back(node_info);

  // 调用回调函数，通知图缓存发生了变化
  // Call the callback function to notify that the graph cache has changed
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(this);

  // 返回包含参与者 GID 和节点实体信息序列的 ParticipantEntitiesInfo 消息
  // Return the ParticipantEntitiesInfo message containing the participant GID and node entities
  // info sequence
  return __create_participant_info_message(participant_gid, it->second.node_entities_info_seq);
}

/**
 * @brief 移除节点
 * @param participant_gid 参与者的全局唯一标识符（GID）
 * @param node_name 节点名称
 * @param node_namespace 节点命名空间
 * @return 返回一个包含参与者 GID 和节点实体信息序列的 ParticipantEntitiesInfo 消息
 */
rmw_dds_common::msg::ParticipantEntitiesInfo GraphCache::remove_node(
    const rmw_gid_t &participant_gid,
    const std::string &node_name,
    const std::string &node_namespace) {
  // 使用互斥锁保护数据结构
  // Use a mutex lock to protect the data structure
  std::lock_guard<std::mutex> guard(mutex_);

  // 查找参与者 GID 是否已存在于 participants_ 映射中
  // Check if the participant GID already exists in the participants_ map
  auto it = participants_.find(participant_gid);
  assert(it != participants_.end());

  // 查找要删除的节点实体信息对象
  // Find the NodeEntitiesInfo object to be removed
  auto to_remove = std::find_if(
      it->second.node_entities_info_seq.begin(), it->second.node_entities_info_seq.end(),
      [&node_name, &node_namespace](const rmw_dds_common::msg::NodeEntitiesInfo &node_info) {
        return node_info.node_name == node_name && node_info.node_namespace == node_namespace;
      });

  assert(to_remove != it->second.node_entities_info_seq.end());

  // 从参与者的节点实体信息序列中删除找到的节点实体信息对象
  // Remove the found NodeEntitiesInfo object from the participant's node entities info sequence
  it->second.node_entities_info_seq.erase(to_remove);

  // 调用回调函数，通知图缓存发生了变化
  // Call the callback function to notify that the graph cache has changed
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(this);

  // 返回包含参与者 GID 和节点实体信息序列的 ParticipantEntitiesInfo 消息
  // Return the ParticipantEntitiesInfo message containing the participant GID and node entities
  // info sequence
  return __create_participant_info_message(participant_gid, it->second.node_entities_info_seq);
}

/**
 * @brief 修改节点信息
 *
 * @tparam FunctorT 函数对象类型，用于修改节点信息
 * @param[in] participant_gid 参与者的全局唯一标识符 (Global Identifier of the participant)
 * @param[in] node_name 节点名称 (Name of the node)
 * @param[in] node_namespace 节点命名空间 (Namespace of the node)
 * @param[in] func 用于修改节点信息的函数对象 (Function object to modify the node information)
 * @param[in,out] participant_map 存储参与者和节点信息的映射 (Mapping of participants and their node
 * information)
 * @return 返回包含修改后的节点信息的消息 (Returns a message containing the modified node
 * information)
 */
template <typename FunctorT>
rmw_dds_common::msg::ParticipantEntitiesInfo __modify_node_info(
    const rmw_gid_t &participant_gid,
    const std::string &node_name,
    const std::string &node_namespace,
    FunctorT func,
    GraphCache::ParticipantToNodesMap &participant_map) {
  // 查找参与者信息 (Find participant information)
  auto participant_info = participant_map.find(participant_gid);
  // 确保找到了参与者信息 (Ensure participant information is found)
  assert(participant_info != participant_map.end());
  // 查找节点信息 (Find node information)
  auto node_info = std::find_if(
      participant_info->second.node_entities_info_seq.begin(),
      participant_info->second.node_entities_info_seq.end(),
      [&](const rmw_dds_common::msg::NodeEntitiesInfo &node_info) {
        return node_info.node_name == node_name && node_info.node_namespace == node_namespace;
      });
  // 确保找到了节点信息 (Ensure node information is found)
  assert(node_info != participant_info->second.node_entities_info_seq.end());

  // 使用函数对象修改节点信息 (Modify node information using the function object)
  func(*node_info);
  // 创建并返回包含修改后的节点信息的消息 (Create and return a message containing the modified node
  // information)
  return __create_participant_info_message(
      participant_gid, participant_info->second.node_entities_info_seq);
}

/**
 * @brief 关联写入者
 *
 * @param[in] writer_gid 写入者的全局唯一标识符 (Global Identifier of the writer)
 * @param[in] participant_gid 参与者的全局唯一标识符 (Global Identifier of the participant)
 * @param[in] node_name 节点名称 (Name of the node)
 * @param[in] node_namespace 节点命名空间 (Namespace of the node)
 * @return 返回包含关联写入者信息的消息 (Returns a message containing the associated writer
 * information)
 */
rmw_dds_common::msg::ParticipantEntitiesInfo GraphCache::associate_writer(
    const rmw_gid_t &writer_gid,
    const rmw_gid_t &participant_gid,
    const std::string &node_name,
    const std::string &node_namespace) {
  // 互斥锁保护 (Mutex protection)
  std::lock_guard<std::mutex> guard(mutex_);
  // 添加写入者的全局唯一标识符 (Add the Global Identifier of the writer)
  auto add_writer_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo &info) {
    info.writer_gid_seq.emplace_back();
    convert_gid_to_msg(&writer_gid, &info.writer_gid_seq.back());
  };
  // 修改节点信息并获取消息 (Modify node information and get the message)
  auto msg =
      __modify_node_info(participant_gid, node_name, node_namespace, add_writer_gid, participants_);

  // 调用更改回调 (Call the change callback)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(this);
  return msg;
}

/**
 * @brief 解除写入者关联
 *
 * @param[in] writer_gid 写入者的全局唯一标识符 (Global Identifier of the writer)
 * @param[in] participant_gid 参与者的全局唯一标识符 (Global Identifier of the participant)
 * @param[in] node_name 节点名称 (Name of the node)
 * @param[in] node_namespace 节点命名空间 (Namespace of the node)
 * @return 返回包含解除写入者关联信息的消息 (Returns a message containing the disassociated writer
 * information)
 */
rmw_dds_common::msg::ParticipantEntitiesInfo GraphCache::dissociate_writer(
    const rmw_gid_t &writer_gid,
    const rmw_gid_t &participant_gid,
    const std::string &node_name,
    const std::string &node_namespace) {
  // 互斥锁保护 (Mutex protection)
  std::lock_guard<std::mutex> guard(mutex_);
  // 将写入者的全局唯一标识符转换为消息格式 (Convert the Global Identifier of the writer to message
  // format)
  rmw_dds_common::msg::Gid writer_gid_msg;
  convert_gid_to_msg(&writer_gid, &writer_gid_msg);
  // 删除写入者的全局唯一标识符 (Delete the Global Identifier of the writer)
  auto delete_writer_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo &info) {
    auto it = std::find_if(
        info.writer_gid_seq.begin(), info.writer_gid_seq.end(),
        [&](const rmw_dds_common::msg::Gid &gid) { return gid == writer_gid_msg; });
    if (it != info.writer_gid_seq.end()) {
      info.writer_gid_seq.erase(it);
    }
  };
  // 修改节点信息并获取消息 (Modify node information and get the message)
  auto msg = __modify_node_info(
      participant_gid, node_name, node_namespace, delete_writer_gid, participants_);

  // 调用更改回调 (Call the change callback)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(this);
  return msg;
}

rmw_dds_common::msg::ParticipantEntitiesInfo GraphCache::associate_reader(
    const rmw_gid_t &reader_gid,  // 读者的全局唯一ID (The Global Unique IDentifier of the reader)
    const rmw_gid_t
        &participant_gid,  // 参与者的全局唯一ID (The Global Unique IDentifier of the participant)
    const std::string &node_name,       // 节点名称 (The name of the node)
    const std::string &node_namespace)  // 节点命名空间 (The namespace of the node)
{
  // 获取互斥锁，保护共享数据 (Acquire the mutex lock to protect shared data)
  std::lock_guard<std::mutex> guard(mutex_);

  // 添加读者GID的lambda函数 (Lambda function to add the reader GID)
  auto add_reader_gid = [&reader_gid](rmw_dds_common::msg::NodeEntitiesInfo &info) {
    // 在读者GID序列的末尾添加一个新元素 (Add a new element at the end of the reader GID sequence)
    info.reader_gid_seq.emplace_back();

    // 将读者的GID转换为消息格式并存储在刚刚创建的元素中 (Convert the reader's GID to message format
    // and store it in the newly created element)
    convert_gid_to_msg(&reader_gid, &info.reader_gid_seq.back());
  };

  // 修改节点信息并添加读者GID (Modify the node information and add the reader GID)
  auto msg =
      __modify_node_info(participant_gid, node_name, node_namespace, add_reader_gid, participants_);

  // 调用图缓存更改回调函数 (Call the Graph Cache change callback function)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(this);

  // 返回参与者实体信息 (Return the Participant Entities Information)
  return msg;
}

/**
 * @brief Dissociate a reader from the participant and node.
 *
 * @param[in] reader_gid        读者的全局唯一ID (The Global Unique IDentifier of the reader)
 * @param[in] participant_gid    参与者的全局唯一ID (The Global Unique IDentifier of the
 * participant)
 * @param[in] node_name          节点名称 (The name of the node)
 * @param[in] node_namespace     节点命名空间 (The namespace of the node)
 * @return rmw_dds_common::msg::ParticipantEntitiesInfo 返回参与者实体信息 (Returns the Participant
 * Entities Information)
 */
rmw_dds_common::msg::ParticipantEntitiesInfo GraphCache::dissociate_reader(
    const rmw_gid_t &reader_gid,
    const rmw_gid_t &participant_gid,
    const std::string &node_name,
    const std::string &node_namespace) {
  // 获取互斥锁，保护共享数据 (Acquire the mutex lock to protect shared data)
  std::lock_guard<std::mutex> guard(mutex_);

  // 将读者的GID转换为消息格式 (Convert the reader's GID to message format)
  rmw_dds_common::msg::Gid reader_gid_msg;
  convert_gid_to_msg(&reader_gid, &reader_gid_msg);

  // 删除读者GID的lambda函数 (Lambda function to delete the reader GID)
  auto delete_reader_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo &info) {
    // 在读者GID序列中查找给定的GID (Find the given GID in the reader GID sequence)
    auto it = std::find_if(
        info.reader_gid_seq.begin(), info.reader_gid_seq.end(),
        [&](const rmw_dds_common::msg::Gid &gid) { return gid == reader_gid_msg; });

    // 如果找到了GID，则从序列中删除它 (If the GID is found, erase it from the sequence)
    if (it != info.reader_gid_seq.end()) {
      info.reader_gid_seq.erase(it);
    }
  };

  // 修改节点信息并删除读者GID (Modify the node information and delete the reader GID)
  auto msg = __modify_node_info(
      participant_gid, node_name, node_namespace, delete_reader_gid, participants_);

  // 调用图缓存更改回调函数 (Call the Graph Cache change callback function)
  GRAPH_CACHE_CALL_ON_CHANGE_CALLBACK(this);

  // 返回参与者实体信息 (Return the Participant Entities Information)
  return msg;
}

/**
 * @brief 获取给定主题名称的实体数量
 * @param[in] entities 实体信息映射，从全局唯一标识符 (GID) 到实体信息
 * @param[in] topic_name 要查询的主题名称
 * @param[out] count 匹配主题名称的实体数量
 * @return RMW_RET_OK if successful, or an error code otherwise
 *
 * @brief Get the count of entities with a given topic name
 * @param[in] entities A mapping of entity information from Global Identifier (GID) to the Entity
 * Info
 * @param[in] topic_name The topic name to query for
 * @param[out] count The number of entities matching the topic name
 * @return RMW_RET_OK if successful, or an error code otherwise
 */
static rmw_ret_t __get_count(
    const GraphCache::EntityGidToInfo &entities, std::string topic_name, size_t *count) {
  // 断言 count 非空
  // Assert that count is not null
  assert(count);

  // 计算匹配主题名称的实体数量
  // Calculate the count of entities matching the topic name
  *count = std::count_if(
      entities.begin(), entities.end(),
      [&topic_name](const GraphCache::EntityGidToInfo::value_type &elem) {
        return elem.second.topic_name == topic_name;
      });
  return RMW_RET_OK;
}

/**
 * @brief 获取给定主题名称的 writer 数量
 * @param[in] topic_name 要查询的主题名称
 * @param[out] count 匹配主题名称的 writer 数量
 * @return RMW_RET_OK if successful, or an error code otherwise
 *
 * @brief Get the count of writers with a given topic name
 * @param[in] topic_name The topic name to query for
 * @param[out] count The number of writers matching the topic name
 * @return RMW_RET_OK if successful, or an error code otherwise
 */
rmw_ret_t GraphCache::get_writer_count(const std::string &topic_name, size_t *count) const {
  // 互斥锁保护
  // Mutex lock protection
  std::lock_guard<std::mutex> guard(mutex_);

  // 检查 count 是否为空，如果为空则返回无效参数错误
  // Check if count is null, return invalid argument error if it is
  if (!count) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 调用 __get_count 函数获取 writer 数量
  // Call the __get_count function to get the count of writers
  return __get_count(data_writers_, topic_name, count);
}

/**
 * @brief 获取给定主题名称的 reader 数量
 * @param[in] topic_name 要查询的主题名称
 * @param[out] count 匹配主题名称的 reader 数量
 * @return RMW_RET_OK if successful, or an error code otherwise
 *
 * @brief Get the count of readers with a given topic name
 * @param[in] topic_name The topic name to query for
 * @param[out] count The number of readers matching the topic name
 * @return RMW_RET_OK if successful, or an error code otherwise
 */
rmw_ret_t GraphCache::get_reader_count(const std::string &topic_name, size_t *count) const {
  // 互斥锁保护
  // Mutex lock protection
  std::lock_guard<std::mutex> guard(mutex_);

  // 检查 count 是否为空，如果为空则返回无效参数错误
  // Check if count is null, return invalid argument error if it is
  if (!count) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 调用 __get_count 函数获取 reader 数量
  // Call the __get_count function to get the count of readers
  return __get_count(data_readers_, topic_name, count);
}

// 定义 EndpointCreator 枚举类
// Define the EndpointCreator enumeration class
enum class EndpointCreator {
  ROS_NODE = 0,               // ROS 节点创建的端点
  UNDISCOVERED_ROS_NODE = 1,  // 尚未发现的 ROS 节点创建的端点
  BARE_DDS_PARTICIPANT = 2,   // Bare DDS 参与者创建的端点
};

/**
 * @brief 查找实体 GID 对应的名称和命名空间 (Find the name and namespace corresponding to the entity
 * GID)
 *
 * @param[in] participant_map 参与者到节点的映射 (Participant to nodes mapping)
 * @param[in] participant_gid 参与者 GID (Participant GID)
 * @param[in] entity_gid 实体 GID (Entity GID)
 * @param[in] is_reader 是否为读取器 (Whether it is a reader)
 * @return std::tuple<std::string, std::string, EndpointCreator>
 * 包含名称、命名空间和端点创建者的元组 (Tuple containing name, namespace, and endpoint creator)
 */
static std::tuple<std::string, std::string, EndpointCreator>
__find_name_and_namespace_from_entity_gid(
    const GraphCache::ParticipantToNodesMap &participant_map,
    rmw_gid_t participant_gid,
    rmw_gid_t entity_gid,
    bool is_reader) {
  // 在参与者映射中查找给定的参与者 GID (Find the given participant GID in the participant map)
  auto it = participant_map.find(participant_gid);
  // 如果找不到参与者 GID，则返回一个空的元组 (If the participant GID is not found, return an empty
  // tuple)
  if (participant_map.end() == it) {
    return {"", "", EndpointCreator::BARE_DDS_PARTICIPANT};
  }
  // 遍历参与者对应的节点实体信息序列 (Iterate through the node entities info sequence corresponding
  // to the participant)
  for (const auto &node_info : it->second.node_entities_info_seq) {
    // 根据 is_reader 参数选择读取器或写入器 GID 序列 (Select the reader or writer GID sequence
    // based on the is_reader parameter)
    auto &gid_seq = is_reader ? node_info.reader_gid_seq : node_info.writer_gid_seq;
    // 在 GID 序列中查找与实体 GID 匹配的 GID (Find the GID in the GID sequence that matches the
    // entity GID)
    auto it = std::find_if(
        gid_seq.begin(), gid_seq.end(), [&](const rmw_dds_common::msg::Gid &gid) -> bool {
          return 0u == std::memcmp(gid.data.data(), entity_gid.data, RMW_GID_STORAGE_SIZE);
        });
    // 如果在 GID 序列中找到匹配的 GID，则返回对应的节点名称、命名空间和端点创建者 (If a matching
    // GID is found in the GID sequence, return the corresponding node name, namespace, and endpoint
    // creator)
    if (gid_seq.end() != it) {
      return {node_info.node_name, node_info.node_namespace, EndpointCreator::ROS_NODE};
    }
  }
  // 如果找不到匹配的节点，返回一个表示未发现的 ROS 节点的元组 (If no matching node is found, return
  // a tuple indicating an undiscovered ROS node)
  return {"", "", EndpointCreator::UNDISCOVERED_ROS_NODE};
}

// 使用 DemangleFunctionT 类型定义一个别名，该类型是 GraphCache 类中的一个成员函数类型。
// Define an alias using DemangleFunctionT type, which is a member function type in the GraphCache
// class.
using DemangleFunctionT = GraphCache::DemangleFunctionT;

// 定义一个静态函数，用于通过主题获取实体信息。
static rmw_ret_t __get_entities_info_by_topic(
    const GraphCache::EntityGidToInfo &entities,
    const GraphCache::ParticipantToNodesMap &participant_map,
    const std::string &topic_name,
    DemangleFunctionT demangle_type,
    bool is_reader,
    rcutils_allocator_t *allocator,
    rmw_topic_endpoint_info_array_t *endpoints_info) {
  // 断言分配器和端点信息指针不为空。
  assert(allocator);
  assert(endpoints_info);

  // 如果实体大小为0，则返回 RMW_RET_OK。
  if (0u == entities.size()) {
    return RMW_RET_OK;
  }

  // 计算与给定主题名称匹配的实体数量。
  size_t size = std::count_if(
      entities.begin(), entities.end(),
      [&topic_name](const GraphCache::EntityGidToInfo::value_type &item) {
        return item.second.topic_name == topic_name;
      });
  // 如果实体数量为0，则返回 RMW_RET_OK。
  if (0u == size) {
    return RMW_RET_OK;
  }

  // 使用指定的分配器和大小初始化端点信息数组。
  rmw_ret_t ret = rmw_topic_endpoint_info_array_init_with_size(endpoints_info, size, allocator);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  // 定义一个 unique_ptr，用于在出错时删除端点信息数组。
  std::unique_ptr<
      rmw_topic_endpoint_info_array_t, std::function<void(rmw_topic_endpoint_info_array_t *)>>
      endpoints_info_delete_on_error(
          endpoints_info, [allocator](rmw_topic_endpoint_info_array_t *p) {
            rmw_ret_t ret = rmw_topic_endpoint_info_array_fini(p, allocator);
            if (RMW_RET_OK != ret) {
              RCUTILS_LOG_ERROR_NAMED(
                  log_tag, "Failed to destroy endpoints_info when function failed.");
            }
          });

  // 遍历实体并根据主题名称筛选实体。
  size_t i = 0;
  for (const auto &entity_pair : entities) {
    if (entity_pair.second.topic_name != topic_name) {
      continue;
    }

    // 获取当前端点信息引用。
    rmw_topic_endpoint_info_t &endpoint_info = endpoints_info->info_array[i];
    endpoint_info = rmw_get_zero_initialized_topic_endpoint_info();

    // 从实体 GID 中查找节点名称和命名空间。
    auto result = __find_name_and_namespace_from_entity_gid(
        participant_map, entity_pair.second.participant_gid, entity_pair.first, is_reader);

    std::string node_name;
    std::string node_namespace;
    // 根据端点创建者类型设置节点名称和命名空间。
    switch (std::get<2>(result)) {
      case EndpointCreator::ROS_NODE:
        node_name = std::move(std::get<0>(result));
        node_namespace = std::move(std::get<1>(result));
        break;
      case EndpointCreator::UNDISCOVERED_ROS_NODE:
        node_name = "_NODE_NAME_UNKNOWN_";
        node_namespace = "_NODE_NAMESPACE_UNKNOWN_";
        break;
      case EndpointCreator::BARE_DDS_PARTICIPANT:
        node_name = "_CREATED_BY_BARE_DDS_APP_";
        node_namespace = "_CREATED_BY_BARE_DDS_APP_";
        break;
    }

    // 设置端点信息的节点名称。
    ret = rmw_topic_endpoint_info_set_node_name(&endpoint_info, node_name.c_str(), allocator);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    // 设置端点信息的节点命名空间。
    ret = rmw_topic_endpoint_info_set_node_namespace(
        &endpoint_info, node_namespace.c_str(), allocator);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    // 设置端点信息的主题类型。
    ret = rmw_topic_endpoint_info_set_topic_type(
        &endpoint_info, demangle_type(entity_pair.second.topic_type).c_str(), allocator);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    // 设置端点信息的主题类型哈希。
    ret = rmw_topic_endpoint_info_set_topic_type_hash(
        &endpoint_info, &entity_pair.second.topic_type_hash);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    // 设置端点信息的端点类型（发布者或订阅者）。
    ret = rmw_topic_endpoint_info_set_endpoint_type(
        &endpoint_info, is_reader ? RMW_ENDPOINT_SUBSCRIPTION : RMW_ENDPOINT_PUBLISHER);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    // 设置端点信息的 GID。
    ret = rmw_topic_endpoint_info_set_gid(
        &endpoint_info, entity_pair.first.data, RMW_GID_STORAGE_SIZE);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    // 设置端点信息的 QoS 配置文件。
    ret = rmw_topic_endpoint_info_set_qos_profile(&endpoint_info, &entity_pair.second.qos);
    if (RMW_RET_OK != ret) {
      return ret;
    }
    // 增加索引值。
    // Increment the index value.
    i++;
  }

  // 释放 unique_ptr，以避免在函数返回时删除端点信息数组。
  // Release the unique_ptr to avoid deleting the endpoint information array when the function
  // returns.
  endpoints_info_delete_on_error.release();
  return RMW_RET_OK;
}

/**
 * @brief 获取指定主题上的发布者信息 (Get the publisher information on the specified topic)
 *
 * @param[in] topic_name 主题名称 (Topic name)
 * @param[in] demangle_type 类型反混淆函数 (Type demangling function)
 * @param[in] allocator 分配器 (Allocator)
 * @param[out] endpoints_info 发布者信息数组 (Publisher information array)
 * @return rmw_ret_t RMW 返回值 (RMW return value)
 */
rmw_ret_t GraphCache::get_writers_info_by_topic(
    const std::string &topic_name,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t *allocator,
    rmw_topic_endpoint_info_array_t *endpoints_info) const {
  // 对互斥量加锁，确保线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::mutex> lock(mutex_);
  // 调用内部函数获取发布者信息 (Call the internal function to get the publisher information)
  return __get_entities_info_by_topic(
      data_writers_, participants_, topic_name, demangle_type, false, allocator, endpoints_info);
}

/**
 * @brief 获取指定主题上的订阅者信息 (Get the subscriber information on the specified topic)
 *
 * @param[in] topic_name 主题名称 (Topic name)
 * @param[in] demangle_type 类型反混淆函数 (Type demangling function)
 * @param[in] allocator 分配器 (Allocator)
 * @param[out] endpoints_info 订阅者信息数组 (Subscriber information array)
 * @return rmw_ret_t RMW 返回值 (RMW return value)
 */
rmw_ret_t GraphCache::get_readers_info_by_topic(
    const std::string &topic_name,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t *allocator,
    rmw_topic_endpoint_info_array_t *endpoints_info) const {
  // 对互斥量加锁，确保线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::mutex> lock(mutex_);
  // 调用内部函数获取订阅者信息 (Call the internal function to get the subscriber information)
  return __get_entities_info_by_topic(
      data_readers_, participants_, topic_name, demangle_type, true, allocator, endpoints_info);
}

// 定义名称和类型映射结构 (Define name and type mapping structure)
using NamesAndTypes = std::map<std::string, std::set<std::string>>;

/**
 * @brief 获取实体的名称和类型 (Get the names and types of entities)
 *
 * @param[in] entities 实体信息集合 (Entity information collection)
 * @param[in] demangle_topic 主题反混淆函数 (Topic demangling function)
 * @param[in] demangle_type 类型反混淆函数 (Type demangling function)
 * @param[out] topics 主题名称和类型映射 (Topic name and type mapping)
 */
static void __get_names_and_types(
    const GraphCache::EntityGidToInfo &entities,
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type,
    NamesAndTypes &topics) {
  // 断言：检查输入参数是否非空 (Assert: Check if input parameters are non-null)
  assert(nullptr != demangle_topic);
  assert(nullptr != demangle_type);
  // 遍历实体集合，获取名称和类型 (Iterate through the entity collection to get names and types)
  for (const auto &item : entities) {
    // 反混淆主题名称 (Demangle the topic name)
    std::string demangled_topic_name = demangle_topic(item.second.topic_name);
    // 如果反混淆后的主题名称非空，则添加到映射中 (If the demangled topic name is non-empty, add it
    // to the mapping)
    if ("" != demangled_topic_name) {
      topics[demangled_topic_name].insert(demangle_type(item.second.topic_type));
    }
  }
}

/**
 * @brief 为 rmw_names_and_types_t 结构体填充主题名称和类型信息 (Populate rmw_names_and_types_t
 * structure with topic names and types information)
 *
 * @param[in] topics 包含主题名称和类型的映射 (A mapping containing topic names and types)
 * @param[in] allocator 分配器用于分配内存 (Allocator used for memory allocation)
 * @param[out] topic_names_and_types 用于存储主题名称和类型信息的结构体 (Structure to store the
 * topic names and types information)
 * @return rmw_ret_t 操作结果状态码 (Operation result status code)
 */
static rmw_ret_t __populate_rmw_names_and_types(
    NamesAndTypes topics,
    rcutils_allocator_t *allocator,
    rmw_names_and_types_t *topic_names_and_types) {
  // 如果 topics 为空，直接返回成功状态码 (If topics is empty, return success status code directly)
  if (topics.empty()) {
    return RMW_RET_OK;
  }

  // 初始化 topic_names_and_types 结构体 (Initialize the topic_names_and_types structure)
  rmw_ret_t rmw_ret = rmw_names_and_types_init(topic_names_and_types, topics.size(), allocator);
  if (RMW_RET_OK != rmw_ret) {
    return rmw_ret;
  }

  size_t index = 0;
  // 遍历 topics 映射，并将主题名称和类型添加到 topic_names_and_types 结构体中 (Iterate through the
  // topics map and add topic names and types to the topic_names_and_types structure)
  for (const auto &item : topics) {
    char *topic_name = rcutils_strdup(item.first.c_str(), *allocator);
    if (!topic_name) {
      RMW_SET_ERROR_MSG("failed to allocate memory for topic name");
      rmw_ret = RMW_RET_BAD_ALLOC;
      goto cleanup;
    }
    topic_names_and_types->names.data[index] = topic_name;

    {
      rcutils_ret_t rcutils_ret = rcutils_string_array_init(
          &topic_names_and_types->types[index], item.second.size(), allocator);
      if (RCUTILS_RET_OK != rcutils_ret) {
        RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
        rmw_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
        goto cleanup;
      }
    }
    size_t type_index = 0;
    // 遍历类型并将它们添加到 topic_names_and_types 结构体中 (Iterate through the types and add them
    // to the topic_names_and_types structure)
    for (const auto &type : item.second) {
      char *type_name = rcutils_strdup(type.c_str(), *allocator);
      if (!type_name) {
        RMW_SET_ERROR_MSG("failed to allocate memory for type name");
        rmw_ret = RMW_RET_BAD_ALLOC;
        goto cleanup;
      }
      topic_names_and_types->types[index].data[type_index] = type_name;
      ++type_index;
    }
    ++index;
  }
  return RMW_RET_OK;
cleanup:
  // 如果清理操作失败，则记录错误日志 (If the cleanup operation fails, log the error)
  if (RMW_RET_OK != rmw_names_and_types_fini(topic_names_and_types)) {
    RCUTILS_LOG_ERROR_NAMED(
        log_tag, "error during report of error: %s", rmw_get_error_string().str);
  }
  return rmw_ret;
}

/**
 * @brief 获取话题名称和类型（Get topic names and types）
 *
 * @param[in] demangle_topic 用于解析话题名称的函数（Function for demangling topic names）
 * @param[in] demangle_type 用于解析类型名称的函数（Function for demangling type names）
 * @param[in] allocator 分配器（Allocator）
 * @param[out] topic_names_and_types 存储获取到的话题名称和类型的结构体指针（Pointer to the
 * structure that stores the obtained topic names and types）
 * @return rmw_ret_t 返回状态码（Return status code）
 */
rmw_ret_t GraphCache::get_names_and_types(
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t *allocator,
    rmw_names_and_types_t *topic_names_and_types) const {
  // 检查参数有效性（Check parameter validity）
  assert(demangle_topic);
  assert(demangle_type);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);
  if (RMW_RET_OK != rmw_names_and_types_check_zero(topic_names_and_types)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // TODO(ivanpauno): 避免使用中间表示（Avoid using an intermediate representation）
  // 我们需要一种重新分配 `topic_names_and_types.names` 和 `topic_names_and_types.names` 的方法
  // 或者对大小有一个好的猜测（下界），然后收缩。
  // (We need a way to reallocate `topic_names_and_types.names` and `topic_names_and_types.names`
  // Or have a good guess of the size (lower bound), and then shrink)
  NamesAndTypes topics;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    __get_names_and_types(data_readers_, demangle_topic, demangle_type, topics);
    __get_names_and_types(data_writers_, demangle_topic, demangle_type, topics);
  }

  return __populate_rmw_names_and_types(topics, allocator, topic_names_and_types);
}

/**
 * @brief 查找节点（Find node）
 *
 * @param[in] participant_map 参与者到节点的映射（Participant to nodes mapping）
 * @param[in] node_name 要查找的节点名称（Node name to find）
 * @param[in] node_namespace 要查找的节点命名空间（Node namespace to find）
 * @return const rmw_dds_common::msg::NodeEntitiesInfo* 返回找到的节点指针，如果未找到，则返回
 * nullptr（Return pointer to the found node, or nullptr if not found）
 */
static const rmw_dds_common::msg::NodeEntitiesInfo *__find_node(
    const GraphCache::ParticipantToNodesMap &participant_map,
    const std::string &node_name,
    const std::string &node_namespace) {
  for (const auto &participant : participant_map) {
    for (const auto &node : participant.second.node_entities_info_seq) {
      if (node.node_name == node_name && node.node_namespace == node_namespace) {
        return &node;
      }
    }
  }
  return nullptr;
}

/**
 * @brief 从 GID 获取名称和类型（Get names and types from GIDs）
 *
 * @param[in] entities_map 实体 GID 到信息的映射（Entity GID to info mapping）
 * @param[in] gids GID 序列（GID sequence）
 * @param[in] demangle_topic 用于解析话题名称的函数（Function for demangling topic names）
 * @param[in] demangle_type 用于解析类型名称的函数（Function for demangling type names）
 * @return NamesAndTypes 返回获取到的名称和类型（Return obtained names and types）
 */
static NamesAndTypes __get_names_and_types_from_gids(
    const GraphCache::EntityGidToInfo &entities_map,
    const GraphCache::GidSeq &gids,
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type) {
  NamesAndTypes topics;

  for (const auto &gid_msg : gids) {
    rmw_gid_t gid;
    rmw_dds_common::convert_msg_to_gid(&gid_msg, &gid);
    auto it = entities_map.find(gid);
    if (it == entities_map.end()) {
      continue;
    }
    std::string demangled_topic_name = demangle_topic(it->second.topic_name);
    if ("" == demangled_topic_name) {
      continue;
    }
    topics[demangled_topic_name].insert(demangle_type(it->second.topic_type));
  }
  return topics;
}

// 使用 GetEntitiesGidsFuncT 类型定义一个函数对象类型，用于获取实体的 GID 序列
// Define a function object type, GetEntitiesGidsFuncT, for obtaining the GID sequence of entities
using GetEntitiesGidsFuncT =
    std::function<const GraphCache::GidSeq &(const rmw_dds_common::msg::NodeEntitiesInfo &)>;

// 定义一个静态函数 __get_names_and_types_by_node，根据节点获取相关实体的名称和类型
// Define a static function, __get_names_and_types_by_node, to get names and types of related
// entities based on the node
static rmw_ret_t __get_names_and_types_by_node(
    const GraphCache::ParticipantToNodesMap &participants_map,
    const GraphCache::EntityGidToInfo &entities_map,
    const std::string &node_name,
    const std::string &namespace_,
    DemangleFunctionT demangle_topic,              // 解析主题名的函数
    DemangleFunctionT demangle_type,               // 解析类型名的函数
    GetEntitiesGidsFuncT get_entities_gids,        // 获取实体 GID 的函数
    rcutils_allocator_t *allocator,                // 分配器
    rmw_names_and_types_t *topic_names_and_types)  // 存储结果的数据结构
{
  // 检查解析主题名和类型名的函数是否有效
  // Check if demangle_topic and demangle_type functions are valid
  assert(demangle_topic);
  assert(demangle_type);

  // 检查分配器是否有效
  // Check if the allocator is valid
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  // 检查 topic_names_and_types 是否为空
  // Check if topic_names_and_types is empty
  if (RMW_RET_OK != rmw_names_and_types_check_zero(topic_names_and_types)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 查找指定节点
  // Find the specified node
  auto node_info_ptr = __find_node(participants_map, node_name, namespace_);

  // 如果找不到指定节点，返回错误代码
  // If the specified node cannot be found, return an error code
  if (nullptr == node_info_ptr) {
    return RMW_RET_NODE_NAME_NON_EXISTENT;
  }

  // 根据 GID 获取名称和类型
  // Get names and types based on GID
  NamesAndTypes topics = __get_names_and_types_from_gids(
      entities_map, get_entities_gids(*node_info_ptr), demangle_topic, demangle_type);

  // 将获取到的名称和类型填充到结果数据结构中
  // Populate the obtained names and types into the result data structure
  return __populate_rmw_names_and_types(topics, allocator, topic_names_and_types);
}

// 定义一个静态函数 __get_writers_gids，用于获取节点的 writer GID 序列
// Define a static function, __get_writers_gids, to get the writer GID sequence of a node
static const GraphCache::GidSeq &__get_writers_gids(
    const rmw_dds_common::msg::NodeEntitiesInfo &node_info) {
  return node_info.writer_gid_seq;
}

// 定义一个成员函数 get_writer_names_and_types_by_node，根据节点获取相关 writer 的名称和类型
// Define a member function, get_writer_names_and_types_by_node, to get names and types of related
// writers based on the node
rmw_ret_t GraphCache::get_writer_names_and_types_by_node(
    const std::string &node_name,
    const std::string &namespace_,
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t *allocator,
    rmw_names_and_types_t *topic_names_and_types) const {
  // 对互斥量进行加锁保护
  // Lock the mutex for protection
  std::lock_guard<std::mutex> guard(mutex_);

  // 调用 __get_names_and_types_by_node 函数获取 writer 的名称和类型
  // Call the __get_names_and_types_by_node function to get the names and types of writers
  return __get_names_and_types_by_node(
      participants_, data_writers_, node_name, namespace_, demangle_topic, demangle_type,
      __get_writers_gids, allocator, topic_names_and_types);
}

// 定义一个静态函数 __get_readers_gids，用于获取节点的 reader GID 序列
// Define a static function, __get_readers_gids, to get the reader GID sequence of a node
static const GraphCache::GidSeq &__get_readers_gids(
    const rmw_dds_common::msg::NodeEntitiesInfo &node_info) {
  return node_info.reader_gid_seq;
}

/**
 * @brief 获取节点的读者名称和类型 (Get reader names and types by node)
 *
 * @param[in] node_name 节点名称 (Node name)
 * @param[in] namespace_ 命名空间 (Namespace)
 * @param[in] demangle_topic 反混淆主题函数 (Demangle function for topic)
 * @param[in] demangle_type 反混淆类型函数 (Demangle function for type)
 * @param[in] allocator 内存分配器 (Memory allocator)
 * @param[out] topic_names_and_types 读者主题名称和类型的输出结构体 (Output structure for reader
 * topic names and types)
 * @return rmw_ret_t 返回操作状态 (Return operation status)
 */
rmw_ret_t GraphCache::get_reader_names_and_types_by_node(
    const std::string &node_name,
    const std::string &namespace_,
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t *allocator,
    rmw_names_and_types_t *topic_names_and_types) const {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> guard(mutex_);
  // 调用内部方法获取节点的读者名称和类型 (Call internal method to get reader names and types by
  // node)
  return __get_names_and_types_by_node(
      participants_, data_readers_, node_name, namespace_, demangle_topic, demangle_type,
      __get_readers_gids, allocator, topic_names_and_types);
}

/**
 * @brief 计算参与者映射中的节点数量 (Calculate the number of nodes in the participant map)
 *
 * @param[in] participants_map 参与者到节点映射 (Participant to nodes map)
 * @return size_t 节点数量 (Number of nodes)
 */
static size_t __get_number_of_nodes(const GraphCache::ParticipantToNodesMap &participants_map) {
  // 初始化节点数量为0 (Initialize the number of nodes to 0)
  size_t nodes_number = 0;
  // 遍历参与者映射 (Iterate through the participant map)
  for (const auto &elem : participants_map) {
    // 累加当前参与者的节点数 (Accumulate the number of nodes for the current participant)
    nodes_number += elem.second.node_entities_info_seq.size();
  }
  // 返回节点总数 (Return the total number of nodes)
  return nodes_number;
}

/**
 * @brief 获取节点数量 (Get the number of nodes)
 *
 * @return size_t 节点数量 (Number of nodes)
 */
size_t GraphCache::get_number_of_nodes() const {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> guard(mutex_);
  // 调用内部方法获取节点数量 (Call internal method to get the number of nodes)
  return __get_number_of_nodes(participants_);
}

/**
 * @brief 获取节点名称、命名空间和安全区域
 *        Get node names, namespaces and enclaves
 *
 * @param[out] node_names 节点名称数组 Node name array
 * @param[out] node_namespaces 节点命名空间数组 Node namespace array
 * @param[out] enclaves 安全区域数组 Enclave array (optional)
 * @param[in] allocator 分配器 Allocator
 * @return rmw_ret_t 返回状态 Return status
 */
rmw_ret_t GraphCache::get_node_names(
    rcutils_string_array_t *node_names,
    rcutils_string_array_t *node_namespaces,
    rcutils_string_array_t *enclaves,
    rcutils_allocator_t *allocator) const {
  // 加锁以保护共享数据
  // Lock to protect shared data
  std::lock_guard<std::mutex> guard(mutex_);

  // 检查参数有效性
  // Check parameter validity
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_names)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_namespaces)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (enclaves && RMW_RET_OK != rmw_check_zero_rmw_string_array(enclaves)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // 检查分配器有效性
  // Check allocator validity
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
      allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  // 获取节点数量
  // Get number of nodes
  size_t nodes_number = __get_number_of_nodes(participants_);

  // 初始化节点名称数组
  // Initialize node name array
  rcutils_ret_t rcutils_ret = rcutils_string_array_init(node_names, nodes_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    rcutils_error_string_t error_msg = rcutils_get_error_string();
    rcutils_reset_error();
    RMW_SET_ERROR_MSG(error_msg.str);
    goto fail;
  }

  // 初始化节点命名空间数组
  // Initialize node namespace array
  rcutils_ret = rcutils_string_array_init(node_namespaces, nodes_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    rcutils_error_string_t error_msg = rcutils_get_error_string();
    rcutils_reset_error();
    RMW_SET_ERROR_MSG(error_msg.str);
    goto fail;
  }

  // 如果提供了安全区域数组，初始化它
  // If enclaves array is provided, initialize it
  if (enclaves) {
    rcutils_ret = rcutils_string_array_init(enclaves, nodes_number, allocator);
    if (RCUTILS_RET_OK != rcutils_ret) {
      rcutils_error_string_t error_msg = rcutils_get_error_string();
      rcutils_reset_error();
      RMW_SET_ERROR_MSG(error_msg.str);
      goto fail;
    }
  }

  // 遍历参与者并填充节点名称、命名空间和安全区域数组
  // Iterate through participants and populate node name, namespace and enclave arrays
  {
    size_t j = 0;
    for (const auto &elem : participants_) {
      const auto &nodes_info = elem.second;
      for (const auto &node_info : nodes_info.node_entities_info_seq) {
        node_names->data[j] = rcutils_strdup(node_info.node_name.c_str(), *allocator);
        if (!node_names->data[j]) {
          goto fail;
        }
        node_namespaces->data[j] = rcutils_strdup(node_info.node_namespace.c_str(), *allocator);
        if (!node_namespaces->data[j]) {
          goto fail;
        }
        if (enclaves) {
          enclaves->data[j] = rcutils_strdup(nodes_info.enclave.c_str(), *allocator);
          if (!enclaves->data[j]) {
            goto fail;
          }
        }
        j++;
      }
    }
  }

  // 返回成功状态
  // Return success status
  return RMW_RET_OK;

// 处理失败情况并清理资源
// Handle failure case and clean up resources
fail:
  rcutils_ret = rcutils_string_array_fini(node_names);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
        "rmw_dds_common", "failed to cleanup during error handling: %s",
        rcutils_get_error_string().str);
  }
  rcutils_ret = rcutils_string_array_fini(node_namespaces);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
        "rmw_dds_common", "failed to cleanup during error handling: %s",
        rcutils_get_error_string().str);
  }
  if (enclaves) {
    rcutils_ret = rcutils_string_array_fini(enclaves);
    if (rcutils_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
          "rmw_dds_common", "failed to cleanup during error handling: %s",
          rcutils_get_error_string().str);
    }
  }

  // 返回错误状态
  // Return error status
  return RMW_RET_BAD_ALLOC;
}

/**
 * @brief 输出 GraphCache 对象的内容到输出流中。
 *        Output the content of a GraphCache object to an output stream.
 *
 * @param ostream 输出流，用于接收 GraphCache 对象的内容。
 *                The output stream to receive the content of the GraphCache object.
 * @param graph_cache 要输出内容的 GraphCache 对象。
 *                    The GraphCache object whose content is to be output.
 * @return std::ostream& 更新后的输出流。
 *                      The updated output stream.
 */
std::ostream &rmw_dds_common::operator<<(std::ostream &ostream, const GraphCache &graph_cache) {
  // 获取互斥锁，确保线程安全。
  // Acquire the mutex lock to ensure thread safety.
  std::lock_guard<std::mutex> guard(graph_cache.mutex_);
  std::ostringstream ss;

  // 输出分隔线。
  // Print the separator line.
  ss << "---------------------------------" << std::endl;
  // 输出 "Graph cache:" 标题。
  // Print the "Graph cache:" title.
  ss << "Graph cache:" << std::endl;
  // 输出 "Discovered data writers:" 子标题。
  // Print the "Discovered data writers:" subtitle.
  ss << "  Discovered data writers:" << std::endl;

  // 遍历并输出 data_writers_ 中的所有数据。
  // Iterate and print all data in data_writers_.
  for (const auto &data_writer_pair : graph_cache.data_writers_) {
    ss << "    gid: '" << data_writer_pair.first << "', topic name: '"
       << data_writer_pair.second.topic_name << "', topic_type: '"
       << data_writer_pair.second.topic_type << "'" << std::endl;
  }

  // 输出 "Discovered data readers:" 子标题。
  // Print the "Discovered data readers:" subtitle.
  ss << "  Discovered data readers:" << std::endl;

  // 遍历并输出 data_readers_ 中的所有数据。
  // Iterate and print all data in data_readers_.
  for (const auto &data_reader_pair : graph_cache.data_readers_) {
    ss << "    gid: '" << data_reader_pair.first << "', topic name: '"
       << data_reader_pair.second.topic_name << "', topic_type: '"
       << data_reader_pair.second.topic_type << "'" << std::endl;
  }

  // 输出 "Discovered participants:" 子标题。
  // Print the "Discovered participants:" subtitle.
  ss << "  Discovered participants:" << std::endl;

  // 遍历并输出 participants_ 中的所有数据。
  // Iterate and print all data in participants_.
  for (const auto &item : graph_cache.participants_) {
    ss << "    gid: '" << item.first << std::endl;
    ss << "    enclave name '" << item.second.enclave << std::endl;
    ss << "    nodes:" << std::endl;

    // 遍历并输出 node_entities_info_seq 中的所有数据。
    // Iterate and print all data in node_entities_info_seq.
    for (const auto &node_info : item.second.node_entities_info_seq) {
      ss << "      namespace: '" << node_info.node_namespace << "' name: '" << node_info.node_name
         << "'" << std::endl;
      ss << "      associated data readers gids:" << std::endl;

      // 遍历并输出 reader_gid_seq 中的所有数据。
      // Iterate and print all data in reader_gid_seq.
      for (const auto &data_reader_gid : node_info.reader_gid_seq) {
        rmw_gid_t rmw_gid;
        convert_msg_to_gid(&data_reader_gid, &rmw_gid);
        ss << "        " << rmw_gid << std::endl;
      }

      ss << "      associated data writers gids:" << std::endl;

      // 遍历并输出 writer_gid_seq 中的所有数据。
      // Iterate and print all data in writer_gid_seq.
      for (const auto &data_writer_gid : node_info.writer_gid_seq) {
        rmw_gid_t rmw_gid;
        convert_msg_to_gid(&data_writer_gid, &rmw_gid);
        ss << "        " << rmw_gid << std::endl;
      }
    }
  }

  // 输出分隔线。
  // Print the separator line.
  ss << "---------------------------------" << std::endl;

  // 将 stringstream 的内容写入输出流，并返回更新后的输出流。
  // Write the content of the stringstream to the output stream and return the updated output
  // stream.
  return ostream << ss.str();
}
