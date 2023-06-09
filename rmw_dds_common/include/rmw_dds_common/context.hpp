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

#ifndef RMW_DDS_COMMON__CONTEXT_HPP_
#define RMW_DDS_COMMON__CONTEXT_HPP_

#include <atomic>
#include <mutex>
#include <thread>

#include "rmw/types.h"
#include "rmw_dds_common/graph_cache.hpp"
#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common {

/**
 * @brief 基础数据结构，一个上下文在任何基于 DDS 的 RMW 实现中都需要，
 *        将一个参与者映射到多个节点。
 */
struct Context {
  /// 参与者的全局 ID，上下文使用。
  rmw_gid_t gid;
  /// 用于发布 ParticipantEntitiesInfo 发现数据的发布者。
  rmw_publisher_t* pub;
  /// 用于监听 ParticipantEntitiesInfo 发现数据的订阅者。
  rmw_subscription_t* sub;
  /// 缓存的发现数据图。
  GraphCache graph_cache;
  /// 在更新图缓存和发布图消息时应锁定的互斥体。
  std::mutex node_update_mutex;
  /// 用于监听发现数据的线程。
  std::thread listener_thread;
  /// 表示监听器线程是否正在运行。
  std::atomic_bool thread_is_running;
  /// 当完成上下文时唤醒监听器线程。
  rmw_guard_condition_t* listener_thread_gc;
  /// 当图发生变化时应触发的保护条件。
  rmw_guard_condition_t* graph_guard_condition;
};

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__CONTEXT_HPP_
