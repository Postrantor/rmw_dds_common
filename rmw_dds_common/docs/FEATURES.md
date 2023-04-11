# rmw_dds_common Features

This package includes:

- A generic [`GraphCache`](rmw_dds_common/include/rmw_dds_common/graph_cache.hpp) to track DDS entities such as participants and data readers and writers, plus ROS nodes as an additional abstraction for any DDS based `rmw` implementation to use
 - 一个通用[`graphCache`]（rmw_dds_common/include/rmw_dds_common/graph_cache.hpp），以跟踪DDS实体，例如参与者，数据阅读器和作家，以及ROS节点，以及ROS节点作为基于DDS的其他用于使用``ROS''
- Common messages to communicate [ROS nodes discovery information](https://github.com/ros2/design/pull/250):
 - 通信传达的常见消息[ROS节点发现信息]（https://github.com/ros2/design/pull/250）：
  - [`rmw_dds_common/msg/Gid`](rmw_dds_common/msg/Gid.msg)
 -  [`rmw_dds_common/msg/gid`]（rmw_dds_common/msg/gid.msg）
  - [`rmw_dds_common/msg/NodeEntitiesInfo`](rmw_dds_common/msg/NodeEntitiesInfo.msg)
 -  [`rmw_dds_common/msg/nodeentitiesInfo`]（rmw_dds_common/msg/nodeentitiesinfo.msg）
  - [`rmw_dds_common/msg/ParticipantEntitiesInfo`](rmw_dds_common/msg/ParticipantEntitiesInfo.msg)
 -  [`rmw_dds_common/msg/garationentitiesInfo`]
- Some useful data types and utilities:
 - 一些有用的数据类型和实用程序：
  - A generic [`Context`](rmw_dds_common/include/rmw_dds_common/context.hpp) type to withhold most state needed to implement [ROS nodes discovery](https://github.com/ros2/design/pull/250)
 - 一个通用[`context`]（rmw_dds_common/include/rmw_dds_common/context.hpp）类型，以扣除实现[ROS Nodes Discovery]所需的大多数状态（https://github.com/ROS/ROS2/ROS2/ros2/design/pull/pull/250）
  - [Comparison utilities and some C++ operator overloads](rmw_dds_common/include/rmw_dds_common/gid_utils.hpp) for `rmw_gid_t` instances
 -  [比较实用程序和某些C ++操作员过载]（rmw_dds_common/include/rmw_dds_common/gid_utils.hpp）
  - [Conversion utilities](rmw_dds_common/include/rmw_dds_common/gid_utils.hpp) between `rmw_dds_common/msg/Gid` messages and `rmw_gid_t` instances
 -  [转换实用程序]（rmw_dds_common/includs/rmw_dds_common/gid_utils.hpp）`rmw_dds_common/msg/gid`messe
  - A function for checking the compatibility of two QoS profiles, [`qos_profile_check_compatible`](rmw_dds_common/include/rmw_dds_common/qos.hpp)
 - 检查两个QoS配置文件的兼容性的功能，[qos_profile_check_compatible`]（rmw_dds_common/include/rmw_dds_common/qos.hpp）
