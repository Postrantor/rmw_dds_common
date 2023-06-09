---
Crafted by [Genie](https://marketplace.visualstudio.com/items?itemName=genieai.chatgpt-vscode)
---

## You

下面这段代码是在 ros2 项目中 rmw_dds_common 相关的代码，请联系所有函数之间可能的相互联系，梳理、解释这份文件的功能、含义，以及函数之间的功能调用关系（以中文语言输出，不限制字数）

## Trantor

这份代码是 ROS2 项目中的 `rmw_dds_common` 模块，主要用于管理图缓存（Graph Cache）。图缓存负责存储和管理与 DDS 实体（如参与者、节点、读写器和写入器）相关的信息。它提供了一系列函数来添加、删除和更新实体以及查询实体信息。

以下是各个函数的功能和调用关系：

1. `set_on_change_callback`：设置当图缓存发生变化时触发的回调函数。
2. `clear_on_change_callback`：清除已设置的回调函数。
3. `add_writer`：向图缓存中添加一个写入器实体。
4. `add_reader`：向图缓存中添加一个读取器实体。
5. `add_entity`：向图缓存中添加一个实体（读取器或写入器）。
6. `remove_writer`：从图缓存中移除一个写入器实体。
7. `remove_reader`：从图缓存中移除一个读取器实体。
8. `remove_entity`：从图缓存中移除一个实体（读取器或写入器）。
9. `add_participant`：向图缓存中添加一个参与者实体。
10. `remove_participant`：从图缓存中移除一个参与者实体。
11. `update_participant_entities`：根据给定的 `ParticipantEntitiesInfo` 消息更新参与者实体。
12. `add_node`：向图缓存中添加一个节点实体。
13. `remove_node`：从图缓存中移除一个节点实体。
14. `associate_writer`：将写入器实体与指定的节点关联。
15. `dissociate_writer`：将写入器实体与指定的节点解除关联。
16. `associate_reader`：将读取器实体与指定的节点关联。
17. `dissociate_reader`：将读取器实体与指定的节点解除关联。
18. `get_writer_count`：获取给定主题名称的写入器数量。
19. `get_reader_count`：获取给定主题名称的读取器数量。
20. `get_writers_info_by_topic`：根据主题名称获取写入器信息。
21. `get_readers_info_by_topic`：根据主题名称获取读取器信息。
22. `get_names_and_types`：获取所有实体的名称和类型。
23. `get_writer_names_and_types_by_node`：根据节点获取写入器的名称和类型。
24. `get_reader_names_and_types_by_node`：根据节点获取读取器的名称和类型。
25. `get_number_of_nodes`：获取图缓存中的节点数量。
26. `get_node_names`：获取图缓存中所有节点的名称、命名空间和围场。

此外，还定义了两个结构体：

- `ParticipantInfo`：包含参与者的节点实体信息序列和围场信息。
- `EntityInfo`：包含实体（读取器或写入器）的主题名称、主题类型、主题类型哈希、参与者 GID 和 QoS 配置。

最后，还有一个输出运算符重载函数 `operator<<`，用于将 `GraphCache` 对象的信息输出到给定的输出流中。

## You

请进一步梳理这些函数之间功能的调用关系

## Trantor

以下是这些函数之间的功能调用关系：

按功能划分，这些函数可以分为以下几类，并梳理它们之间的关系：

1. 回调函数管理：

   - `set_on_change_callback`：设置回调函数。
   - `clear_on_change_callback`：清除回调函数。

2. 实体（DataWriter 和 DataReader）管理：

   - `add_writer`：添加 DataWriter。
   - `add_reader`：添加 DataReader。
   - `add_entity`：添加实体（DataWriter 或 DataReader），由 `is_reader` 参数决定。
   - `remove_writer`：移除 DataWriter。
   - `remove_reader`：移除 DataReader。
   - `remove_entity`：移除实体（DataWriter 或 DataReader），由 `is_reader` 参数决定。

3. 参与者管理：

   - `add_participant`：添加参与者。
   - `remove_participant`：移除参与者。
   - `update_participant_entities`：更新参与者及其关联的节点实体信息。

4. 节点管理：

   - `add_node`：将节点添加到指定的参与者中。
   - `remove_node`：从指定的参与者中移除节点。

5. 关联和解除关联实体与节点：

   - `associate_writer`：关联 DataWriter 到指定的参与者和节点。
   - `dissociate_writer`：解除 DataWriter 与指定的参与者和节点的关联。
   - `associate_reader`：关联 DataReader 到指定的参与者和节点。
   - `dissociate_reader`：解除 DataReader 与指定的参与者和节点的关联。

6. 查询功能：
   - `get_writer_count`：获取指定主题的 DataWriter 数量。
   - `get_reader_count`：获取指定主题的 DataReader 数量。
   - `get_writers_info_by_topic`：根据主题名称获取所有 DataWriter 的信息。
   - `get_readers_info_by_topic`：根据主题名称获取所有 DataReader 的信息。
   - `get_names_and_types`：获取 GraphCache 中所有实体的主题名称和类型。
   - `get_writer_names_and_types_by_node`：获取指定节点上的所有 DataWriter 的主题名称和类型。
   - `get_reader_names_and_types_by_node`：获取指定节点上的所有 DataReader 的主题名称和类型。
   - `get_number_of_nodes`：获取 GraphCache 中节点的数量。
   - `get_node_names`：获取 GraphCache 中所有节点的名称、命名空间和围场。

这些函数之间的关系主要体现在它们对 GraphCache 对象的操作。例如，添加、删除、更新实体或参与者时，可能会触发回调函数；查询功能则用于获取实体、参与者和节点之间的关系以及它们的属性等信息。

1. `add_writer` 和 `add_reader` 都会调用 `add_entity` 函数来向图缓存中添加实体（读取器或写入器）。
2. `remove_writer` 和 `remove_reader` 都会调用 `remove_entity` 函数来从图缓存中移除实体（读取器或写入器）。
3. `associate_writer` 和 `dissociate_writer` 会影响 `get_writer_names_and_types_by_node` 的结果，因为它们会改变与节点关联的写入器实体。
4. `associate_reader` 和 `dissociate_reader` 会影响 `get_reader_names_and_types_by_node` 的结果，因为它们会改变与节点关联的读取器实体。
5. `add_node` 和 `remove_node` 会影响 `get_number_of_nodes` 和 `get_node_names` 的结果，因为它们会改变图缓存中的节点数量和信息。
6. `add_writer`, `add_reader`, `remove_writer`, `remove_reader`, `add_participant`, `remove_participant`, `update_participant_entities`, `add_node`, `remove_node`, `associate_writer`, `dissociate_writer`, `associate_reader`, `dissociate_reader` 这些函数在执行操作后都可能触发 `on_change_callback_` 回调函数，表示图缓存发生了变化。

其他查询类函数如 `get_writer_count`, `get_reader_count`, `get_writers_info_by_topic`, `get_readers_info_by_topic`, `get_names_and_types` 等，它们之间没有直接的调用关系，但它们的结果会受到上述修改图缓存的函数的影响。
