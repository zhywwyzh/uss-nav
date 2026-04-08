import collections
from typing import List, Union

def find_shortest_path(adjacency_list: dict, start_location: str, end_location: str) -> Union[List[str], None]:
    """
    根据给定的邻接表（图），查找从起点到终点的最短路径。

    Args:
        adjacency_list (dict): 一个字典，表示图的邻接表。
                               键是节点（字符串），值是与该节点直接相连的节点列表（字符串列表）。
        start_location (str): 起始节点的名称。
        end_location (str): 终点节点的名称。

    Returns:
        list[str] | None: 一个包含最短路径上所有节点的列表（包括起点和终点）。
                          如果找不到路径，则返回 None。
    """
    # 检查起点和终点是否存在于图中
    if start_location not in adjacency_list or end_location not in adjacency_list:
        print("错误：起点或终点不在邻接表中。")
        return None

    # 如果起点和终点相同，路径就是它本身
    if start_location == end_location:
        return [start_location]

    # 使用双端队列（deque）来实现广度优先搜索（BFS）
    # 队列中存储的是到达当前节点的路径
    queue = collections.deque([[start_location]])

    # 使用集合（set）来记录已经访问过的节点，以避免循环
    visited = {start_location}

    while queue:
        # 取出队列中的第一条路径
        path = queue.popleft()
        # 获取该路径的最后一个节点
        current_node = path[-1]

        # 遍历当前节点的所有邻居
        for neighbor in adjacency_list.get(current_node, []):
            if neighbor not in visited:
                # 将邻居标记为已访问
                visited.add(neighbor)
                # 创建一条新的路径
                new_path = list(path)
                new_path.append(neighbor)

                # 如果邻居是终点，我们找到了最短路径
                if neighbor == end_location:
                    return new_path

                # 否则，将新路径加入队列
                queue.append(new_path)

    # 如果队列为空仍然没有找到路径，说明起点和终点不连通
    return None

# --- 使用您提供的示例进行测试 ---
# layout_graph = {
#     "Corridor_A": ["Living_Room", "Office_A", "Storage_Room", "Office_B"],
#     "Living_Room": ["Corridor_A", "Toilet/Bathroom", "Corridor_B"],
#     "Corridor_B": ["Living_Room", "Garage", "Kitchen", "Bedroom"],
#     "Office_A": ["Corridor_A"],
#     "Storage_Room": ["Corridor_A"],
#     "Toilet/Bathroom": ["Living_Room"],
#     "Office_B": ["Corridor_A"],
#     "Garage": ["Corridor_B"],
#     "Kitchen": ["Corridor_B"],
#     "Bedroom": ["Corridor_B"]
# }

# # 示例1：从 Office_A 到 Kitchen
# start = "Corridor_B"
# end = "Corridor_A"
# shortest_path = find_shortest_path(layout_graph, start, end)

# if shortest_path:
#     print(f"从 {start} 到 {end} 的最短路径是: {' -> '.join(shortest_path)}")
# else:
#     print(f"找不到从 {start} 到 {end} 的路径。")

# 示例输出：
# 从 Office_A 到 Kitchen 的最短路径是: Office_A -> Corridor_A -> Living_Room -> Corridor_B -> Kitchen
