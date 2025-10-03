from expand import expand

from collections import deque as DQ

# TO DO: Implement Breadth-first Search.
def breadth_first_search(time_map, start, end):
    """
    Breadth-first Search

    Args:
        time_map (dict): A map containing travel times between connected nodes (places or intersections), where every
        node is a dictionary key, and every value is an inner dictionary whose keys are the children of that node and
        values are travel times. Travel times are "null" for nodes that are not connected.
        start (str): The name of the node from where to start traversal
        end (str): The name of the node where to end traversal

    Returns:
        visited (list): A list of visited nodes in the order in which they were visited
        path (list): The final path found by the search algorithm
    """
    visited = []
    path = []

    child_parent_map = {}
    v_set = set()
    found = False

    child_parent_map[start] = None
    visited.append(start)
    v_idx = 0
    while not found:
        # no more nodes in fringe
        if v_idx >= len(visited):
            break

        node = visited[v_idx]
        if node not in time_map:
            # unknown node name
            break

        # visit
        v_set.add(node)

        if node == end:
            # reached goal
            found = True

        for child in expand(node, time_map):
            if child in v_set or time_map[node][child] is None:
                continue
            # add child to fringe and establish mapping to current node
            visited.append(child)
            child_parent_map.setdefault(child, node)

        # advance past current node in fringe
        v_idx += 1

    if found:
        parent = end
        while parent:
            path.append(parent)
            parent = child_parent_map.get(parent)
        path.reverse()

    return visited, path

# TO DO: Implement Depth-first Search.
def depth_first_search(time_map, start, end):
    """
    Depth-first Search

    Args:
        time_map (dict): A map containing travel times between connected nodes (places or intersections), where every
        node is a dictionary key, and every value is an inner dictionary whose keys are the children of that node and
        values are travel times. Travel times are "null" for nodes that are not connected.
        start (str): The name of the node from where to start traversal
        end (str): The name of the node where to end start traversal

    Returns:
        visited (list): A list of visited nodes in the order in which they were visited
        path (list): The final path found by the search algorithm
    """

    visited = []
    path = []

    child_parent_map = {}
    v_set = set()

    def dfs_local(local_start):
        if not local_start:
            return False

        visited.append(local_start)
        v_set.add(local_start)

        if local_start == end:
            return True
        else:
            children = expand(local_start, time_map)
            children.reverse()
            if not children:
                return False
            for child in children:
                if child not in v_set and time_map[local_start][child]:
                    child_parent_map.setdefault(child, local_start)
                    if dfs_local(child):
                        return True
            return False

    if (dfs_local(start)):
        parent = end
        while parent:
            path.append(parent)
            parent = child_parent_map.get(parent)
        path.reverse()

    return visited, path

# TO DO: Implement Greedy Best-first Search.
def best_first_search(dis_map, time_map, start, end):
    """
    Greedy Best-first Search

    Args:
        time_map (dict): A map containing travel times between connected nodes (places or intersections), where every
        node is a dictionary key, and every value is an inner dictionary whose keys are the children of that node and
        values are travel times. Travel times are "null" for nodes that are not connected.
        dis_map (dict): A map containing straight-line (Euclidean) distances between every pair of nodes (places or
        intersections, connected or not), where every node is a dictionary key, and every value is an inner dictionary whose keys are the
        children of that node and values are straight-line distances.
        start (str): The name of the node from where to start traversal
        end (str): The name of the node where to end traversal

    Returns:
        visited (list): A list of visited nodes in the order in which they were visited
        path (list): The final path found by the search algorithm
    """

    visited = []
    path = []

    return visited, path

# TO DO: Implement A* Search.
def a_star_search(dis_map, time_map, start, end):
    """
    A* Search

    Args:
        time_map (dict): A map containing travel times between connected nodes (places or intersections), where every
        node is a dictionary key, and every value is an inner dictionary whose keys are the children of that node and
        values are travel times. Travel times are "null" for nodes that are not connected.
        dis_map (dict): A map containing straight-line (Euclidean) distances between every pair of nodes (places or
        intersections, connected or not), where every node is a dictionary key, and every value is an inner dictionary whose keys are the
        children of that node and values are straight-line distances.
        start (str): The name of the node from where to start traversal
        end (str): The name of the node where to end traversal

    Returns:
        visited (list): A list of visited nodes in the order in which they were visited
        path (list): The final path found by the search algorithm
    """

    visited = []
    path = []

    return visited, path


