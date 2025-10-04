from expand import expand

import heapq
import itertools
from collections import deque as DQ, namedtuple as NTup

class PQueue:
    def __init__(self, data=[]):
        heapq.heapify(data)
        self.data = data

    def empty(self):
        return (self.data == [])

    def push(self, elem):
        heapq.heappush(self.data, elem)

    def pop(self):
        return heapq.heappop(self.data)

    def pushpop(self, elem):
        return heapq.heappushpop(self.data, elem)

    def replace(self, elem):
        return heapq.heapreplace(self.data, elem)


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

    if start == end:
        return [start], [start]

    queue = DQ([start])
    visited_parent_map = {start: None}

    while queue:
        node = queue.popleft()
        ##print(f"BFS node: {node}")
        visited.append(node)
        if node == end:
            while node is not None:
                path.append(node)
                node = visited_parent_map.get(node)
            path.reverse()
            return visited, path

        for child in expand(node, time_map):
            ##print(f"BFS child: {child}")
            if child not in visited_parent_map:
                visited_parent_map[child] = node
                queue.append(child)

    return [], []

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

    # start recursive search from a given local start node
    # return True iff found goal in local_start's subtree
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

    # heuristic
    def H(n):
        return dis_map[n][end]

    visited = []
    path = []

    v_set = set()
    child_parent_map = {}

    node = start
    while node and (node != end):
        v_set.add(node)
        visited.append(node)
        # only a single path is traversed, can directly append
        path.append(node) 

        children = expand(node, time_map)
        children.reverse()
        best_child = None
        for child in children:
            if (time_map[node][child]) and (child not in v_set):
                visited.append(child)
                # H(end) = 0 < H(n) for any other n
                if (not best_child) or (H(child) < H(best_child)):
                    best_child = child

        node = best_child 

    # append goal
    visited.append(node)
    path.append(node)

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

    v_set = set()
    child_parent_map = {}

    nid_cnt = itertools.count()

    # val: F(n) of node n
    # cost: G(n) of node n; tiebreaker
    # nodeid: increasing; secondary tiebreaker
    NodeData = NTup("NodeData", ["val", "cost", "nodeid", "node"])

    # heuristic
    def H(n):
        return dis_map[n][end]

    pq = PQueue()
    pq.push(NodeData(val=H(start), cost=0, nodeid=next(nid_cnt), node=start))
    while not pq.empty():
        val, cost, _, node = pq.pop()
        print(f"A* node {node}")
        visited.append(node)
        v_set.add(node)

        if node == end:
            break

        children = expand(node, time_map)
        if not children:
            continue
        #children.reverse()

        for child in children:
            edge_cost = time_map[node][child] 
            if (edge_cost and (child not in v_set)):
                child_cost = cost + edge_cost
                child_parent_map.setdefault(child, node)

                pq.push(NodeData(val=child_cost + H(child),\
                                 cost=child_cost,\
                                 nodeid=next(nid_cnt),\
                                 node=child))

    parent = end
    while parent:
        path.append(parent)
        parent = child_parent_map.get(parent)
    path.reverse()

    return visited, path


