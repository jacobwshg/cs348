from expand import expand

import heapq
import itertools
from collections import deque as DQ, namedtuple as NTup

class PQueue:
    def __init__(self, data=None):
        self.data = data if data else []
        heapq.heapify(self.data)

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
    # record visited status and predecessor at once
    vis_pred_map = { start: None }

    while queue:
        node = queue.popleft()
        visited.append(node)
        if node == end:
            while node:
                path.append(node)
                node = vis_pred_map.get(node)
            path.reverse()
            return visited, path

        succs = expand(node, time_map)
        if not succs:
            continue
        for succ in succs:
            if succ not in vis_pred_map:
                vis_pred_map[succ] = node
                queue.append(succ)

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

    vis_pred_map = { start: None }

    # start recursive search from a given local start node
    # return True iff found goal in local_start's subtree
    def dfs_local(local_start):
        if not local_start:
            return False

        visited.append(local_start)

        if local_start == end:
            return True
        else:
            succs = expand(local_start, time_map)
            if not succs:
                return False
            # simulate stack
            succs.reverse()
            for succ in succs:
                if succ not in vis_pred_map:
                    # proceed to unvisited successor
                    vis_pred_map[succ] = local_start
                    if dfs_local(succ):
                        return True
            return False

    if (dfs_local(start)):
        # path exists
        node = end
        while node:
            path.append(node)
            node = vis_pred_map.get(node)
        path.reverse()
        return visited, path

    return [], []

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

    # store visited status and predecessor at once
    vis_pred_map = { start: None }

    nid_cnt = itertools.count()

    NodeData = NTup("NodeData", ["G", "nodeid", "node"])

    pq = PQueue()
    pq.push( NodeData(node=start, nodeid=next(nid_cnt), G=H(start)) )

    while pq:
        G, _, node = pq.pop()
        ##print(f"GBFS node: {node}")
        visited.append(node)

        if node == end:
            while node:
                path.append(node)
                node = vis_pred_map.get(node)
            path.reverse()
            return visited, path

        succs = expand(node, time_map)
        if not succs:
            continue
        succs.reverse()

        for succ in succs:
            old_pred = vis_pred_map.get(succ)
            if (succ not in vis_pred_map) :
                vis_pred_map[succ] = node
                pq.push( NodeData(G=H(succ), nodeid=next(nid_cnt), node=succ) )
    return [], []

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

    vis_pred_map = { start: None }
    G_map = { start: 0 }

    nid_cnt = itertools.count()

    # nodeid: increasing; secondary tiebreaker
    NodeData = NTup("NodeData", ["F", "G", "nodeid", "node"])

    # heuristic
    def H(n):
        return dis_map[n][end]

    pq = PQueue()
    pq.push(NodeData(F=H(start), G=0, nodeid=next(nid_cnt), node=start))

    while not pq.empty():
        F, G, _, node = pq.pop()
        visited.append(node)

        if node == end:
            while node:
                path.append(node)
                if node == start:
                    break;
                node = vis_pred_map.get(node)
            path.reverse()
            return visited, path

        G_old = G_map.get(node)
        if (G_old) and G > G_old:
            continue

        succs = expand(node, time_map)
        if not succs:
            continue
        for succ in succs:
            G_edge = time_map[node][succ]
            G_succ = G + G_edge

            G_succ_old = G_map.get(succ)
            if (not G_succ_old) or G_succ < G_succ_old:
                G_map[succ] = G_succ
                vis_pred_map[succ] = node

                pq.push(NodeData(F=G_succ + H(succ),\
                                 G=G_succ,\
                                 nodeid=next(nid_cnt),\
                                 node=succ))

    return [], []


