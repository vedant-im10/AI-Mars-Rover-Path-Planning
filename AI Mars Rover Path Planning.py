# Import Libraries
import math
from collections import deque, defaultdict
from queue import PriorityQueue

# To read the Input File
with open('input.txt', 'r') as file:
    lines = [line.strip() for line in file if line.strip()]
        
# Algorithm to use
algorithm = lines[0]

# Energy Limit
energy_limit = int(lines[1])

# Number of Safe Locations - Vertices
num_locations = int(lines[2])

# Parsing locations
locations = {}
start_location, goal_location = None, None
for i in range(3, 3 + num_locations):
    name, x, y, z = lines[i].split()
    locations[name] = (int(x), int(y), int(z))
    if name == "start":
        start_location = name
    elif name == "goal":
        goal_location = name

# Number of Safe Path Segments - Edges
num_segments = int(lines[3 + num_locations])
segments_index = 4 + num_locations

# Parsing edges and creating adjacency list
edges = defaultdict(list)
for edge in lines[segments_index:segments_index + num_segments]:
    a, b = edge.split()
    edges[a].append(b)
    edges[b].append(a)
graph = edges  # Use the edges dictionary directly as your graph

# For calculating if move is possible or not - Energy Limit
def can_move(current_loc, next_loc, energy_limit, momentum):
    z1 = locations[current_loc][2]
    z2 = locations[next_loc][2]
    energy_required = z2 - z1
    if energy_required <= 0:  # Downhill or flat, gain or maintain momentum
        return True, max(0, -energy_required)
    else:  # Uphill
        if energy_required <= energy_limit + momentum:
            return True, 0
    return False, 0

# For Calculating Euclidean Distance - UCS and A*
def euclidean_distance(loc1, loc2, is_3d=False):
    if is_3d:
        return math.sqrt((loc1[0] - loc2[0])**2 + (loc1[1] - loc2[1])**2 + (loc1[2] - loc2[2])**2)
    else:
        return math.sqrt((loc1[0] - loc2[0])**2 + (loc1[1] - loc2[1])**2)

# BFS Implementation
def bfs(start, goal, energy_limit, graph, locations):
    node_counter = 0  # A unique identifier for each state
    queue = deque([(node_counter, start, 0, None)])  # (Unique Node ID, Node, Momentum, Parent Node ID)
    parents = {node_counter: None}  # Mapping: Node ID -> Parent Node ID
    node_to_name = {node_counter: start}  # Mapping: Node ID -> Node Name

    while queue:
        node_id, current, momentum, parent_id = queue.popleft()
        
        if current == goal:
            return reconstruct_path_bfs(parents, node_to_name, node_id)
        
        neighbors = graph[current]
        unvisited_neighbors = []
        
        for next_node in neighbors:
            can_move_result, new_momentum = can_move(current, next_node, energy_limit, momentum)
            
            if can_move_result:
                node_counter += 1
                queue.append((node_counter, next_node, new_momentum, node_id))
                parents[node_counter] = node_id
                node_to_name[node_counter] = next_node
            else:
                unvisited_neighbors.append(next_node)
        graph[current] = unvisited_neighbors
        
    return None

# Reconstruction Path function for BFS
def reconstruct_path_bfs(parents, node_to_name, node_id):
    path = []
    while node_id is not None:
        path.append(node_to_name[node_id])
        node_id = parents[node_id]
    path.reverse()
    return path

# UCS Implementation
def ucs(start, goal, locations, graph, energy_limit):
    open_set = PriorityQueue()
    node_id_counter = 0
    open_set.put((0, start, 0, node_id_counter, None))  # (cost, current_node, momentum, node_id, parent_id)
    parents = {}  # Maps node_id to parent_id for path reconstruction
    node_to_name = {node_id_counter: start}  # Maps node_id to node_name

    while open_set:
        total_cost, current_node, momentum, node_id, parent_id = open_set.get()

        parents[node_id] = parent_id
        node_to_name[node_id] = current_node

        # Goal check moved after the visited update to ensure all paths are considered
        if current_node == goal:
            break
        
        neighbors = graph[current_node]
        unvisited_neighbors = []
        
        for neighbor in neighbors:
            can_move_result, new_momentum = can_move(current_node, neighbor, energy_limit, momentum)
            
            if can_move_result:
                # New cost calculation includes energy required for uphill movement
                new_cost = total_cost +  euclidean_distance(locations[current_node], locations[neighbor], is_3d=False)
                node_id_counter += 1
                open_set.put((new_cost, neighbor, new_momentum, node_id_counter, node_id))
            else:
                unvisited_neighbors.append(neighbor)
        graph[current_node] = unvisited_neighbors

    return reconstruct_path_ucs(parents, node_to_name, node_id)

# Reconstruction Path function for UCS
def reconstruct_path_ucs(parents, node_to_name, end_node_id):
    path = []
    while end_node_id is not None:
        path.append(node_to_name[end_node_id])
        end_node_id = parents.get(end_node_id)
    path.reverse()
    return path

# A* Search Implementation
def a_star(start, goal, locations, graph, energy_limit):
    open_set = PriorityQueue()
    node_id_counter = 0
    open_set.put((0, 0, start, 0, node_id_counter, None))  # (f_cost, g_cost, node, momentum, node_id, parent_id)
    parents = {}  # Maps node_id to parent_id for path reconstruction
    node_to_name = {node_id_counter: start}  # Maps node_id to node_name
    g_costs = {start: 0}  # Cost from start to the current node

    while open_set:
        _, g_cost, current_node, momentum, current_node_id, parent_id = open_set.get()

        if current_node == goal:
            return reconstruct_path_a_star(parents, node_to_name, current_node_id)

        neighbors = graph[current_node]
        unvisited_neighbors = []
        
        for next_node in neighbors:
            can_move_result, new_momentum = can_move(current_node, next_node, energy_limit, momentum)
            
            if can_move_result:
                tentative_g_cost = g_cost + euclidean_distance(locations[current_node], locations[next_node], is_3d=True)
                node_id_counter += 1
                parents[node_id_counter] = current_node_id
                node_to_name[node_id_counter] = next_node
                g_costs[next_node] = tentative_g_cost
                f_cost = tentative_g_cost + heuristic(next_node, goal, locations)
                open_set.put((f_cost, tentative_g_cost, next_node, new_momentum, node_id_counter, current_node_id))
            else:
                unvisited_neighbors.append(next_node)
        graph[current_node] = unvisited_neighbors
        
    return None

# Reconstruction Path function for A*
def reconstruct_path_a_star(parents, node_to_name, end_node_id):
    path = []
    while end_node_id is not None:
        path.append(node_to_name[end_node_id])
        end_node_id = parents.get(end_node_id, None)
    path.reverse()
    return path

# Heuristic function for A*
def heuristic(current, goal, locations):
    # Using Euclidean distance for 3D as heuristic
    return euclidean_distance(locations[current], locations[goal], is_3d=True)

# For getting the Output File
def write_output(path, output_file='output.txt'):
    with open(output_file, 'w') as f:
        if path:
            f.write(' '.join(path))
        else:
            f.write('FAIL')

# For choosing which Algorithm to Implement
if algorithm == "BFS":
    path = bfs('start', 'goal', energy_limit, graph, locations) # Implement BFS
elif algorithm == "UCS":
    path = ucs('start', 'goal', locations, graph, energy_limit)  # Implement UCS
elif algorithm == "A*":
    path = a_star('start', 'goal', locations, graph, energy_limit)  # Implement A*

write_output(path)