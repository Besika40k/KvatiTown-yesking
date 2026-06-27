import heapq
import math
import collections

from tasks.project.packages.road_map import road_map

# Compass directions in clockwise order — used for turn calculation.
_CLOCKWISE = ["N", "E", "S", "W"]


def compute_maneuver(heading, exit_dir):
    """
    Return the maneuver needed to go from current heading to exit_dir.

    heading  : current compass direction the robot is facing (N/E/S/W)
    exit_dir : compass direction the road exits the intersection (N/E/S/W)
    returns  : 'forward' | 'right' | 'turnaround' | 'left'
    """
    hi = _CLOCKWISE.index(heading)
    ei = _CLOCKWISE.index(exit_dir)
    diff = (ei - hi) % 4
    return ("forward", "right", "turnaround", "left")[diff]


def apply_maneuver(heading, maneuver):
    """
    Return the new compass heading after executing a maneuver.

    heading  : current compass direction (N/E/S/W)
    maneuver : 'forward' | 'right' | 'turnaround' | 'left'
    returns  : new compass direction (N/E/S/W)
    """
    idx = _CLOCKWISE.index(heading)
    delta = {"forward": 0, "right": 1, "turnaround": 2, "left": -1}
    return _CLOCKWISE[(idx + delta.get(maneuver, 0)) % 4]


def reconstruct_path(previous, start_state, goal_state):
    path = []
    edges = []
    current_state = goal_state

    while current_state is not None:
        path.append(current_state[0])
        if current_state == start_state:
            break
        prev_data = previous.get(current_state)
        if prev_data is None:
            break
        prev_state, edge_id = prev_data
        edges.append(edge_id)
        current_state = prev_state

    path.reverse()
    edges.reverse()
    
    if not path or path[0] != start_state[0]:
        return [], []

    return path, edges


def dijkstra(start, goal, start_heading="N", graph=road_map):
    if start not in graph.nodes:
        raise ValueError(f"Start node {start} does not exist")

    if goal not in graph.nodes:
        raise ValueError(f"Goal node {goal} does not exist")

    # max_cost is the sum of all edges in the graph. 
    # Any valid path without a U-turn will cost less than this.
    max_cost = sum(graph.get_edge(e)["length"] for e in graph.all_edges())
    U_TURN_PENALTY = max_cost + 1

    distances = collections.defaultdict(lambda: math.inf)
    previous = {}

    start_state = (start, start_heading)
    distances[start_state] = 0
    pq = [(0, start, start_heading)]
    
    goal_state = None

    while pq:
        current_distance, current_node, current_heading = heapq.heappop(pq)
        
        current_state = (current_node, current_heading)
        if current_distance > distances[current_state]:
            continue

        if current_node == goal:
            goal_state = current_state
            break

        # Use all edges (not just absolute shortest) so we can pick a longer path 
        # to avoid a U-turn penalty on the shortest path.
        for neighbor, length, edge_id in graph.neighbors(current_node):
            edge_data = graph.get_edge(edge_id)
            
            if edge_data["from"] == current_node:
                exit_dir = edge_data["direction1"]
                enter_dir = edge_data["direction2"]
            else:
                exit_dir = edge_data["direction2"]
                enter_dir = edge_data["direction1"]

            maneuver = compute_maneuver(current_heading, exit_dir)
            penalty = U_TURN_PENALTY if maneuver == "turnaround" else 0
            
            new_distance = current_distance + length + penalty
            
            # The heading when we arrive at the neighbor is the opposite of the 
            # direction the road points when leaving the neighbor.
            opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}
            next_heading = opposite[enter_dir]
            
            neighbor_state = (neighbor, next_heading)

            # Tie-breaking: heapq.heappop uses strict less than (<), so if distances 
            # are equal, the first edge processed (dict insertion order in road_map.py) wins.
            # This is incidental rather than a deliberate rule.
            if new_distance < distances[neighbor_state]:
                distances[neighbor_state] = new_distance
                previous[neighbor_state] = (current_state, edge_id)
                heapq.heappush(pq, (new_distance, neighbor, next_heading))

    path, edges = reconstruct_path(previous, start_state, goal_state)

    if not path:
        return {
            "path": [],
            "edges": [],
            "directions": [],
            "distance": math.inf,
        }

    # Compute final directions based on the exact edges we reconstructed
    directions = []
    heading = start_heading
    
    for i, edge_id in enumerate(edges):
        current_node = path[i]
        edge_data = graph.get_edge(edge_id)
        if edge_data["from"] == current_node:
            exit_dir = edge_data["direction1"]
        else:
            exit_dir = edge_data["direction2"]
            
        maneuver = compute_maneuver(heading, exit_dir)
        directions.append(maneuver)
        
        enter_dir = edge_data["direction2"] if edge_data["from"] == current_node else edge_data["direction1"]
        opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}
        heading = opposite[enter_dir]

    return {
        "path": path,
        "edges": edges,
        "directions": directions,
        "distance": distances[goal_state] if goal_state else math.inf,
    }


if __name__ == "__main__":
    print("=" * 80)
    print("Running Specific Dijkstra Pathfinding Tests")
    print("=" * 80)
    
    def run_test_case(name, start, goal, heading, expected_path=None, test_graph=road_map):
        print(f"\n[{name}]")
        print(f"Start: {start}, Goal: {goal}, Heading: {heading}")
        try:
            result = dijkstra(start, goal, heading, graph=test_graph)
            if result["path"]:
                print(f"[PASS] Path found!")
                print(f"  Path:       {' -> '.join(map(str, result['path']))}")
                print(f"  Edges:      {result['edges']}")
                print(f"  Directions: {result['directions']}")
                print(f"  Distance:   {result['distance']:.1f}")
            else:
                print(f"[FAIL] No path found")
                print(f"  Distance:   {result['distance']}")
        except Exception as e:
            print(f"[FAIL] Error: {e}")

    # --- Test 1: Dead-end fallback ---
    # Create a dummy graph where a U-turn is absolutely required
    class DummyGraph:
        def __init__(self):
            self.nodes = {4: {}, 5: {}}
            self.edges = {
                "4-5": {"from": 4, "to": 5, "length": 5, "direction1": "S", "direction2": "N"}
            }
        def get_edge(self, edge_id): return self.edges[edge_id]
        def all_edges(self): return list(self.edges.keys())
        def all_nodes(self): return list(self.nodes.keys())
        def neighbors(self, node_id):
            res = []
            for eid, e in self.edges.items():
                if e["from"] == node_id: res.append((e["to"], e["length"], eid))
                elif e["to"] == node_id: res.append((e["from"], e["length"], eid))
            return res
            
    run_test_case("Test 1: Dead-end fallback (U-Turn Required)", 4, 5, "N", test_graph=DummyGraph())
    
    # --- Test 2: Node 1 -> Node 3 (Cost 9 vs Cost 5+Penalty) ---
    run_test_case("Test 2: Route avoids U-turn via 1-3-b, uses two lefts instead", 1, 3, "N")
    
    # --- Test 3: Ordinary Route Regression (Node 2 -> Node 1) ---
    run_test_case("Test 3: Ordinary Route Regression", 2, 1, "E")

    print(f"\n{'=' * 80}\n")
