import heapq
import math

from tasks.project.packages.road_map import road_map


# ============================================================================
# PATHFINDING — Dijkstra on road graph with compass heading
# ============================================================================

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


def reconstruct_path(predecessor_map, start_node, goal_node):
    """Walk predecessor_map from goal back to start to build the path."""
    path = []
    current = goal_node

    while current is not None:
        path.append(current)
        if current == start_node:
            break
        current = predecessor_map.get(current)

    path.reverse()

    if not path or path[0] != start_node:
        return []

    return path


def dijkstra(start_node, goal_node, start_heading="N", road_graph=road_map):
    if start_node not in road_graph.nodes:
        raise ValueError(f"Start node {start_node} does not exist")

    if goal_node not in road_graph.nodes:
        raise ValueError(f"Goal node {goal_node} does not exist")

    distance_map = {node: math.inf for node in road_graph.get_all_nodes()}
    predecessor_map = {node: None for node in road_graph.get_all_nodes()}

    distance_map[start_node] = 0
    priority_queue = [(0, start_node)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > distance_map[current_node]:
            continue

        if current_node == goal_node:
            break

        for neighbor, length, edge_id in road_graph.filter_shortest_neighbors(current_node):
            new_distance = current_distance + length

            if new_distance < distance_map[neighbor]:
                distance_map[neighbor] = new_distance
                predecessor_map[neighbor] = current_node
                heapq.heappush(priority_queue, (new_distance, neighbor))

    path = reconstruct_path(predecessor_map, start_node, goal_node)

    if not path:
        return {
            "path": [],
            "edges": [],
            "directions": [],
            "distance": math.inf,
        }

    edge_ids = []
    directions = []
    heading = start_heading

    for from_node, to_node in zip(path, path[1:]):
        shortest_edge = road_graph.shortest_edge(from_node, to_node)
        if shortest_edge is None:
            raise ValueError(f"No edge between {from_node} and {to_node}")
        edge_id, _length = shortest_edge
        edge_ids.append(edge_id)

        edge_data = road_graph.get_edge(edge_id)
        if edge_data["from"] == from_node:
            exit_dir = edge_data["direction1"]
        else:
            exit_dir = edge_data["direction2"]

        maneuver = compute_maneuver(heading, exit_dir)
        directions.append(maneuver)
        heading = exit_dir

    return {
        "path": path,
        "edges": edge_ids,
        "directions": directions,
        "distance": distance_map[goal_node],
    }


if __name__ == "__main__":
    print("=" * 80)
    print("Testing ALL possible combinations of Dijkstra pathfinding")
    print("=" * 80)

    nodes = [1, 2, 3]
    headings = ["N", "E", "S", "W"]
    combination_count = 0

    for start in nodes:
        for goal in nodes:
            if start == goal:
                continue
            for heading in headings:
                combination_count += 1
                print(f"\n{'─' * 80}")
                print(f"Combo #{combination_count}: {start} → {goal}, heading={heading}")
                print(f"{'─' * 80}")

                try:
                    result = dijkstra(start, goal, heading)
                    if result["path"]:
                        print(f"  Path:       {' → '.join(map(str, result['path']))}")
                        print(f"  Edges:      {result['edges']}")
                        print(f"  Directions: {result['directions']}")
                        print(f"  Distance:   {result['distance']:.1f}")
                    else:
                        print(f"  No path found")
                except Exception as e:
                    print(f"  Error: {e}")

    print(f"\n{'=' * 80}")
    print(f"Total combinations tested: {combination_count}")
    print(f"{'=' * 80}\n")
