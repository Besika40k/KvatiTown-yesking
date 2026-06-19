import heapq
import math
from pickletools import optimize

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


def reconstruct_path(previous, start, goal):
    path = []
    current = goal

    while current is not None:
        path.append(current)
        if current == start:
            break
        current = previous.get(current)

    path.reverse()

    if not path or path[0] != start:
        return []

    return path


def dijkstra(start, goal, start_heading="N", graph=road_map):
    if start not in graph.nodes:
        raise ValueError(f"Start node {start} does not exist")

    if goal not in graph.nodes:
        raise ValueError(f"Goal node {goal} does not exist")

    distances = {node: math.inf for node in graph.all_nodes()}
    previous = {node: None for node in graph.all_nodes()}

    distances[start] = 0
    pq = [(0, start)]

    while pq:
        current_distance, current_node = heapq.heappop(pq)

        if current_distance > distances[current_node]:
            continue

        if current_node == goal:
            break

        for neighbor, length, edge_id in graph.all_neighbors_shortest(current_node):
            new_distance = current_distance + length

            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                previous[neighbor] = current_node
                heapq.heappush(pq, (new_distance, neighbor))

    path = reconstruct_path(previous, start, goal)

    if not path:
        return {
            "path": [],
            "edges": [],
            "directions": [],
            "distance": math.inf,
        }

    # Build edge list and compute the maneuver at each intersection.
    edges = []
    directions = []
    heading = start_heading

    for a, b in zip(path, path[1:]):
        shortest = graph.shortest_edge(a, b)
        if shortest is None:
            raise ValueError(f"No edge between {a} and {b}")
        edge_id, _ = shortest
        edges.append(edge_id)

        edge_data = graph.get_edge(edge_id)
        if edge_data["from"] == a:
            exit_dir = edge_data["direction1"]
        else:
            exit_dir = edge_data["direction2"]

        maneuver = compute_maneuver(heading, exit_dir)
        directions.append(maneuver)
        heading = exit_dir  # robot's heading after traversing this edge

    return {
        "path": path,
        "edges": edges,
        "directions": directions,
        "distance": distances[goal],
    }


if __name__ == "__main__":
    print("=" * 80)
    print("Testing ALL possible combinations of Dijkstra pathfinding")
    print("=" * 80)
    print()

    # All nodes in the graph
    nodes = [1, 2, 3]

    # All valid headings
    headings = ["N", "E", "S", "W"]

    # Test all combinations where start != goal
    combination_count = 0

    for start in nodes:
        for goal in nodes:
            if start == goal:
                continue  # Skip same start/goal

            for heading in headings:
                combination_count += 1
                print(f"\n{'─' * 80}")
                print(
                    f"Combination #{combination_count}: Start={start}, Goal={goal}, Heading={heading}"
                )
                print(f"{'─' * 80}")

                try:
                    result = dijkstra(start, goal, heading)

                    if result["path"]:
                        print(f"✓ Path found!")
                        print(f"  Path:       {' → '.join(map(str, result['path']))}")
                        print(f"  Edges:      {result['edges']}")
                        print(f"  Directions: {result['directions']}")
                        print(f"  Distance:   {result['distance']:.1f}")
                    else:
                        print(f"✗ No path found")
                        print(f"  Distance:   {result['distance']}")

                except Exception as e:
                    print(f"✗ Error: {e}")

    print(f"\n{'=' * 80}")
    print(f"Total combinations tested: {combination_count}")
    print(f"={'=' * 80}\n")
