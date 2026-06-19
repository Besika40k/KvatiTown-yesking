from typing import Dict, List, Set, Tuple

import godot_parser
from godot_parser import GDScene

import launcher.config

# Maps a grid step vector to a compass direction.
# grid_x increases East, grid_y increases South (Godot top-down convention).
_STEP_TO_COMPASS = {
    (1, 0): "E",
    (-1, 0): "W",
    (0, 1): "S",
    (0, -1): "N",
}


class Node:
    def __init__(self, id: int, cord: Tuple[int, int]):
        self.id = id
        self.x = cord[0]
        self.y = cord[1]

    def __str__(self) -> str:
        return f"Node(id={self.id}, x={self.x}, y={self.y})"

    def __repr__(self) -> str:
        return self.__str__()


def get_nodes_and_edges(godot_scene_name: str) -> Tuple[Dict, Dict]:
    """
    Extract intersections as nodes, and roads between them as edges.
    """
    file_name = godot_scene_name + ".tscn"
    path = launcher.config.MAP_DIR / file_name
    scene: GDScene = godot_parser.load(path)

    grid = {}
    with scene.use_tree() as tree:
        tiles = tree.get_node("Tiles")
        if tiles is None:
            print("Loaded scene without 'Tiles' node")
            return {}, {}
        for tile in tiles.get_children():
            name = tile.name
            if not name.startswith("Tile_"):
                continue
            parts = name.split("_")
            grid_x, grid_y = int(parts[1]), int(parts[2])

            ext = scene.find_ext_resource(id=tile.instance)
            type_str = ext.path.split("/")[-1] if ext else "unknown"

            t = tile.properties.get("transform")
            real_x = t.args[9] if t else 0.0
            real_z = t.args[11] if t else 0.0

            grid[(grid_x, grid_y)] = {"type": type_str, "rx": real_x, "rz": real_z}

            # 1. Find nodes (intersections and corners)
    intersections = {}
    nodes = {}
    node_id_counter = 1
    for (gx, gy), data in grid.items():
        if "cross" in data["type"]:
            intersections[(gx, gy)] = node_id_counter
            # Using math rounding to 2 decimal places to keep x and y clean
            x_val = round(float(data["rx"]), 2)
            y_val = round(float(data["rz"]), 2)
            nodes[node_id_counter] = {"id": node_id_counter, "x": x_val, "y": y_val}
            node_id_counter += 1

    # 2. Trace edges
    edges = {}
    edge_counters = {}
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    for (start_x, start_y), start_id in intersections.items():
        for dx, dy in directions:
            nx, ny = start_x + dx, start_y + dy
            if (nx, ny) not in grid:
                continue

            curr = (nx, ny)
            prev = (start_x, start_y)
            length = 1
            path = [(nx, ny)]

            while curr not in intersections:
                neighbors = []
                for ndx, ndy in directions:
                    nnx, nny = curr[0] + ndx, curr[1] + ndy
                    if (nnx, nny) in grid and (nnx, nny) != prev:
                        neighbors.append((nnx, nny))
                if not neighbors:
                    curr = None
                    break

                prev = curr
                curr = neighbors[0]
                path.append(curr)
                length += 1

            if curr is not None and curr in intersections:
                end_id = intersections[curr]
                # Only add edge in one direction to avoid duplicates
                if start_id < end_id:
                    pair = (start_id, end_id)
                    edge_counters[pair] = edge_counters.get(pair, 0) + 1
                    suffix = chr(ord("a") + edge_counters[pair] - 1)
                    edge_name = f"{start_id}-{end_id}-{suffix}"

                    # direction1: compass direction the road exits start_id
                    dir1 = _STEP_TO_COMPASS[(dx, dy)]
                    # direction2: compass direction the road exits end_id
                    # (opposite of the direction we arrived from)
                    last_dx = curr[0] - prev[0]
                    last_dy = curr[1] - prev[1]
                    dir2 = _STEP_TO_COMPASS[(-last_dx, -last_dy)]

                    edges[edge_name] = {
                        "from": start_id,
                        "to": end_id,
                        "length": length,
                        "direction1": dir1,
                        "direction2": dir2,
                    }

    return nodes, edges


if __name__ == "__main__":
    nodes, edges = get_nodes_and_edges("test")
    print("Nodes:", nodes)
    print("Edges:", edges)
