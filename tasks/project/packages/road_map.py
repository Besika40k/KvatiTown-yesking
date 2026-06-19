try:
    import godot.utils.map

    _GODOT_AVAILABLE = True
except ImportError:
    _GODOT_AVAILABLE = False


class RoadMap:
    """
    Weighted undirected graph representing the Duckietown road network.
    Nodes = intersections (cross tiles only), Edges = roads between them.
    In simulation: loaded from the Godot scene file.
    On real robot: falls back to hardcoded map matching the physical track.
    """

    def __init__(self, scene_name="test1_actual_map_kiu"):
        # Try loading from scene file first (works in sim and on real robot if .tscn exists)
        if _GODOT_AVAILABLE:
            try:
                self.nodes, self.edges = godot.utils.map.get_nodes_and_edges(scene_name)
                if not self.nodes or not self.edges:
                    raise ValueError("Scene load returned empty nodes/edges")
                print(
                    f"[RoadMap] Loaded from scene: {scene_name}; \n nodes: {self.nodes}; \n edges: {self.edges}"
                )
                return
            except Exception as e:
                print(
                    f"[RoadMap] Scene load failed: {e}, falling back to hardcoded map"
                )
        else:
            print("[RoadMap] godot.utils.map not available, using hardcoded map")

        # Fallback: hardcoded map for real robot
        self._load_hardcoded()

    def _load_hardcoded(self):
        """Hardcoded map matching the physical real robot track."""
        self.nodes = {
            1: {"id": 1, "x": 2.7, "y": 2.1},
            2: {"id": 2, "x": 0.9, "y": 4.5},
            3: {"id": 3, "x": 2.1, "y": 4.5},
        }
        self.edges = {
            "1-3-a": {
                "from": 1,
                "to": 3,
                "length": 13,
                "direction1": "E",
                "direction2": "E",
            },
            "1-2-a": {
                "from": 1,
                "to": 2,
                "length": 7,
                "direction1": "W",
                "direction2": "N",
            },
            "1-3-b": {
                "from": 1,
                "to": 3,
                "length": 5,
                "direction1": "S",
                "direction2": "N",
            },
            "2-3-a": {
                "from": 2,
                "to": 3,
                "length": 2,
                "direction1": "E",
                "direction2": "W",
            },
            "2-3-b": {
                "from": 2,
                "to": 3,
                "length": 10,
                "direction1": "S",
                "direction2": "S",
            },
        }
        print(
            f"[RoadMap] Hardcoded map loaded ({len(self.nodes)} nodes, {len(self.edges)} edges)"
        )

    def neighbors(self, node_id):
        """Return all roads reachable from node_id as (neighbor_id, length, edge_id) tuples."""
        result = []
        for edge_id, edge in self.edges.items():
            if edge["from"] == node_id:
                result.append((edge["to"], edge["length"], edge_id))
            elif edge["to"] == node_id:
                result.append((edge["from"], edge["length"], edge_id))
        return result

    def all_neighbors_shortest(self, node_id):
        """
        Like neighbors() but returns only the shortest road to each neighbor.
        Use this for Dijkstra — ignores longer parallel roads to the same node.
        """
        seen = {}
        for neighbor, length, edge_id in self.neighbors(node_id):
            if neighbor not in seen or length < seen[neighbor][0]:
                seen[neighbor] = (length, edge_id)
        return [
            (neighbor, length, edge_id) for neighbor, (length, edge_id) in seen.items()
        ]

    def edges_between(self, node_a, node_b):
        """Return all edges between node_a and node_b, sorted by length ascending."""
        result = []
        for edge_id, edge in self.edges.items():
            if (edge["from"] == node_a and edge["to"] == node_b) or (
                edge["from"] == node_b and edge["to"] == node_a
            ):
                result.append((edge_id, edge["length"]))
        return sorted(result, key=lambda x: x[1])

    def shortest_edge(self, node_a, node_b):
        """Return the shortest (edge_id, length) between two nodes, or None if no road exists."""
        edges = self.edges_between(node_a, node_b)
        return edges[0] if edges else None

    def get_node(self, node_id):
        """Return node data dict {id, x, y} for node_id."""
        return self.nodes.get(node_id)

    def get_edge(self, edge_id):
        """Return edge data dict {from, to, length} for edge_id."""
        return self.edges.get(edge_id)

    def all_nodes(self):
        """Return list of all node ids."""
        return list(self.nodes.keys())

    def all_edges(self):
        """Return list of all edge ids."""
        return list(self.edges.keys())


# Singleton — import this in pathfinding and navigation code
road_map = RoadMap()