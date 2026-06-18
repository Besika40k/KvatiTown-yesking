try:
    import godot.utils.map
    _GODOT_AVAILABLE = True
except ImportError:
    _GODOT_AVAILABLE = False


# ============================================================================
# ROAD NETWORK GRAPH — nodes (intersections), edges (roads)
# ============================================================================

class RoadMap:
    """
    Weighted undirected graph representing the Duckietown road network.
    Nodes = intersections, Edges = roads between them.
    In simulation: loaded from Godot; on real hardware: hardcoded fallback.
    """

    def __init__(self, scene_name="test1_actual_map_kiu"):
        if _GODOT_AVAILABLE:
            try:
                self.nodes, self.edges = godot.utils.map.get_nodes_and_edges(scene_name)
                if not self.nodes or not self.edges:
                    raise ValueError("Scene load returned empty nodes/edges")
                print(f"[RoadMap] Scene loaded: {scene_name} ({len(self.nodes)} nodes, {len(self.edges)} edges)")
                return
            except Exception as e:
                print(f"[RoadMap] Scene error: {e}, falling back to hardcoded map")
        else:
            print("[RoadMap] Godot unavailable, falling back to hardcoded map")

        self._load_hardcoded()

    def _load_hardcoded(self):
        """Hardcoded map matching the physical real robot track."""
        self.nodes = {
            1: {"id": 1, "x": 2.7, "y": 2.1},
            2: {"id": 2, "x": 0.9, "y": 4.5},
            3: {"id": 3, "x": 2.1, "y": 4.5},
        }
        self.edges = {
            "1-3-a": {"from": 1, "to": 3, "length": 13, "direction1": "E", "direction2": "E"},
            "1-2-a": {"from": 1, "to": 2, "length": 7,  "direction1": "W", "direction2": "N"},
            "1-3-b": {"from": 1, "to": 3, "length": 5,  "direction1": "S", "direction2": "N"},
            "2-3-a": {"from": 2, "to": 3, "length": 2,  "direction1": "E", "direction2": "W"},
            "2-3-b": {"from": 2, "to": 3, "length": 10, "direction1": "S", "direction2": "S"},
        }
        print(f"[RoadMap] Hardcoded map loaded ({len(self.nodes)} nodes, {len(self.edges)} edges)")

    def neighbors(self, node_id):
        """Return all roads reachable from node_id as (neighbor_id, length, edge_id) tuples."""
        result = []
        for edge_id, edge in self.edges.items():
            if edge["from"] == node_id:
                result.append((edge["to"], edge["length"], edge_id))
            elif edge["to"] == node_id:
                result.append((edge["from"], edge["length"], edge_id))
        return result

    def filter_shortest_neighbors(self, node_id):
        """Return only the shortest road to each neighbor (ignores parallel roads)."""
        best_match = {}
        for neighbor, length, edge_id in self.neighbors(node_id):
            if neighbor not in best_match or length < best_match[neighbor][0]:
                best_match[neighbor] = (length, edge_id)
        return [(neighbor, length, edge_id) for neighbor, (length, edge_id) in best_match.items()]

    def edges_between(self, from_node, to_node):
        """Return all edges between two nodes, sorted by length."""
        result = []
        for edge_id, edge in self.edges.items():
            if (edge["from"] == from_node and edge["to"] == to_node) or \
               (edge["from"] == to_node and edge["to"] == from_node):
                result.append((edge_id, edge["length"]))
        return sorted(result, key=lambda x: x[1])

    def shortest_edge(self, from_node, to_node):
        """Return the shortest (edge_id, length) between two nodes, or None."""
        edges = self.edges_between(from_node, to_node)
        return edges[0] if edges else None

    def get_node(self, node_id):
        """Return node data dict {id, x, y} for node_id."""
        return self.nodes.get(node_id)

    def get_edge(self, edge_id):
        """Return edge data dict {from, to, length} for edge_id."""
        return self.edges.get(edge_id)

    def get_all_nodes(self):
        """Return list of all node ids."""
        return list(self.nodes.keys())

    def get_all_edges(self):
        """Return list of all edge ids."""
        return list(self.edges.keys())


road_map = RoadMap()
