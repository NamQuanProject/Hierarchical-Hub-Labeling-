import heapq
from collections import defaultdict 





class Graph:
    def __init__(self, num_nodes):
        self.num_nodes = num_nodes
        self.edges = defaultdict(list)

    def add_edge(self, u, v, weight):
        self.edges[u].append((v, weight))
        self.edges[v].append((u, weight))

    def dijkstra(self, start):
        distances = {node: float('infinity') for node in range(self.num_nodes)}
        predecessors = {node: None for node in range(self.num_nodes)}
        distances[start] = 0
        priority_queue = [(0, start)]
        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)
            if current_distance > distances[current_node]:
                continue
            for neighbor, weight in self.edges[current_node]:
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    predecessors[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))
        return distances, predecessors

class HubLabeling:
    def __init__(self, graph):
        self.graph = graph
        self.labels = {node: [] for node in range(graph.num_nodes)}
        self.predecessors = {node: {} for node in range(graph.num_nodes)}
        self._preprocess()

    def _preprocess(self):
        for node in range(self.graph.num_nodes):
            distances, predecessors = self.graph.dijkstra(node)
            for neighbor, dist in distances.items():
                if dist < float('infinity'):
                    self.labels[node].append((neighbor, dist))
                    self.labels[neighbor].append((node, dist))
                    self.predecessors[node][neighbor] = predecessors[neighbor]
            self.labels[node] = sorted(self.labels[node], key=lambda x: x[1])

    def query(self, u, v):
        u_labels = {hub: dist for hub, dist in self.labels[u]}
        v_labels = {hub: dist for hub, dist in self.labels[v]}
        common_hubs = set(u_labels.keys()) & set(v_labels.keys())
        min_distance = float('infinity')
        best_hub = None
        for hub in common_hubs:
            distance = u_labels[hub] + v_labels[hub]
            if distance < min_distance:
                min_distance = distance
                best_hub = hub
        if min_distance == float('infinity'):
            return None, []

        # Reconstruct the path
        path_u = self._reconstruct_path(u, best_hub)
        path_v = self._reconstruct_path(v, best_hub)
        path_v.reverse()
        path = path_u + path_v[1:]

        return min_distance, path

    def _reconstruct_path(self, start, end):
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = self.predecessors[start].get(current)
        path.reverse()
        return path

# Example usage:
graph = Graph(num_nodes=5)
graph.add_edge(0, 1, 2)
graph.add_edge(0, 2, 4)
graph.add_edge(1, 2, 1)
graph.add_edge(1, 3, 7)
graph.add_edge(2, 3, 3)
graph.add_edge(3, 4, 1)

hub_labeling = HubLabeling(graph)
distance, path = hub_labeling.query(1, 4)
print(f"Distance: {distance}, Path: {path}")
