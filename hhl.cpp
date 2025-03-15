#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <cmath>
#include <tuple>
#include <list>
#include <map>
#include <unordered_set>
#include <iostream>

class Shortcut {
public:
    int from_vertex;
    int to_vertex;
    double distance;
    Shortcut(int from_vertex, int to_vertex, double distance) 
        : from_vertex(from_vertex), to_vertex(to_vertex), distance(distance) {}
};


class HierarchicalHubLabeling {
public:
    int n;
    std::vector<std::vector<double>> bidirect;
    std::vector<bool> visited;
    std::vector<double> rank;
    std::vector<int> level;
    std::unordered_map<int, std::unordered_map<std::string, std::vector<std::pair<int, double>>>> neighbors;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> q;
    int count;
    std::vector<int> processing;
    std::vector<int> vertices; 
    std::unordered_map<int, std::vector<Shortcut>> edges; 

    std::unordered_map<int, std::vector<std::pair<int, double>>> labelsForward;
    std::unordered_map<int, std::vector<std::pair<int, double>>> labelsBackward;

    HierarchicalHubLabeling(const std::vector<int>& vertices, const std::unordered_map<int, std::vector<Shortcut>>& edges, int n)
        : n(n),
          bidirect(2, std::vector<double>(n, std::numeric_limits<double>::infinity())),
          visited(n, false),
          rank(n, std::numeric_limits<double>::infinity()),
          level(n, 0),
          count(0),
          vertices(vertices),
          edges(edges)
    {
        buildNeighborsAndCosts(); 
    }

    void buildNeighborsAndCosts() {
        for (const auto& vertex : vertices) {
            neighbors[vertex] = {{"incoming", {}}, {"outgoing", {}}};
        }
        for (const auto& [vertex, edgesList] : edges) {
            for (const auto& edge : edgesList) {
                neighbors[edge.from_vertex]["outgoing"].emplace_back(edge.to_vertex, edge.distance);
                neighbors[edge.to_vertex]["incoming"].emplace_back(edge.from_vertex, edge.distance);
            }
        }
        std::cout << "Neighbors and costs built" << std::endl;
    } 

    double max(double a, double b) {
        return (a > b) ? a : b;
    }

    void preImportance() {
        for (const auto& vertex : vertices) {
            int incoming_size = neighbors[vertex]["incoming"].size();
            int outgoing_size = neighbors[vertex]["outgoing"].size();

            double importance = incoming_size * outgoing_size - incoming_size - outgoing_size;
            q.emplace(importance, vertex);
            std::cout << importance << std::endl;
        }
        
        std::cout << "Importance calculated" << std::endl;
    }
 

    double computeNeighborsLevel(int vertex) {
        int num = 0;
        double level1 = 0, level2 = 0;
        for (const auto& [prev, dist] : neighbors[vertex]["incoming"]) {
            if (rank[prev] < rank[vertex]) {
                num++;
                level1 = max(level1, level[prev] + 1);
            }
        }
        for (const auto& [next_, dist] : neighbors[vertex]["outgoing"]) {
            if (rank[next_] < rank[vertex]) {
                num++;
                level2 = max(level2, level[next_] + 1);
            }
        }
        return num + (level1 + level2) / 2;
    }

    void recalibrateNeighbors(int vertex) {
        for (const auto& [prev, dist] : neighbors[vertex]["incoming"]) {
            level[prev] = max(level[prev], level[vertex] + 1);
        }
        for (const auto& [next_, dist] : neighbors[vertex]["outgoing"]) {
            level[next_] = max(level[next_], level[vertex] + 1);
        }
    }

    void addShortcut(int from_stop, int to_stop, double dist) {
        bool updated = false;

        for (auto& [prev, weight] : neighbors[to_stop]["incoming"]) {
            if (prev == from_stop) {
                weight = std::min(weight, dist);
                updated = true;
                break;
            }
        }
        if (!updated) {
            neighbors[to_stop]["incoming"].emplace_back(from_stop, dist);
            count++;
        }

        updated = false;

        for (auto& [next_, weight] : neighbors[from_stop]["outgoing"]) {
            if (next_ == to_stop) {
                weight = std::min(weight, dist);
                updated = true;
                break;
            }
        }
        if (!updated) {
            neighbors[from_stop]["outgoing"].emplace_back(to_stop, dist);
            count++;
        }
    }

    void preProcess() {
        int rank_counter = 1;
        preImportance();

        while (!q.empty()) {
            
            int vertex = q.top().second;
            q.pop();

            auto [importance, shortcuts] = calculateShortcut(vertex, true);
            std::cout << "Number of shortcuts: " << shortcuts.size() << std::endl;
            for (auto shortcut: shortcuts) {
                std::cout << shortcut.from_vertex << " " << shortcut.to_vertex << " " << shortcut.distance << std::endl;
            }
            
            if (!q.empty() && importance > q.top().first) {
                q.emplace(importance, vertex); 
                continue;
            }
            else {
                std::cout << shortcuts.size() << std::endl;
                for (const auto& shortcut : shortcuts) {
                    addShortcut(shortcut.from_vertex, shortcut.to_vertex, shortcut.distance);
                }

                rank[vertex] = rank_counter++;
                recalibrateNeighbors(vertex);
            }
        }
    }


    std::unordered_map<int, double> witnessPath(int source, int contract, double limit) {
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> heap;
        heap.emplace(0, source);
        std::unordered_map<int, double> dist_dict;
        for (const auto& [vertex, _] : neighbors) {
            dist_dict[vertex] = std::numeric_limits<double>::infinity();
        }
        dist_dict[source] = 0;

        while (!heap.empty()) {
            auto [dist, vertex] = heap.top();
            heap.pop();
            if (dist >= limit) {
                return dist_dict;
            }

            for (const auto& [to_vertex, weight] : neighbors[vertex]["outgoing"]) {
                if (rank[to_vertex] < rank[vertex]) {
                    continue;
                }
                if (contract == to_vertex) {
                    continue;
                }
                if (dist_dict[to_vertex] > dist + weight) {
                    dist_dict[to_vertex] = dist + weight;
                    heap.emplace(dist_dict[to_vertex], to_vertex);
                }
            }
        }
        return dist_dict;
    }

    std::pair<double, std::vector<Shortcut>> calculateShortcut(int vertex, bool adding_shortcuts = true) {
        int shortcut_count = 0;
        std::unordered_set<int> shortcut_cover;
        std::vector<Shortcut> shortcuts;
        double maxOutgoing = 0;
        double minOutgoing = std::numeric_limits<double>::infinity();

        if (neighbors[vertex].find("outgoing") != neighbors[vertex].end() && neighbors[vertex]["outgoing"].size() != 0) {
            for (const auto& [_, dist] : neighbors[vertex]["outgoing"]) {
                maxOutgoing = max(maxOutgoing, dist);
                minOutgoing = std::min(minOutgoing, dist);
            }
        } else {
            minOutgoing = 0; 
        }
        
        if (neighbors[vertex].find("incoming") != neighbors[vertex].end()) {
            for (const auto& [prev, dist1] : neighbors[vertex]["incoming"]) {
                if (rank[prev] > rank[vertex]) {
                    continue;
                }
                std::cout << "Maxcomming:" << maxOutgoing << " " << "Mincomming:" << minOutgoing << " " << "Dist1: " << dist1 << std::endl;
                auto dist_dict = witnessPath(prev, vertex, dist1 + maxOutgoing - minOutgoing);
                for (auto key : dist_dict) {
                    
                    std::cout << "Dictionary " << " " << prev << " " << key.first << " " << key.second << " " << dist1 + maxOutgoing - minOutgoing << std::endl;
                }
                if (neighbors[vertex].find("outgoing") != neighbors[vertex].end() && neighbors[vertex]["outgoing"].size() != 0) {
                    for (const auto& [next_, dist2] : neighbors[vertex]["outgoing"]) {
                        if (rank[next_] < rank[vertex] || rank[prev] > rank[next_]) {
                            continue;
                        }

                        if (dist1 + dist2 < dist_dict[next_]) {
                            shortcut_count++;
                            shortcut_cover.insert(prev);
                            shortcut_cover.insert(next_);
                            if (adding_shortcuts) {
                                shortcuts.emplace_back(prev, next_, dist1 + dist2);
                            }
                        }
                    }
                }
            }
        }


        
        double edge_diff = shortcut_count - double(neighbors[vertex]["incoming"].size()) - double(neighbors[vertex]["outgoing"].size());
        double importance = edge_diff + shortcut_cover.size() + computeNeighborsLevel(vertex);

        return {importance, shortcuts};
    }


    void clearProcess() {
        bidirect = std::vector<std::vector<double>>(2, std::vector<double>(n, std::numeric_limits<double>::infinity()));
        visited = std::vector<bool>(n, false);
        processing.clear();
    }

    void markVisited(int vertex) {
        if (!visited[vertex]) {
            visited[vertex] = true;
            processing.push_back(vertex);
        }
    }

    void removeEdge() {
        int incoming_edges = 0, outgoing_edges = 0, delete_edges = 0;
        for (const auto& [stop, _] : neighbors) {
            for (auto it = neighbors[stop]["incoming"].begin(); it != neighbors[stop]["incoming"].end();) {
                if (rank[it->first] < rank[stop]) {
                    it = neighbors[stop]["incoming"].erase(it);
                    delete_edges++;
                } else {
                    ++it;
                }
            }
            for (auto it = neighbors[stop]["outgoing"].begin(); it != neighbors[stop]["outgoing"].end();) {
                if (rank[it->first] < rank[stop]) {
                    it = neighbors[stop]["outgoing"].erase(it);
                    delete_edges++;
                } else {
                    ++it;
                }
            }
            incoming_edges += neighbors[stop]["incoming"].size();
            outgoing_edges += neighbors[stop]["outgoing"].size();
        }
        std::cout << "Incoming edges: " << incoming_edges << ", Outgoing edges: " << outgoing_edges << std::endl;
        std::cout << "Total shortcuts added: " << count << std::endl;
        std::cout << "Total edges deleted: " << delete_edges << std::endl;
    }

    std::map<int, std::pair<double, std::list<int>>> dijkstra(int source) {
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<int> pred(n, -1);
        dist[source] = 0;

        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
        pq.emplace(0, source);

        while (!pq.empty()) {
            int u = pq.top().second;
            double currentDist = pq.top().first;
            pq.pop();

            if (currentDist > dist[u]) continue;

            for (const auto& [v, weight] : neighbors[u]["outgoing"]) {
                if (currentDist + weight < dist[v]) {
                    dist[v] = currentDist + weight;
                    pred[v] = u;
                    pq.emplace(dist[v], v);
                }
            }
        }

        std::map<int, std::pair<double, std::list<int>>> results;
        for (int i = 0; i < n; ++i) {
            std::list<int> path;
            int node = i;
            while (node != -1) {
                path.push_front(node);
                node = pred[node];
            }
            results[i] = {dist[i], path};
        }

        return results;
    }

    void buildLabels() {
        labelsForward.clear();
        labelsBackward.clear();
        
        for (int vertex : vertices) {
            std::map<int, std::pair<double, std::list<int>>> fwd = dijkstra(vertex);

            if (labelsForward.find(vertex) == labelsForward.end()) {
                labelsForward[vertex] = std::vector<std::pair<int, double>>();
            }
            
            if (labelsBackward.find(vertex) == labelsBackward.end()) {
                labelsBackward[vertex] = std::vector<std::pair<int, double>>();
            }

            for (const auto& [target, pair] : fwd) {
                double distance = pair.first;
                auto paths = pair.second;
                if (distance != std::numeric_limits<double>::infinity()) {
                    int highest_rank_vertex = vertex;
                    for (auto path : paths) {
                        if (rank[path] > rank[vertex]) {
                            highest_rank_vertex = path;
                        }
                    }
                    
                    
                    bool forwardFound = false;
                    for (const auto& entry : labelsForward[vertex]) {
                        if (entry.first == highest_rank_vertex) {
                            forwardFound = true;
                            break;
                        }
                    }
                    if (!forwardFound) {
                        labelsForward[vertex].push_back({highest_rank_vertex, fwd[highest_rank_vertex].first});
                    }
                    bool backwardFound = false;
                    for (const auto& entry : labelsBackward[target]) {
                        if (entry.first == highest_rank_vertex) {
                            backwardFound = true;
                            break;
                        }
                    }
                    if (!backwardFound) {
                        labelsBackward[target].push_back({highest_rank_vertex, fwd[target].first - fwd[highest_rank_vertex].first});
                    }
                    

                }
            }
        }
    }
   
    double query(int source_vertex, int target_vertex) {
        // Retrieve labels
        auto source_labels = labelsForward[source_vertex];
        auto target_labels = labelsBackward[target_vertex];
        std::vector<double> fw_dis;
        std::vector<double> bw_dis;
        // Collect vertices from source_labels into a set
        std::unordered_set<int> common_vertices;
        for (const auto& [joint, dis] : source_labels) {
            common_vertices.insert(joint);

        }

        // Find common vertices and determine the highest rank
        int highest_rank_vertex = -1;
        int highest_rank = -1;

        for (const auto& [joint, dis] : target_labels) {
            if (common_vertices.find(joint) != common_vertices.end()) {
                // Vertex is common in both forward and backward labels
                if (rank[joint] > highest_rank) {
                    highest_rank = rank[joint];
                    highest_rank_vertex = joint;
                    
                }
            }
        }

        if (highest_rank_vertex == -1) {
            std::map<int, std::pair<double, std::list<int>>> fwd = dijkstra(source_vertex);
            auto result = fwd[target_vertex].first;
            std::cerr << "No common vertices found between source " << source_vertex << " and target " << target_vertex << std::endl;
            return result;
        }

        // Create maps for easy lookup of distances
        std::unordered_map<int, double> source_distance_map;
        for (const auto& [joint, dis] : source_labels) {
            source_distance_map[joint] = dis;
        }

        std::unordered_map<int, double> target_distance_map;
        for (const auto& [joint, dis] : target_labels) {
            target_distance_map[joint] = dis;
        }

        // Calculate the result based on the highest_rank_vertex
        double result = source_distance_map[highest_rank_vertex] + target_distance_map[highest_rank_vertex];
        return result;
    }
};




int main() {
    // Example usage
    std::vector<int> vertices = {0, 1, 2, 3, 4, 5};
    std::unordered_map<int, std::string> vertex_labels = {
        {0, "S"},
        {1, "A"},
        {2, "B"},
        {3, "C"},
        {4, "D"},
        {5, "E"}
    };
    std::unordered_map<int, std::vector<Shortcut>> edges = {
        {0, {Shortcut(0, 1, 6.0), Shortcut(0, 2, 7), Shortcut(0, 3, 16)}},
        {1, {Shortcut(1, 2, 2.0), Shortcut(1, 5, 11.0)}},
        {2, {Shortcut(2, 3, 8.0), Shortcut(2, 5, 6.0)}},
        {3, {Shortcut(3, 4, 4.0)}},
        {4, {}},
        {5, {Shortcut(5, 3, 5), Shortcut(5, 4, 5.0)}},
    };
    HierarchicalHubLabeling hhl(vertices, edges, 6);
    hhl.preProcess();
    hhl.buildLabels();
    

    
    
    std::cout << "Ranks after preprocessing:" << std::endl;
    for (int i = 0; i < hhl.rank.size(); i++) {
        std::cout << "Vertex " << vertex_labels[i] << ": " << hhl.rank[i] << std::endl;
    }
    
    std::cout << "Levels:" << std::endl;
    for (int i = 0; i < hhl.level.size(); i++) {
        std::cout << "Vertex " << vertex_labels[i] << ": " << hhl.level[i] << std::endl;
    }


    std::cout << std::endl;

    std::cout << "Forward Labels:" << std::endl;
    for (const auto& [vertex, labels] : hhl.labelsForward) {
        std::cout << "Vertex " << vertex_labels[vertex] << ":" << std::endl;
        for (const auto& [target, dist_rank] : labels) {
            std::cout << "  Target " << vertex_labels[target] << " Distance: " << dist_rank << std::endl;
        }
    }

    std::cout << std::endl;

    // Display backward labels with vertex labels
    std::cout << "Backward Labels:" << std::endl;
    for (const auto& [vertex, labels] : hhl.labelsBackward) {
        std::cout << "Vertex " << vertex_labels[vertex] << ":" << std::endl;
        for (const auto& [target, dist_rank] : labels) {
            std::cout << "  Target " << vertex_labels[target] << " Distance: " << dist_rank << std::endl;
        }
    }

    std::cout << std::endl;

    int s = 0, t = 4;
    int distance = hhl.query(s, t);
    std::cout << "Shortest path distance from " << vertex_labels[s] << " to " << vertex_labels[t] << " is " << distance << std::endl;


    return 0;
}
