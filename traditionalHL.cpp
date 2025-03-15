#include <vector>
#include <queue>
#include <limits>
#include <unordered_map>
#include <iostream>

using namespace std;

class HubLabeling {
public:
    HubLabeling(int n) : n(n) {
        graph.resize(n);
        reverseGraph.resize(n);
        forwardLabels.resize(n);
        backwardLabels.resize(n);
    }

    void addEdge(int u, int v, int weight) {
        graph[u].emplace_back(v, weight);
        reverseGraph[v].emplace_back(u, weight);
    }

    void buildLabels() {
        
        // Forward labels
        for (int v = 0; v < n; ++v) {
            auto [dist, _] = dijkstra(v, graph);
            for (int u = 0; u < n; ++u) {
                if (dist[u] < numeric_limits<int>::max()) {
                    forwardLabels[v].emplace_back(u, dist[u]);
                }
            }
        }

        // Backward labels
        for (int v = 0; v < n; ++v) {
            auto [dist, _] = dijkstra(v, reverseGraph);
            for (int u = 0; u < n; ++u) {
                if (dist[u] < numeric_limits<int>::max()) {
                    backwardLabels[v].emplace_back(u, dist[u]);
                }
            }
        }
    }

    int query(int s, int t) {
        int minDistance = numeric_limits<int>::max();

        unordered_map<int, int> forwardMap, backwardMap;

        for (const auto& pair : forwardLabels[s]) {
            int u = pair.first;
            int d = pair.second;
            forwardMap[u] = d;
        }

        for (const auto& pair : backwardLabels[t]) {
            int u = pair.first;
            int d = pair.second;
            backwardMap[u] = d;
        }

        for (const auto& pair : forwardMap) {
            int u = pair.first;
            int distS = pair.second;
            if (backwardMap.find(u) != backwardMap.end()) {
                minDistance = min(minDistance, distS + backwardMap[u]);
            }
        }

        return minDistance;
    }

private:
    vector<vector<pair<int, int>>> graph;
    vector<vector<pair<int, int>>> reverseGraph;
    vector<vector<pair<int, int>>> forwardLabels;
    vector<vector<pair<int, int>>> backwardLabels;
    int n;
    
    pair<vector<int>, vector<int>> dijkstra(int source, const vector<vector<pair<int, int>>>& g) {
        vector<int> dist(n, numeric_limits<int>::max());
        vector<int> pred(n, -1);
        dist[source] = 0;

        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.emplace(0, source);

        while (!pq.empty()) {
            int u = pq.top().second;
            int currentDist = pq.top().first;
            pq.pop();

            if (currentDist > dist[u]) continue;

            for (const auto& pair : g[u]) {
                int v = pair.first;
                int weight = pair.second;
                if (currentDist + weight < dist[v]) {
                    dist[v] = currentDist + weight;
                    pred[v] = u;
                    pq.emplace(dist[v], v);
                }
            }
        }

        return {dist, pred};
    }
};

int main() {
    int n = 6; 
    HubLabeling hl(6);

    hl.addEdge(0, 1, 6);
    hl.addEdge(0, 2, 7);
    hl.addEdge(0, 3, 16);
    hl.addEdge(1, 2, 2);
    hl.addEdge(1, 5, 11);
    hl.addEdge(2, 3, 8);
    hl.addEdge(2, 5, 6);
    hl.addEdge(3, 4, 9);
    hl.addEdge(5, 3, 5);
    hl.addEdge(5, 4, 5);

    hl.buildLabels();

    int s = 0, t = 4;
    int distance = hl.query(s, t);
    cout << "Shortest path distance from " << s << " to " << t << " is " << distance << endl;

    return 0;
}
