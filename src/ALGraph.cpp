#include "ALGraph.h"
#include "MyStack.h"
#include <queue>
#include <stack>
#include <algorithm>
#include <climits>
#include <map>
#include <fstream>

ALGraph::ALGraph() : numVex(0), numEdge(0) {}

ALGraph::~ALGraph() {
    for (int i = 0; i < numVex; ++i) {
        ArcNode* p = adjList[i].firstarc;
        while (p) {
            ArcNode* temp = p;
            p = p->nextarc;
            delete temp;
        }
    }
}

void ALGraph::createGraph(int v, const std::vector<std::tuple<int, int, int>>& edges) {
    numVex = v;
    numEdge = edges.size();
    adjList.resize(numVex);

    for (int i = 0; i < numVex; ++i) {
        adjList[i].firstarc = nullptr;
    }

    for (const auto& edge : edges) {
        int u = std::get<0>(edge);
        int v_node = std::get<1>(edge);
        int w = std::get<2>(edge);

        int i = u - 1;
        int j = v_node - 1;

        if (i < 0 || i >= numVex || j < 0 || j >= numVex) continue;

        ArcNode* node1 = new ArcNode(j, w);
        node1->nextarc = adjList[i].firstarc;
        adjList[i].firstarc = node1;

        ArcNode* node2 = new ArcNode(i, w);
        node2->nextarc = adjList[j].firstarc;
        adjList[j].firstarc = node2;
    }
}


void ALGraph::DFS(int v, std::vector<std::pair<int, int>>& treeEdges) {
    visited[v] = true;
    ArcNode* p = adjList[v].firstarc;
    while (p) {
        int neighbor = p->adjvex;
        if (!visited[neighbor]) {
            treeEdges.push_back({v, neighbor});
            DFS(neighbor, treeEdges);
        }
        p = p->nextarc;
    }
}

void ALGraph::buildDFSTree(int startNode, const std::string& filename) {
    int startIdx = startNode - 1;
    if (startIdx < 0 || startIdx >= numVex) {
        std::cout << "Start node not found!" << std::endl;
        return;
    }

    visited.assign(numVex, false);
    std::vector<std::pair<int, int>> treeEdges;
    DFS(startIdx, treeEdges);

    std::ofstream out(filename);
    if (!out) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    out << "digraph DFS_Tree {" << std::endl;
    for (const auto& edge : treeEdges) {
        int u = edge.first;
        int v = edge.second;
        int weight = 0;
        ArcNode* p = adjList[u].firstarc;
        while (p) {
            if (p->adjvex == v) {
                weight = p->weight;
                break;
            }
            p = p->nextarc;
        }
        out << "  " << (u + 1) << " -> " << (v + 1) << " [label=\"" << weight << "\"];" << std::endl;
    }
    out << "}" << std::endl;
    out.close();
    std::cout << "DFS Spanning Tree exported to " << filename << std::endl;
}

void ALGraph::buildBFSTree(int startNode, const std::string& filename) {
    int startIdx = startNode - 1;
    if (startIdx < 0 || startIdx >= numVex) {
        std::cout << "Start node not found!" << std::endl;
        return;
    }

    visited.assign(numVex, false);
    std::vector<std::pair<int, int>> treeEdges;
    std::queue<int> q;

    visited[startIdx] = true;
    q.push(startIdx);

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        ArcNode* p = adjList[u].firstarc;
        while (p) {
            int neighbor = p->adjvex;
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                treeEdges.push_back({u, neighbor});
                q.push(neighbor);
            }
            p = p->nextarc;
        }
    }

    std::ofstream out(filename);
    if (!out) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    out << "digraph BFS_Tree {" << std::endl;
    for (const auto& edge : treeEdges) {
        int u = edge.first;
        int v = edge.second;
        int weight = 0;
        ArcNode* p = adjList[u].firstarc;
        while (p) {
            if (p->adjvex == v) {
                weight = p->weight;
                break;
            }
            p = p->nextarc;
        }
        out << "  " << (u + 1) << " -> " << (v + 1) << " [label=\"" << weight << "\"];" << std::endl;
    }
    out << "}" << std::endl;
    out.close();
    std::cout << "BFS Spanning Tree exported to " << filename << std::endl;
}

void ALGraph::dijkstra(int startNode, const std::string& filename) {
    int startIdx = startNode - 1;

    if (startIdx < 0 || startIdx >= numVex) {
        std::cout << "Start node not found!" << std::endl;
        return;
    }

    std::vector<int> dist(numVex, INT_MAX);
    std::vector<int> parent(numVex, -1);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    dist[startIdx] = 0;
    pq.push({0, startIdx});

    while (!pq.empty()) {
        int d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u]) continue;
        
        ArcNode* p = adjList[u].firstarc;
        while (p) {
            int v = p->adjvex;
            int weight = p->weight;
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push({dist[v], v});
            }
            p = p->nextarc;
        }
    }

    // I kept the code for printing the shortest path to endNode

    // if (dist[endIdx] == INT_MAX) {
    //     std::cout << "No path from " << startNode << " to " << endNode << std::endl;
    // } else {
    //     std::cout << "Shortest Path Length: " << dist[endIdx] << std::endl;
    //     std::cout << "Path: ";
    //     MyStack<int> path;
    //     int curr = endIdx;
    //     while (curr != -1) {
    //         path.push(curr + 1);
    //         curr = parent[curr];
    //     }
    //     while (!path.isEmpty()) {
    //         std::cout << path.top();
    //         path.pop();
    //         if (!path.isEmpty()) std::cout << " -> ";
    //     }
    //     std::cout << std::endl;
    // }

    std::ofstream out(filename);
    if (!out) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    out << "digraph SPT {" << std::endl;
    for (int i = 0; i < numVex; ++i) {
        if (parent[i] != -1) {
            int pIdx = parent[i];
            int weight = 0;
            ArcNode* p = adjList[pIdx].firstarc;
            while (p) {
                if (p->adjvex == i) {
                    weight = p->weight;
                    break;
                }
                p = p->nextarc;
            }
            out << "  " << (pIdx + 1) << " -> " << (i + 1) << " [label=\"" << weight << "\"];" << std::endl;
        }
    }
    out << "}" << std::endl;
    out.close();
    std::cout << "Shortest Path Tree exported to " << filename << std::endl;
}

void ALGraph::DFSNonRecursive(int startNode) {
    int startIdx = startNode - 1;
    if (startIdx < 0 || startIdx >= numVex) {
        std::cout << "Start node not found!" << std::endl;
        return;
    }

    visited.assign(numVex, false);
    std::vector<int> result;
    MyStack<int> s;
    s.push(startIdx);

    std::cout << "DFS Traversal (Non-Recursive): ";

    while (!s.isEmpty()) {
        int u = s.top();
        s.pop();

        if (!visited[u]) {
            visited[u] = true;
            result.push_back(u + 1);

            std::vector<int> neighbors;
            ArcNode* p = adjList[u].firstarc;
            while (p) {
                if (!visited[p->adjvex]) {
                    neighbors.push_back(p->adjvex);
                }
                p = p->nextarc;
            }
            
            // Push in reverse order
            for (auto it = neighbors.rbegin(); it != neighbors.rend(); ++it) {
                s.push(*it);
            }
        }
    }

    for (int val : result) std::cout << val << " ";
    std::cout << std::endl;
}

void ALGraph::exportDOT(const std::string& filename) {
    std::ofstream out(filename);
    if (!out) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    out << "graph G {" << std::endl;
    for (int i = 0; i < numVex; ++i) {
        ArcNode* p = adjList[i].firstarc;
        while (p) {
            int neighborIdx = p->adjvex;
            if (i < neighborIdx) {
                out << "  " << (i + 1) << " -- " << (neighborIdx + 1) 
                    << " [label=\"" << p->weight << "\"];" << std::endl;
            }
            p = p->nextarc;
        }
    }
    out << "}" << std::endl;
    out.close();
    std::cout << "Graph exported to " << filename << std::endl;
}
