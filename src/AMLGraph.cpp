#include "AMLGraph.h"
#include <queue>
#include <iostream>
#include <algorithm>
#include <set>

AMLGraph::AMLGraph() : numVex(0), numEdge(0) {}

AMLGraph::~AMLGraph() {
    std::set<EBox*> uniqueEdges;
    for (int i = 0; i < numVex; ++i) {
        EBox* p = adjMulList[i].firstedge;
        while (p) {
            uniqueEdges.insert(p);
            if (p->ivex == i) p = p->ilink;
            else p = p->jlink;
        }
    }
    for (auto* edge : uniqueEdges) {
        delete edge;
    }
}

void AMLGraph::createGraph(int v, const std::vector<std::tuple<int, int, int>>& edges) {
    numVex = v;
    numEdge = edges.size();
    adjMulList.resize(numVex);
    
    for (int i = 0; i < numVex; ++i) {
        adjMulList[i].firstedge = nullptr;
    }

    for (const auto& edge : edges) {
        int i = std::get<0>(edge) - 1;
        int j = std::get<1>(edge) - 1;
        int w = std::get<2>(edge);

        EBox* newEdge = new EBox();
        newEdge->ivex = i;
        newEdge->jvex = j;
        newEdge->weight = w;

        newEdge->ilink = adjMulList[i].firstedge;
        newEdge->jlink = adjMulList[j].firstedge;

        adjMulList[i].firstedge = newEdge;
        adjMulList[j].firstedge = newEdge;
    }
}

void AMLGraph::DFS(int v, std::vector<int>& result, std::vector<std::pair<int, int>>& spanningTreeEdges) {
    visited[v] = true;
    result.push_back(v + 1);

    EBox* p = adjMulList[v].firstedge;
    while (p) {
        int neighbor = (p->ivex == v) ? p->jvex : p->ivex;
        if (!visited[neighbor]) {
            spanningTreeEdges.push_back({v + 1, neighbor + 1});
            DFS(neighbor, result, spanningTreeEdges);
        }
        if (p->ivex == v) p = p->ilink;
        else p = p->jlink;
    }
}

void AMLGraph::DFSTraverse(int startNode) {
    int startIdx = startNode - 1;
    if (startIdx < 0 || startIdx >= numVex) {
        std::cout << "Start node not found!" << std::endl;
        return;
    }

    visited.assign(numVex, false);
    std::vector<int> result;
    std::vector<std::pair<int, int>> spanningTreeEdges;

    DFS(startIdx, result, spanningTreeEdges);
    
    std::cout << "DFS Traversal (Recursive): ";
    for (int val : result) std::cout << val << " ";
    std::cout << std::endl;

    std::cout << "DFS Spanning Tree Edges: ";
    for (const auto& edge : spanningTreeEdges) {
        std::cout << "(" << edge.first << ", " << edge.second << ") ";
    }
    std::cout << std::endl;
}

void AMLGraph::BFSTraverse(int startNode) {
    int startIdx = startNode - 1;
    if (startIdx < 0 || startIdx >= numVex) {
        std::cout << "Start node not found!" << std::endl;
        return;
    }

    visited.assign(numVex, false);
    std::vector<int> result;
    std::vector<std::pair<int, int>> spanningTreeEdges;
    std::queue<int> q;

    visited[startIdx] = true;
    result.push_back(startIdx + 1);
    q.push(startIdx);

    std::cout << "BFS Traversal: ";

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        EBox* p = adjMulList[u].firstedge;
        while (p) {
            int neighbor = (p->ivex == u) ? p->jvex : p->ivex;
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                result.push_back(neighbor + 1);
                spanningTreeEdges.push_back({u + 1, neighbor + 1});
                q.push(neighbor);
            }
            if (p->ivex == u) p = p->ilink;
            else p = p->jlink;
        }
    }

    std::cout << "BFS Traversal Order: ";
    for (int val : result) std::cout << val << " ";
    std::cout << std::endl;

    std::cout << "BFS Spanning Tree Edges: ";
    for (const auto& edge : spanningTreeEdges) {
        std::cout << "(" << edge.first << ", " << edge.second << ") ";
    }
    std::cout << std::endl;
}
