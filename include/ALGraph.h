#ifndef ALGRAPH_H
#define ALGRAPH_H

#include <vector>
#include <string>
#include <iostream>
#include <tuple>

struct ArcNode {
    int adjvex;
    int weight;
    ArcNode* nextarc;
    ArcNode(int v, int w) : adjvex(v), weight(w), nextarc(nullptr) {}
};

struct VNode {
    ArcNode* firstarc;
    VNode() : firstarc(nullptr) {}
};

class ALGraph {
private:
    std::vector<VNode> adjList;
    int numVex, numEdge;
    std::vector<bool> visited;

    void DFS(int v, std::vector<std::pair<int, int>>& treeEdges);

public:
    ALGraph();
    ~ALGraph();
    void createGraph(int v, const std::vector<std::tuple<int, int, int>>& edges);

    void buildDFSTree(int startNode, const std::string& filename);
    void buildBFSTree(int startNode, const std::string& filename);
    
    void DFSNonRecursive(int startNode);

    void dijkstra(int startNode, const std::string& filename);

    void exportDOT(const std::string& filename);
};

#endif
