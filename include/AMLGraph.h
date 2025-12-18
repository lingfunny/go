#ifndef AMLGRAPH_H
#define AMLGRAPH_H

#include <vector>
#include <string>
#include <iostream>
#include <tuple>

struct EBox {
    int ivex, jvex;
    EBox *ilink, *jlink;
    int weight;
    EBox() : ivex(-1), jvex(-1), ilink(nullptr), jlink(nullptr), weight(1) {}
};

struct VexBox {
    EBox *firstedge;
    VexBox() : firstedge(nullptr) {}
};

class AMLGraph {
private:
    std::vector<VexBox> adjMulList;
    int numVex, numEdge;
    std::vector<bool> visited;

public:
    AMLGraph();
    ~AMLGraph();
    void createGraph(int v, const std::vector<std::tuple<int, int, int>>& edges); // u, v, weight
    
    void BFSTraverse(int startNode);
    void DFSTraverse(int startNode);

private:
    void DFS(int v, std::vector<int>& result, std::vector<std::pair<int, int>>& spanningTreeEdges);
};

#endif
