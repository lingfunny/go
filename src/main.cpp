#include <iostream>
#include <vector>
#include <tuple>
#include <random>
#include <ctime>
#include <set>
#include <numeric>
#include <algorithm>
#include "AMLGraph.h"
#include "ALGraph.h"

std::vector<std::tuple<int, int, int>> generateGraphData(int numVex, int numEdge) {
    std::vector<std::tuple<int, int, int>> edges;
    std::set<std::pair<int, int>> existingEdges; // avoid duplicates
    
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<> weightDist(1, 999);
    std::uniform_int_distribution<> nodeDist(1, numVex);

    std::vector<int> nodes(numVex);
    std::iota(nodes.begin(), nodes.end(), 1);
    std::shuffle(nodes.begin(), nodes.end(), rng);

    // Connect each node to a random previous node
    for (int i = 1; i < numVex; ++i) {
        std::uniform_int_distribution<> prevNodeDist(0, i - 1);
        int u = nodes[i];
        int v = nodes[prevNodeDist(rng)];
        
        int w = weightDist(rng);
        edges.emplace_back(u, v, w);
        
        if (u > v) std::swap(u, v);
        existingEdges.insert({u, v});
    }

    // Add remaining edges randomly
    int currentEdgeCount = numVex - 1;
    while (currentEdgeCount < numEdge) {
        int u = nodeDist(rng);
        int v = nodeDist(rng);
        
        if (u == v) continue;
        if (u > v) std::swap(u, v);
        
        if (existingEdges.find({u, v}) == existingEdges.end()) {
            int w = weightDist(rng);
            edges.emplace_back(u, v, w);
            existingEdges.insert({u, v});
            currentEdgeCount++;
        }
    }
    return edges;
}

void printMenu() {
    std::cout << "\n========================================\n";
    std::cout << "      Graph Explorer (Project 4)        \n";
    std::cout << "========================================\n";
    std::cout << "1. Create New Random Graph (>= 3 nodes)\n";
    std::cout << "2. [AML] DFS Traversal (Recursive)\n";
    std::cout << "3. [AML] BFS Traversal\n";
    std::cout << "4. [AL] DFS Traversal (Non-Recursive)\n";
    std::cout << "5. [AL] Export DFS Spanning Tree to DOT\n";
    std::cout << "6. [AL] Export BFS Spanning Tree to DOT\n";
    std::cout << "7. [AL] Shortest Path (Dijkstra) (SPT)\n";
    std::cout << "8. Export Graph to DOT\n";
    std::cout << "0. Exit\n";
    std::cout << "========================================\n";
    std::cout << "Enter your choice: ";
}

int main() {
    AMLGraph amlGraph;
    ALGraph alGraph;
    int numVex = 25;
    int numEdge = 40;
    bool graphCreated = false;

    while (true) {
        printMenu();
        int choice;
        std::cin >> choice;

        if (choice == 0) break;

        switch (choice) {
            case 1: {
                std::cout << "Enter number of vertices: ";
                std::cin >> numVex;
                if (numVex < 3) numVex = 3;
                std::cout << "Enter number of edges (>= " << numVex - 1 << "): ";
                std::cin >> numEdge;
                if (numEdge < numVex - 1) numEdge = numVex - 1;

                auto edges = generateGraphData(numVex, numEdge);
                
                amlGraph.createGraph(numVex, edges);
                alGraph.createGraph(numVex, edges);
                
                std::cout << "Graph created successfully with " << numVex << " vertices and " << edges.size() << " edges.\n";
                graphCreated = true;
                break;
            }
            case 2: {
                if (!graphCreated) { std::cout << "Please create graph first.\n"; break; }
                int startNode;
                std::cout << "Enter start node (1-" << numVex << "): ";
                std::cin >> startNode;
                amlGraph.DFSTraverse(startNode);
                break;
            }
            case 3: {
                if (!graphCreated) { std::cout << "Please create graph first.\n"; break; }
                int startNode;
                std::cout << "Enter start node (1-" << numVex << "): ";
                std::cin >> startNode;
                amlGraph.BFSTraverse(startNode);
                break;
            }
            case 4: {
                if (!graphCreated) { std::cout << "Please create graph first.\n"; break; }
                int startNode;
                std::cout << "Enter start node (1-" << numVex << "): ";
                std::cin >> startNode;
                alGraph.DFSNonRecursive(startNode);
                break;
            }
            case 5: {
                if (!graphCreated) { std::cout << "Please create graph first.\n"; break; }
                int startNode;
                std::cout << "Enter start node (1-" << numVex << "): ";
                std::cin >> startNode;
                alGraph.buildDFSTree(startNode, "dfs_tree.dot");
                std::cout << "DFS Tree saved to dfs_tree.dot\n";
                break;
            }
            case 6: {
                if (!graphCreated) { std::cout << "Please create graph first.\n"; break; }
                int startNode;
                std::cout << "Enter start node (1-" << numVex << "): ";
                std::cin >> startNode;
                alGraph.buildBFSTree(startNode, "bfs_tree.dot");
                std::cout << "BFS Tree saved to bfs_tree.dot\n";
                break;
            }
            case 7: {
                if (!graphCreated) { std::cout << "Please create graph first.\n"; break; }
                int startNode;
                std::cout << "Enter start node (1-" << numVex << "): ";
                std::cin >> startNode;
                alGraph.dijkstra(startNode, "spt.dot");
                std::cout << "Shortest Path Tree saved to spt.dot\n";
                break;
            }
            case 8: {
                if (!graphCreated) { std::cout << "Please create graph first.\n"; break; }
                std::string filename = "graph.dot";
                alGraph.exportDOT(filename);
                std::cout << "Graph exported to " << filename << "\n";
                break;
            }
            default:
                std::cout << "Invalid choice.\n";
        }
    }

    return 0;
}
