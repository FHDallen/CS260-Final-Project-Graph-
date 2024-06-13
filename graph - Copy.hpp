/**
 * Program: Graph.cpp
 * Name: Dylan Allen
 * Class: CS260
 * Professor: Joseph Jess
 * 
 * 
 * Description:
 * This program defines a graph data structure with various functionalities. 
 * The graph consists of nodes (vertices) and edges with weights. It supports 
 * adding vertices and edges, performing depth-first search (DFS), finding the 
 * shortest path between two nodes using Dijkstra's algorithm, and finding the 
 * minimum spanning tree (MST) using Kruskal's algorithm. The program also 
 * includes test cases to demonstrate these functionalities.
 */

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <limits>
#include <set>
#include <algorithm>

class Node {
public:
    int value;
    std::vector<std::pair<Node*, int>> adjacent;

    Node(int val);
};

class Graph {
private:
    std::unordered_map<int, Node*> nodes;

    int findRoot(int node, std::unordered_map<int, int>& parent);

public:
    ~Graph();

    void add_vertex(int value);
    void add_edge(int from, int to, int weight = 1);
    void get_adjacent_nodes(int value);
    void dfs(int startValue);
    void shortest_path(int startValue, int endValue);
    void min_span_tree();

    void dfsHelper(Node* node, std::set<int>& visited);
};

void run_tests();

#endif // GRAPH_HPP