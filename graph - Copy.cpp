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

#include "Graph.hpp"

// Node class constructor
Node::Node(int val) : value(val) {}

// Graph class destructor
Graph::~Graph() {
    for (auto pair : nodes) {
        delete pair.second;
    }
}

// Add a vertex to the graph
void Graph::add_vertex(int value) {
    if (nodes.find(value) == nodes.end()) {
        nodes[value] = new Node(value); // Create a new node if it doesn't already exist
    }
}

// Add an edge to the graph
void Graph::add_edge(int from, int to, int weight) {
    if (nodes.find(from) != nodes.end() && nodes.find(to) != nodes.end()) {
        nodes[from]->adjacent.push_back(std::make_pair(nodes[to], weight)); // Add edge from 'from' node to 'to' node
        nodes[to]->adjacent.push_back(std::make_pair(nodes[from], weight)); // For undirected graph, add edge from 'to' node to 'from' node
    }
}

// Get adjacent nodes of a given node
void Graph::get_adjacent_nodes(int value) {
    if (nodes.find(value) != nodes.end()) {
        for (auto pair : nodes[value]->adjacent) {
            std::cout << pair.first->value << " "; // Print the value of each adjacent node
        }
        std::cout << std::endl;
    }
}

// Helper function for DFS
void Graph::dfsHelper(Node* node, std::set<int>& visited) { 
    if (node == nullptr) return;
    visited.insert(node->value); // Mark the current node as visited
    std::cout << node->value << " "; // Print the value of the current node
    for (auto pair : node->adjacent) {
        if (visited.find(pair.first->value) == visited.end()) {
            dfsHelper(pair.first, visited); // Recursively visit all adjacent unvisited nodes
        }
    }
}

// Perform DFS
void Graph::dfs(int startValue) { 
    if (nodes.find(startValue) == nodes.end()) return; // If the start node does not exist, return immediately
    std::set<int> visited; // Set to keep track of visited nodes
    dfsHelper(nodes[startValue], visited); // Start DFS from the start node
    std::cout << std::endl;
}

// Find the shortest path using Dijkstra's algorithm
void Graph::shortest_path(int startValue, int endValue) {
    if (nodes.find(startValue) == nodes.end() || nodes.find(endValue) == nodes.end()) return; // If the start or end node does not exist, return immediately
    std::unordered_map<int, int> distances; // Map to store the shortest distance from start to each node
    std::unordered_map<int, int> previous; // Map to store the previous node in the shortest path
    for (auto pair : nodes) { // Initialize distances and previous maps
        distances[pair.first] = std::numeric_limits<int>::max(); // Set all distances to infinity 
        previous[pair.first] = -1; // Set all previous nodes to -1 (undefined)
    }
    distances[startValue] = 0; // Distance to the start node is 0

    auto compare = [&distances](Node* lhs, Node* rhs) {
        return distances[lhs->value] > distances[rhs->value]; // Comparator for the priority queue
    };
    std::priority_queue<Node*, std::vector<Node*>, decltype(compare)> pq(compare); // Priority queue to process nodes

    pq.push(nodes[startValue]); // Start with the start node

    while (!pq.empty()) { // Process nodes in the priority queue
        Node* current = pq.top();
        pq.pop();

        for (auto pair : current->adjacent) { // For each adjacent node
            int alt = distances[current->value] + pair.second; // Calculate the new distance
            if (alt < distances[pair.first->value]) { // If the new distance is shorter
                distances[pair.first->value] = alt; // Update the distance
                previous[pair.first->value] = current->value; // Update the previous node
                pq.push(pair.first); // Push the adjacent node to the priority queue
            }
        }
    }

    // Print shortest path
    if (distances[endValue] == std::numeric_limits<int>::max()) {
        std::cout << "No path found from " << startValue << " to " << endValue << std::endl;
    } else {
        std::vector<int> path;
        for (int at = endValue; at != -1; at = previous[at]) { // Reconstruct the path from end to start
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end()); // Reverse the path to get the correct order
        std::cout << "Shortest path from " << startValue << " to " << endValue << ": "; 
        for (int node : path) {
            std::cout << node << " ";
        }
        std::cout << "with distance " << distances[endValue] << std::endl;
    }
}

// Helper function to find the root in Kruskal's algorithm
int Graph::findRoot(int node, std::unordered_map<int, int>& parent) {
    if (parent[node] != node) {
        parent[node] = findRoot(parent[node], parent); // Path compression
    }
    return parent[node]; // Return the root of the node
}

// Find the minimum spanning tree using Kruskal's algorithm
void Graph::min_span_tree() {
    if (nodes.empty()) return;

    std::vector<std::tuple<int, int, int>> edges; 
    for (auto pair : nodes) { // Collect all edges in the graph
        int from = pair.first;
        for (auto adj : pair.second->adjacent) { // For each adjacent node of the current node
            int to = adj.first->value;
            int weight = adj.second;
            if (from < to) { // To avoid duplicating edges
                edges.push_back(std::make_tuple(weight, from, to)); // Add the edge to the list
            }
        }
    }

    std::sort(edges.begin(), edges.end()); // Sort edges by weight

    std::unordered_map<int, int> parent; // Initialize the parent map for union-find
    for (auto pair : nodes) {
        parent[pair.first] = pair.first; // Each node is its own parent
    }

    std::vector<std::tuple<int, int, int>> mst;
    for (auto edge : edges) { // Process each edge
        int weight, from, to;
        std::tie(weight, from, to) = edge;

        int rootFrom = findRoot(from, parent); // Find the root of the 'from' node
        int rootTo = findRoot(to, parent); // Find the root of the 'to' node

        if (rootFrom != rootTo) { // If the nodes are in different sets, union them
            mst.push_back(edge); // Add the edge to the MST
            parent[rootFrom] = rootTo; // Union the sets
        }
    }

    std::cout << "Minimum Spanning Tree edges (with weights):" << std::endl; 
    for (auto edge : mst) { 
        int weight, from, to;
        std::tie(weight, from, to) = edge;
        std::cout << from << " - " << to << " (weight " << weight << ")" << std::endl; // Print each edge with its weight
    }
}

// Tests
void run_tests() {
    Graph graph;

    // Test adding vertices
    graph.add_vertex(1);
    graph.add_vertex(2);
    graph.add_vertex(3);
    graph.add_vertex(4);

    // Test adding edges
    graph.add_edge(1, 2, 1);
    graph.add_edge(1, 3, 4);
    graph.add_edge(2, 3, 2);
    graph.add_edge(3, 4, 1);

    // Test adjacency
    std::cout << "Adjacent nodes to 1: ";
    graph.get_adjacent_nodes(1); // Should output: 2 3 

    std::cout << "Adjacent nodes to 3: ";
    graph.get_adjacent_nodes(3); // Should output: 1 2 4

    // Test DFS
    std::cout << "DFS from node 1: ";
    graph.dfs(1); // Should output nodes in DFS order

    // Test shortest path
    graph.shortest_path(1, 4); // Should output shortest path from 1 to 4

    // Test minimum spanning tree
    graph.min_span_tree(); // Should output edges of the MST
}