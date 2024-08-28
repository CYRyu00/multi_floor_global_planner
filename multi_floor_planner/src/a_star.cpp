#include "multi_floor_planner/a_star.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <iostream>

int get_id(int x, int y, const nav_msgs::OccupancyGrid& map){
    //some bias correction (tuned)
    return (y + map.info.width/2 - map.info.origin.position.y/2 + 2)*map.info.width + x + map.info.width/2 - map.info.origin.position.x/2 + 2;
}
// Check if a cell is free
bool isFree(int x, int y, const nav_msgs::OccupancyGrid& map) {
    int width = map.info.width;
    int height = map.info.height;
    
    if (x + map.info.width/2 - map.info.origin.position.x/2 < 0 || y + map.info.height/2 - map.info.origin.position.y/2 + 2 < 0 || x + map.info.width/2 - map.info.origin.position.x/2 >= width || y + map.info.height/2 - map.info.origin.position.y/2 + 2 >= height) return false;
    int index = get_id(x,y,map);
    return map.data[index] == 0;
}

// Find the nearest free cell around a given node
Node findNearestFreeCell(const Node& node, const nav_msgs::OccupancyGrid& map) {
    int width = map.info.width;
    int height = map.info.height;

    // Search within a small radius for a free cell
    int radius = 2;  // Adjust the radius as needed
    for (int r = 0; r <= radius; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                int nx = node.x + dx;
                int ny = node.y + dy;
                if (isFree(nx, ny, map)) {
                    return {nx, ny, 0, 0, nullptr};
                }
            }
        }
    }

    return node;  // Return the original node if no free cell is found within the radius
}

bool Node::operator>(const Node& other) const {
    return cost + heuristic > other.cost + other.heuristic;
}

std::vector<Node> getNeighbors(const Node& node, const nav_msgs::OccupancyGrid& map) {
    std::vector<Node> neighbors;
    int width = map.info.width;
    int height = map.info.height;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int nx = node.x + dx;
            int ny = node.y + dy;
            if (nx + width/2 - map.info.origin.position.x/2 +2 >= 0 && ny + height/2 - map.info.origin.position.y/2 >= 0 && nx+ width/2 - map.info.origin.position.x/2 +2 < width && ny + height/2 - map.info.origin.position.y/2 < height && map.data[get_id(nx,ny,map)] == 0) {
                neighbors.push_back({nx, ny, 0, 0, nullptr});
            }
        }
    }
    return neighbors;
}

double heuristic(const Node& a, const Node& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

PathResult reconstructPath(Node* node) {
    std::vector<Node> path;
    double total_cost = 0.0;

    while (node) {
        path.push_back(*node);
        if (node->parent) {
            total_cost += heuristic(*node, *(node->parent));  // Add the cost between the node and its parent
        }
        node = node->parent;
    }

    std::reverse(path.begin(), path.end());
    return {path, total_cost};
}

PathResult aStar(const nav_msgs::OccupancyGrid& map, Node start, Node goal) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_set<int> closed_set;
    std::unordered_map<int, double> g_costs;

    int start_id = get_id(start.x,start.y,map);
    int goal_id = get_id(goal.x, goal.y,map);// (goal.y + map.info.width/2 - map.info.origin.position.y/2 + 2)*map.info.width + goal.x + map.info.width/2 - map.info.origin.position.x/2 + 2;
    
    if (!isFree(start.x, start.y, map)) {
        std::cout << "Start is occupied by an obstacle. Adjusting start position..." << std::endl;
        std::cout << start.x << ", " << start.y << " : " << start_id << " " <<  static_cast<int>(map.data[start_id]) << std::endl;
        start = findNearestFreeCell(start, map);
    }

    if (!isFree(goal.x, goal.y, map)) {
        std::cout << "Goal is occupied by an obstacle. Adjusting goal position..." << std::endl;
        std::cout << goal.x << ", " << goal.y << " : " <<  goal_id << " " << static_cast<int>(map.data[goal_id]) << std::endl;
        goal = findNearestFreeCell(goal, map);
    }

    if (!isFree(start.x, start.y, map) || !isFree(goal.x, goal.y, map)) {
        std::cout << "Could not find a free start or goal position." << std::endl;
        return {{}, 0.0};  // No path found
    }

    open_set.push(start);
    g_costs[start_id] = 0;

    // std::cout << "Starting A* search from (" << start.x << ", " << start.y << ") to (" << goal.x << ", " << goal.y << ")" << std::endl;

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        int current_id = get_id(current.x,current.y,map);

        if (current_id == goal_id) {
            // std::cout << "Path found!" << std::endl;
            return reconstructPath(&current);
        }

        closed_set.insert(current_id);

        for (Node& neighbor : getNeighbors(current, map)) {
            int neighbor_id = get_id(neighbor.x,neighbor.y,map);
            if (closed_set.find(neighbor_id) != closed_set.end()) continue;

            double tentative_g_cost = g_costs[current_id] + heuristic(current, neighbor);
            if (g_costs.find(neighbor_id) == g_costs.end() || tentative_g_cost < g_costs[neighbor_id]) {
                neighbor.cost = tentative_g_cost;
                neighbor.heuristic = heuristic(neighbor, goal);
                neighbor.parent = new Node(current);
                g_costs[neighbor_id] = tentative_g_cost;
                open_set.push(neighbor);
            }
        }
    }

    std::cout << "No path found" << std::endl;
    return {{}, 0.0};  // No path found
}
