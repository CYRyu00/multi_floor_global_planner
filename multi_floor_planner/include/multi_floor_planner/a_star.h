#ifndef A_STAR_H
#define A_STAR_H

#include <nav_msgs/OccupancyGrid.h>
#include <vector>

struct Node {
    int x, y;
    double cost, heuristic;
    Node* parent;
    bool operator>(const Node& other) const;
};

struct PathResult {
    std::vector<Node> path;
    double total_cost;

    //PathResult() : total_cost(std::numeric_limits<double>::infinity());
};

PathResult aStar(const nav_msgs::OccupancyGrid& map, Node start, Node goal);

#endif // A_STAR_H
