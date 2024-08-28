#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "multi_floor_planner/a_star.h"
 

std::vector<nav_msgs::OccupancyGrid> maps(3);
nav_msgs::OccupancyGrid current_map;
ros::Publisher path_pub;
bool map_received = false;

//ros::Publisher map_pub;

// Callback to receive the map from the /map1 topic
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg, const int floor) { 
    //int floor=0;
    if (floor < maps.size()) {
        maps[floor] = *map_msg;  // Copy the map data to the global variable
        current_map = *map_msg;  // Save the latest map for path planning
        map_received = true;
        ROS_INFO("Map received from /map%d.", floor+1);
        
        // Publish the map
        //map_pub.publish(maps[floor]);
    }
    else {
        ROS_WARN("Floor index out of range.");
    }

}

std::vector<int> multistageGraph(int n, const std::vector<std::vector<double>>& graph) {
    // Initialize dp array to store minimum cost to reach destination from each node
    std::vector<double> dp(n, std::numeric_limits<double>::infinity());
    dp[n-1] = 0.0;  // Cost to reach destination from itself is 0

    // Path array to store the path from source to destination
    std::vector<int> path(n, -1);

    // Fill dp array in reverse order (from second last node to first node)
    for (int i = n - 2; i >= 0; i--) {
        for (int j = i + 1; j < n; j++) {
            if (graph[i][j] != std::numeric_limits<double>::infinity() && dp[i] > graph[i][j] + dp[j]) {
                dp[i] = graph[i][j] + dp[j];
                path[i] = j;
            }
        }
    }

    // Reconstruct the path from source to destination
    std::vector<int> minPath;
    int node = 0;
    while (node != -1) {
        minPath.push_back(node);
        node = path[node];
    }

    // Return the path
    return minPath;
}


// [stair_index][floor][x or y]
std::vector<std::vector<std::vector<int>>> stair_poses = {{{-32, -54},{40, -54}} ,
                                                          {{33, 67}, {-33,67}}};

// Callback for the start and goal poses
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& start_msg, const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
    ROS_INFO("Goal received");
    if (!map_received) {
        ROS_WARN("No map received yet. Cannot compute path.");
        return;
    }
    int start_floor = 0; 
    int goal_floor  = 1;
    int n = 4; //num of nodes (start + stairs + goal)

    Node start = {static_cast<int>(start_msg->pose.position.x / maps[start_floor].info.resolution),
                  static_cast<int>(start_msg->pose.position.y / maps[start_floor].info.resolution), 0, 0, nullptr};
    Node goal = {static_cast<int>(goal_msg->pose.position.x / maps[goal_floor].info.resolution),
                 static_cast<int>(goal_msg->pose.position.y / maps[goal_floor].info.resolution), 0, 0, nullptr};
    
    std::vector<std::vector<Node>> stairs(stair_poses.size());
    stairs[0] = {{stair_poses[0][0][0],stair_poses[0][0][1], 0, 0, nullptr}, {stair_poses[0][1][0],stair_poses[0][1][1], 0, 0, nullptr}};
    stairs[1] = {{stair_poses[1][0][0],stair_poses[1][0][1], 0, 0, nullptr}, {stair_poses[1][1][0],stair_poses[1][1][1], 0, 0, nullptr}};

    std::vector<std::vector<PathResult>> results(n, std::vector<PathResult>(n)); //stair index
    std::vector<std::vector<double>> graph(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<int>> is_available = {
        {0, 1, 1, 0},
        {0, 0, 0, 1},
        {0, 0, 0, 1},
        {0, 0, 0, 0}
    };
    std::vector<int> floor_of_nodes = {0,1,1,1};

    

    for (int i = 0; i < n ; i++){
        for(int j=0 ; j < n;j++){
            
            if(is_available[i][j]==1){
                Node start_node = {0, 0, 0, 0, nullptr};
                Node goal_node  = {0, 0, 0, 0, nullptr};
                int floor = 0;
                
                if (i == 0){
                    start_node.x = start.x;
                    start_node.y = start.y; 
                    floor = start_floor;
                }else{
                    floor = floor_of_nodes[i];
                    start_node.x = stairs[i-1][floor].x;
                    start_node.y = stairs[i-1][floor].y;
                   
                }

                if (j == n-1){
                    goal_node.x = goal.x; 
                    goal_node.y = goal.y;                
                }else{ 
                    goal_node.x= stairs[j-1][floor].x;
                    goal_node.y= stairs[j-1][floor].y;
                }

                results[i][j] = aStar(maps[floor], start_node, goal_node); 
                graph[i][j] = results[i][j].total_cost;
            }
        }
    }


    // Calculate the minimum cost path
    std::vector<int> minPath = multistageGraph(n, graph);

    // Output the minimum path and cost
    nav_msgs::Path ros_path;
    ros_path.header.stamp = ros::Time::now();
    ros_path.header.frame_id = "map";

    std::cout << "Path: ";
    int floor = start_floor;
    int prev_node_idx = 0;
    int next_node_idx = 0;
    
    for (int node_idx : minPath) {
        std::cout << node_idx << " ";
        if(node_idx == 0) continue;    
        
        next_node_idx = node_idx;
        for (const Node& node : results[prev_node_idx][next_node_idx].path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = node.x * maps[floor].info.resolution;
            pose.pose.position.y = node.y * maps[floor].info.resolution;
            pose.pose.position.z = floor;
            ros_path.poses.push_back(pose);
        }
        prev_node_idx = next_node_idx;
        floor++;

    }
    std::cout << std::endl;
    path_pub.publish(ros_path);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "one2two_planner");
    ros::NodeHandle nh;
    
    // Subscribe to the map topic
    
    // Subscribe to /map1 and /map2 topics using the same callback
    ros::Subscriber map1_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map1", 1, boost::bind(mapCallback, _1, 0));
    ros::Subscriber map2_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map2", 1, boost::bind(mapCallback, _1, 1));


    message_filters::Subscriber<geometry_msgs::PoseStamped> start_sub(nh, "my_initialpose", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> goal_sub(nh, "move_base_simple/goal", 1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), start_sub, goal_sub);
    sync.registerCallback(boost::bind(&goalCallback, _1, _2));

    path_pub = nh.advertise<nav_msgs::Path>("one2two_plan", 1);
    
    ros::spin();
    return 0;
}
