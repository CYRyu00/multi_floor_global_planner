#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "multi_floor_planner/a_star.h"
 

std::vector<nav_msgs::OccupancyGrid> maps(3);
nav_msgs::OccupancyGrid current_map;
ros::Publisher map_pub;
ros::Publisher path_pub;
ros::Publisher stair_pose_pub;
bool map_received = false;
bool map_published = false;
double ONE_PER_ROOT_2 = 0.7071067805519557;

// [stair_index][floor][x,y,z, x,y,z,w] 
// last one is middle point of each stair
std::vector<std::vector<std::vector<double>>> stair_poses = {{{5.395, -3.700, 0, 0, 0, ONE_PER_ROOT_2, ONE_PER_ROOT_2},{5.132, 3.536 ,1, 0, 0, 1, 0 },{1000,1000},{5.264, -0.168, 0.5, 0, 0, ONE_PER_ROOT_2, ONE_PER_ROOT_2}} ,
                                                          {{-5.367, 3.334, 0, 0, 0, -ONE_PER_ROOT_2,ONE_PER_ROOT_2},{-5.434, -3.415 ,1, 0, 0, -ONE_PER_ROOT_2,ONE_PER_ROOT_2 },{1000,1000},{-5.4005, -0.0405, 0.5, 0, 0, -ONE_PER_ROOT_2, ONE_PER_ROOT_2}}, 
                                                          {{1000,1000}, {-4.319, -3.510, 1, 0,0, ONE_PER_ROOT_2,ONE_PER_ROOT_2},{-4.068, 3.721, 2, 0,0, ONE_PER_ROOT_2,ONE_PER_ROOT_2}, {-4.1935, 0.1055, 1.5, 0,0, ONE_PER_ROOT_2,ONE_PER_ROOT_2}},
                                                          {{1000,1000}, {3.611,  -4.010, 1, 0,0, 1,0},{-2.643, -3.804, 2, 0,0,1,0},{0.484, -3.907, 1.5, 0,0, 1,0}}};


                                                        //   {{{271, -189,0, 0,0.7071067,0.7071067},{257, 174 ,1, 0,0.7071067,0.7071067 },{1000,1000}} ,
                                                        //   {{-265, 170}, {-265, -170},{1000,1000}}, 
                                                        //   {{1000,1000}, {-203, -170},{-203, 180}},
                                                        //   {{1000,1000}, {170, -194},{-125, -194}}};
int start_floor = 0; 
int goal_floor  = 2;
int n = 6; //num of nodes (start + stairs + goal)

std::vector<std::vector<Node>> stairs(stair_poses.size());
Node start, goal;

std::vector<int> minPath = {-1,-1,-1,-1,-1,-1};     

int current_floor = start_floor ;
bool is_in_stair = false;
int stair_idx = -1;



// Callback to receive the map from the /map1 topic
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg, const int floor) { 
    //int floor=0;
    if (floor < maps.size()) {
        maps[floor] = *map_msg;  // Copy the map data to the global variable
        current_map = *map_msg;  // Save the latest map for path planning
        map_received = true;
        ROS_INFO("Map received from /map%d.", floor+1);
 
        
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
    std::vector<int> minPath_;
    int node = 0;
    while (node != -1) {
        minPath_.push_back(node);
        node = path[node];
    }

    // Return the path
    return minPath_;
}

// Callback for the start and goal poses
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& start_msg, const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
    ROS_INFO("Goal received");
    if (!map_received) {
        ROS_WARN("No map received yet. Cannot compute path.");
        return;
    }
    maps[current_floor].info.origin.position.z=0;
    map_pub.publish(maps[current_floor]);
    
    std::vector<std::vector<PathResult>> results(n, std::vector<PathResult>(n)); //stair index
    std::vector<std::vector<double>> graph(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<int>> is_available = {
        {0, 1, 1, 0, 0, 0},
        {0, 0, 0, 1, 1, 0},
        {0, 0, 0, 1, 1 ,0},
        {0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 1} , 
        {0, 0, 0, 0, 0, 0}
    };
    std::vector<int> floor_of_nodes = {0,1,1,2,2,2};

    start = {static_cast<int>(start_msg->pose.position.x / maps[start_floor].info.resolution),
                static_cast<int>(start_msg->pose.position.y / maps[start_floor].info.resolution), 0, 0, nullptr};
    goal = {static_cast<int>(goal_msg->pose.position.x / maps[goal_floor].info.resolution),
                    static_cast<int>(goal_msg->pose.position.y / maps[goal_floor].info.resolution), 0, 0, nullptr};


    

    for (int i = 0; i < n ; i++){
        for(int j=0 ; j < n;j++){
            if(is_available[i][j]==1){
                Node start_node = {0, 0, 0, 0, nullptr};
                Node goal_node  = {0, 0, 0, 0, nullptr};
                int floor = 0;
                
                if (i == 0){
                    floor = start_floor;
                    start_node.x = start.x;
                    start_node.y = start.y; 
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
                    // std::cout << "j - 1 :" << j-1 << " floor : " << floor << std::endl;
                }

                // std::cout << "Planning from node " << i << " to " << j <<" at floor : " << floor << std::endl;
                // std::cout << "goal is " << goal_node.x << ", " << goal_node.y << std::endl;
                results[i][j] = aStar(maps[floor], start_node, goal_node); 
                graph[i][j] = results[i][j].total_cost;
            }
        }
    }

    // Calculate the minimum cost path
    minPath = multistageGraph(n, graph);

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

double prev_goal_x = -1000;
double prev_goal_y = -1000;
bool is_goal_middle = false;
int pub_floor = start_floor;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // Access the pose data from the message
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;


    // ROS_INFO("Received pose:  x = %f, y = %f", x, y);
    // ROS_INFO("Prev goal pose: x = %f, y = %f", prev_goal_x, prev_goal_y);
    // Create a PoseStamped message
    geometry_msgs::PoseStamped pose_msg;
    
    if(minPath[1]!=-1){
        if(stair_idx == -1) stair_idx = minPath[1]-1;
        
        double threshold = 0.3;
        bool is_reached = ((prev_goal_x -x)*(prev_goal_x -x) + (prev_goal_y -y)*(prev_goal_y -y)) < (threshold*threshold);
        // int pub_floor = current_floor;

        if(is_reached){
            if(!is_in_stair){
                is_in_stair = true;
                // current_floor++;
                std::cout<< "Get in to the stair from " << current_floor << " to " << current_floor + 1<< std::endl;
                ros::Duration(3.0).sleep();
                //publish middle point of the stair
                is_goal_middle = true;
                pub_floor = goal_floor + 1;
                
            }
            else if(is_goal_middle){
                current_floor++;
                pub_floor = current_floor;
                is_goal_middle = false;
                std::cout<< "Reached middle point between floor " << current_floor -1 << " and " << current_floor << std::endl;
                ROS_INFO("Map changing...");
                ros::Duration(3.0).sleep();
            }
            else{   
                is_in_stair = false;
                std::cout<< "Get out to the stair from " << current_floor -1 << " to " << current_floor << std::endl;
                ros::Duration(3.0).sleep();
                //we should change our goal  stair
                stair_idx = minPath[1 + current_floor]-1;

            }
        }

        
        if(current_floor == goal_floor & !is_in_stair){
            pose_msg.pose.position.x =  goal.x*maps[goal_floor].info.resolution;
            pose_msg.pose.position.y =  goal.y*maps[goal_floor].info.resolution;
            pose_msg.pose.position.z =  goal_floor;//stair_poses[stair_idx][current_floor][2];
            // Orientation must be fixed TODO
            pose_msg.pose.orientation.x =  0;
            pose_msg.pose.orientation.y =  0;
            pose_msg.pose.orientation.z =  0;
            pose_msg.pose.orientation.w =  1; 
        }
        else{
            pose_msg.pose.position.x =  stair_poses[stair_idx][pub_floor][0];
            pose_msg.pose.position.y =  stair_poses[stair_idx][pub_floor][1];
            pose_msg.pose.position.z =  stair_poses[stair_idx][pub_floor][2];
            pose_msg.pose.orientation.x =  stair_poses[stair_idx][pub_floor][3];
            pose_msg.pose.orientation.y =  stair_poses[stair_idx][pub_floor][4];
            pose_msg.pose.orientation.z =  stair_poses[stair_idx][pub_floor][5];
            pose_msg.pose.orientation.w =  stair_poses[stair_idx][pub_floor][6]; 
        }
        
        
        pose_msg.header.frame_id = "map"; 
        pose_msg.header.stamp = ros::Time::now();
        stair_pose_pub.publish(pose_msg);

        prev_goal_x = pose_msg.pose.position.x;
        prev_goal_y = pose_msg.pose.position.y;

        // maps[current_floor].header.frame_id = "map";
        // map_pub.publish(maps[current_floor]);
        
    }
    map_pub.publish(maps[current_floor]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "one2three_planner");
    ros::NodeHandle nh;
    
    // Subscribe to the map topic
    ros::Subscriber map1_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map1", 1, boost::bind(mapCallback, _1, 0));
    ros::Subscriber map2_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map2", 1, boost::bind(mapCallback, _1, 1));
    ros::Subscriber map3_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map3", 1, boost::bind(mapCallback, _1, 2));
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 5);

    message_filters::Subscriber<geometry_msgs::PoseStamped> start_sub(nh, "multi_floor_planner/initial_pose", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> goal_sub(nh, "multi_floor_planner/goal", 1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), start_sub, goal_sub);
    sync.registerCallback(boost::bind(&goalCallback, _1, _2));

    path_pub = nh.advertise<nav_msgs::Path>("global_plan", 1);
    stair_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    
    double resolution = 0.02;
    for(int i=0 ; i<stair_poses.size();i++){
    stairs[i] = { {static_cast<int>(stair_poses[i][0][0] / resolution),static_cast<int>(stair_poses[i][0][1] / resolution), 0, 0, nullptr}, 
                {static_cast<int>(stair_poses[i][1][0] / resolution),static_cast<int>(stair_poses[i][1][1] / resolution), 0, 0, nullptr},
                {static_cast<int>(stair_poses[i][2][0] / resolution),static_cast<int>(stair_poses[i][2][1] / resolution), 0, 0, nullptr}};
    
    }
  
    ros::Subscriber localized_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 5, poseCallback);

    ros::spin();
    return 0;
}
