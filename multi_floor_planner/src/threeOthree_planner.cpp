#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "multi_floor_planner/a_star.h"
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
 

std::vector<nav_msgs::OccupancyGrid> maps(3);
nav_msgs::OccupancyGrid current_map;
ros::Publisher map_pub;
ros::Publisher path_pub;
ros::Publisher stair_pose_pub;
ros::Publisher floor_pub;
ros::Publisher estimate_pose_pub;
ros::ServiceClient clearCostmapsClient;
bool map_received = false;
bool map_published = false;
double ONE_PER_ROOT_2 = 0.7071067805519557;

// [stair_index][floor][x,y,z, x,y,z,w] 
// last one is middle point of each stair
std::vector<std::vector<std::vector<double>>> stair_poses = {{{-0.7611, 4.7775, 0, 0, 0, 0.616683, 0.78721099},{-4.898, 1.447401 ,1, 0, 0, 1, 0},{1000,1000}
                                                             ,{1.1037, 12.5152, 0, 0, 0,  -0.124465021,  0.99222399607},{-0.017147362,-0.697180, 0.5, 0, 0,0.710779729537,0.70341465443}} , 
                                                             {{1000,1000}, { -10.2837, -15.04201, 1, 0,0, -0.0230720746983, 0.99973380},{-0.1649, 0.2791, 2, 0,0, 0.23394,0.9722510},
                                                              { -2.82033729, -15.7544593, 1, 0,0, -0.0230720746983, 0.99973380},{-4.5485, -1.4678, 1.5, 0,0, 0.23394,0.9722510}}};

//303
// {{{-0.1612, 3.259, 0, 0, 0, 0.4319965129, 0.901875275},{5.5462, 3.8062 ,1, 0, 0, 0.68775885, 0.7259392},{1000,1000}
// ,{5.2816, 10.687, 0, 0, 0, -0.7378795, 0.951493429},{3.3560,-1.447, 0.5, 0, 0,-0.0131546,0.9999134}} , 
// {{1000,1000}, { -10.801, 7.477829, 1, 0,0, -0.7378795,0.6749323},{-0.188, 3.051, 2, 0,0, 0.23394,0.9722510},
// { -11.296, 2.9742, 1, 0,0, -0.7378795,0.6749323},{-4.5485, -1.4678, 1.5, 0,0, 0.23394,0.9722510}}};
                                                        //   {{{271, -189,0, 0,0.7071067,0.7071067},{257, 174 ,1, 0,0.7071067,0.7071067 },{1000,1000}} ,
                                                        //   {{-265, 170}, {-265, -170},{1000,1000}}, 
                                                        //   {{1000,1000}, {-203, -170},{-203, 180}},
                                                        //   {{1000,1000}, {170, -194},{-125, -194}}};
int start_floor = 0; 
int goal_floor  = 2;
int n = 4; //num of nodes (start + stairs + goal)

std::vector<std::vector<int>> is_available = {
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {0, 0, 0, 0},
    };
std::vector<int> floor_of_nodes = {0,1,2,2};

std::vector<std::vector<Node>> stairs(stair_poses.size());
Node start, goal;

std::vector<int> minPath = {-1,-1,-1,-1};     

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
                    start_node.x = start.x - maps[start_floor].info.origin.position.x/maps[start_floor].info.resolution;
                    start_node.y = start.y - maps[start_floor].info.origin.position.y/maps[start_floor].info.resolution; 
                }else{
                    floor = floor_of_nodes[i];
                    start_node.x = stairs[i-1][floor].x - maps[floor].info.origin.position.x/maps[floor].info.resolution;
                    start_node.y = stairs[i-1][floor].y - maps[floor].info.origin.position.y/maps[floor].info.resolution;
                   
                }

                if (j == n-1){
                    goal_node.x = goal.x - maps[goal_floor].info.origin.position.x/maps[goal_floor].info.resolution; 
                    goal_node.y = goal.y - maps[goal_floor].info.origin.position.y/maps[goal_floor].info.resolution;                
                }else{ 
                    
                    goal_node.x= stairs[j-1][floor].x - maps[floor].info.origin.position.x/maps[floor].info.resolution;
                    goal_node.y= stairs[j-1][floor].y - maps[floor].info.origin.position.y/maps[floor].info.resolution;
                    // std::cout << "j - 1 :" << j-1 << " floor : " << floor << std::endl;
                }

                // std::cout << "Planning from node " << i << " to " << j <<" at floor  " << floor << std::endl;
                // std::cout << "start is " << start_node.x << ", " << start_node.y << std::endl;
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
            pose.pose.position.x = node.x * maps[floor].info.resolution + maps[floor].info.origin.position.x;
            pose.pose.position.y = node.y * maps[floor].info.resolution + maps[floor].info.origin.position.y;
            pose.pose.position.z = floor;
            ros_path.poses.push_back(pose);
        }
        //add middle point
        if(floor != goal_floor){
        geometry_msgs::PoseStamped pose;
        
        pose.pose.position.x = stair_poses[node_idx-1][goal_floor+1][0];
        pose.pose.position.y = stair_poses[node_idx-1][goal_floor+1][1];
        pose.pose.position.z = floor + 0.5;
        ros_path.poses.push_back(pose);
        pose.pose.position.x = stair_poses[node_idx-1][goal_floor+2][0];
        pose.pose.position.y = stair_poses[node_idx-1][goal_floor+2][1];
        pose.pose.position.z = floor + 1;
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

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
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

                //publish pose for localization
                //TODO
                pose_msg.header.frame_id = "map"; 
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.pose.position.x =  stair_poses[stair_idx][goal_floor + 2][0];
                pose_msg.pose.position.y =  stair_poses[stair_idx][goal_floor + 2][1];
                pose_msg.pose.position.z =  stair_poses[stair_idx][goal_floor + 2][2];
                pose_msg.pose.orientation.x =  stair_poses[stair_idx][goal_floor + 2][3];
                pose_msg.pose.orientation.y =  stair_poses[stair_idx][goal_floor + 2][4];
                pose_msg.pose.orientation.z =  stair_poses[stair_idx][goal_floor + 2][5];
                pose_msg.pose.orientation.w =  stair_poses[stair_idx][goal_floor + 2][6]; 
                estimate_pose_pub.publish(pose_msg);

                //Clear cost map
                std_srvs::Empty srv;
                if (clearCostmapsClient.call(srv)) {
                    ROS_INFO("Successfully called /move_base/clear_costmaps service.");
                } else {
                    ROS_ERROR("Failed to call service /move_base/clear_costmaps.");
                }

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

        std_msgs::Int32 floor_msg;
        floor_msg.data = current_floor+1;
        floor_pub.publish(floor_msg);
        // maps[current_floor].header.frame_id = "map";
        // map_pub.publish(maps[current_floor]);
        
    }

    map_pub.publish(maps[current_floor]);
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "threeOthree_planner");
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

    path_pub = nh.advertise<nav_msgs::Path>("multi_floor_planner/global_plan", 1);
    floor_pub = nh.advertise<std_msgs::Int32>("multi_floor_planner/current_floor", 1);
    stair_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    estimate_pose_pub =nh.advertise<geometry_msgs::PoseStamped>("multi_floor_planner/estimate_pose", 1);

    clearCostmapsClient = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    double resolution = 0.1;
    for(int i=0 ; i<stair_poses.size();i++){
    stairs[i] = { {static_cast<int>(stair_poses[i][0][0] / resolution),static_cast<int>(stair_poses[i][0][1] / resolution), 0, 0, nullptr}, 
                {static_cast<int>(stair_poses[i][1][0] / resolution),static_cast<int>(stair_poses[i][1][1] / resolution), 0, 0, nullptr},
                {static_cast<int>(stair_poses[i][2][0] / resolution),static_cast<int>(stair_poses[i][2][1] / resolution), 0, 0, nullptr}};
    
    }
    
    ros::Subscriber localized_pose = nh.subscribe<nav_msgs::Odometry>("/localized_odom", 5, poseCallback);
    
    ros::spin();
    return 0;
}
