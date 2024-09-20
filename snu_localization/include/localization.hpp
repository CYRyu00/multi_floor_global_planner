#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <fstream>  // Include this for file operations

#include <iostream>
#include <string>
#include <cstdlib> //for servers
#include <thread>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>  // for servers

//PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> //icp
#include <pcl/registration/gicp.h>//gicp
#include <pcl/registration/ndt.h>//ndt





#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <queue>
#include <vector>
#include <math.h>


#include "frame.hpp"
#include "transformation.hpp"
#include "icp.hpp"
double UROP_localization(bool& icp_success, Eigen::Matrix4d& icp_transformation, int cnt);
double localization(bool& icp_success, Eigen::Matrix4d& icp_transformation, int cnt);
double estimate_initial_position(bool& success, Eigen::Matrix4d& icp_transformation, int cnt);
pcl::PointCloud<pcl::PointXYZ>::Ptr convertToBEV(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

pcl::PointCloud<pcl::PointXYZI>::Ptr make_cloud_submap(int index, int submap_size);
void voxelize_pcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
void voxelize_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
void voxelize_entire_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
void voxelize_multi_resolution(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double size);



extern std::vector<frame_pose> frames; 
extern pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
extern std::string map_frame; 
extern ros::Publisher pubinitodom;
extern ros::Publisher pubodom;
extern ros::Publisher pubinit_cloud;
extern ros::Publisher pubcrop_cloud;
extern ros::Publisher pubchange;
extern std::mutex update_pose_mutex;
extern std::mutex update_icp_trans;

extern bool initialized;
extern Eigen::Matrix4d icp_transformation_result;
extern Eigen::Matrix4d center_trans;

extern double scan_voxel_size;
extern double map_voxel_size;
extern double map_entire_voxel_size;
extern double NDT_voxel_size;

extern std::vector<Eigen::Matrix4d> map_div_points;
extern int failed_count;
extern bool failed_bool;
extern bool changed_floor_input;

extern bool UROP_change_posecbk_input;
extern bool UROP_floor_change;

#endif