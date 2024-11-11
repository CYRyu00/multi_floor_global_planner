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
#include <pcl/filters/passthrough.h>





#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <queue>
#include <vector>
#include <math.h>


#include "frame.hpp"
#include "transformation.hpp"
#include "icp.hpp"


bool UROP_localization(Eigen::Matrix4d& pose_transformation, int size);
pcl::PointCloud<pcl::PointXYZI>::Ptr make_cloud_submap(int index, int submap_size);
void voxelize_pcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double voxel_size);
void crop_map_around_origin(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double x, double y, double z);


/////VOX size
// scan voxel_size
extern double scan_voxel_size;
extern double map_voxel_size;
extern double NDT_voxel_size;

extern ros::Publisher map_crop;
extern ros::Publisher scan_cloud;
extern ros::Publisher scan_result;
extern ros::Publisher pubinitodom;

extern int icp_failed_count;
extern int current_floor;
extern bool initialized_bool;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
extern std::string map_frame;
extern std::vector<frame_pose> frames;
extern bool floor_changed_bool;

extern std::mutex map_mutex;

#endif