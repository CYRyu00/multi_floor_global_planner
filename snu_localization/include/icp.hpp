#ifndef ICP_HPP
#define ICP_HPP

#include "frame.hpp"
#include "transformation.hpp"
#include "localization.hpp"

Eigen::Matrix4d performICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);
Eigen::Matrix4d performGICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);

Eigen::Matrix4d performNDT(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);
Eigen::Matrix4d performBEVICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, 
                              bool& icp_success, double& score);


#endif