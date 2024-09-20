#ifndef ICP_HPP
#define ICP_HPP

#include "frame.hpp"
#include "transformation.hpp"
#include "localization.hpp"

Eigen::Matrix4d performICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);
Eigen::Matrix4d performGICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);
Eigen::Matrix4d performGICP_for_Test(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);

Eigen::Matrix4d performNDT(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);
Eigen::Matrix4d performBEVICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, 
                              bool& icp_success, double& score);

Eigen::Matrix4d initialize_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, 
                                pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud,
                                bool &icp_result,
                                const std::vector<Eigen::Matrix4d>& divisions);
Eigen::Matrix4d rotated_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud,
                            bool& icp_result,
                            Eigen::Matrix4d result_matrix,
                            int divide);


extern float map_x_size;
extern float map_y_size;
extern float map_z_size;

extern Eigen::Vector4f min_pt, max_pt;
extern int result_cnt;
#endif