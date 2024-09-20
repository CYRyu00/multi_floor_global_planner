#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <cmath>

void World_TO_IMU(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po, Eigen::Matrix4d transformation_matrix);
void IMU_TO_LIDAR(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po);
void LIDAR_TO_CAMERA(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po);
void Camera_to_IMU(pcl::PointXYZRGB *pi, pcl::PointXYZRGB *po);
void Transformation_points(pcl::PointXYZI *pi, pcl::PointXYZI *po, Eigen::Matrix4d corrected_matrix);

void ROBOT_TO_WORLD_CORRECTION(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po, Eigen::Matrix4d corrected_matrix);
Eigen::Matrix4d make_eigen_matrix(double x, double y, double z);
Eigen::Matrix4d make_rot_matrix(double angle);

extern Eigen::Matrix4d extrinsic_matrix;
extern Eigen::Matrix4d imu_lidar_matrix;
#endif 
