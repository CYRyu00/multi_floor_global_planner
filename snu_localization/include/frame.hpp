#ifndef FRAME_HPP
#define FRAME_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

struct frame_pose {

    Eigen::Matrix4d changed_odoemtry;
    Eigen::Matrix4d transformation_matrix;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud;

    frame_pose(const Eigen::Matrix4d& tf_matrix, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc)
        : transformation_matrix(tf_matrix), lidar_cloud(pc), changed_odoemtry(tf_matrix) {}
};


#endif // FRAME_HPP
