#include <iostream>
#include <string>
#include <cstdlib> //for servers
#include <thread>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h> // for servers
#include <std_msgs/Int32.h>

// PCL
#include <pcl/point_types.h>                 //pt
#include <pcl/point_cloud.h>                 //cloud
#include <pcl/conversions.h>                 //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>

// Transformation, common
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <queue>
#include <vector>
#include <math.h>

// IMAGE
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "transformation.hpp"
#include "frame.hpp"
#include "localization.hpp"
#include "icp.hpp"

using namespace std;
string root_dir = ROOT_DIR;

// for saving all the pcd
pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_wait_save(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);

int pcd_save_interval = -1;
bool pcd_save_en = true;
int pcd_index = 0;

// rosservice
bool program_start = false;

vector<double> imu_lidar_matrix_vector;

// frames
std::vector<frame_pose> frames; // 1_frame={odom,pointcloud}

ros::Subscriber points_sub;

// for initial estimation
ros::Publisher pubinitodom;
ros::Publisher pubinit_cloud;
ros::Publisher pubchange;
ros::Publisher pubmap;
ros::Publisher pubcrop_cloud;

ros::Subscriber robot_signal_current_floor;
ros::Subscriber robot_signal_estimate_pose;
// lio result
ros::Publisher pubodom;
ros::Publisher pubpath;

std::string map_frame = "map";

int recent_index = 0;
std::mutex update_pose_mutex;
std::mutex update_icp_trans;

std::mutex floor_update;

// for initial thread
double min_score = 100.0;
int cnt = 1;
bool initialized = false;
Eigen::Matrix4d icp_transformation_result = Eigen::Matrix4d::Identity();
Eigen::Matrix4d center_trans = Eigen::Matrix4d::Identity();

// scan voxel_size
double scan_voxel_size;
double map_voxel_size;
double map_entire_voxel_size;
double NDT_voxel_size;

//map management
float map_x_size;
float map_y_size;
float map_z_size;

Eigen::Vector4f min_pt, max_pt;
std::vector<Eigen::Matrix4d> map_div_points;

//localization failed
int failed_count =0;
bool failed_bool = false;

bool changed_floor_input = false;

int result_cnt = 1; 

//floor
int current_floor=1;
std::string floor_1;
std::string floor_2;
std::string floor_3;

std::atomic<bool> stop_flag(false);
std::thread pub_initial;
std::thread pub_path;
std::thread input;

//UROP path planning
bool UROP_change_posecbk_input = false;
bool UROP_floor_change = false;

void publish_path(const ros::Publisher &pubodom, const ros::Publisher &pubpath)
{

    if (frames.size() == 0)
    {
        nav_msgs::Odometry empty_odom;
        nav_msgs::Path empty_path;

        empty_odom.header.frame_id = map_frame;
        empty_odom.header.stamp = ros::Time::now();

        empty_path.header.frame_id = map_frame;
        empty_path.header.stamp = ros::Time::now();

        pubodom.publish(empty_odom);
        pubpath.publish(empty_path);

        return;
    }


    // updating

    update_pose_mutex.lock();
    update_icp_trans.lock();
    for (int i = 0; i < frames.size(); i++)
    {
        frames.at(i).changed_odoemtry = icp_transformation_result * frames.at(i).transformation_matrix;
    }
    update_icp_trans.unlock();
    update_pose_mutex.unlock();

    nav_msgs::Odometry corrected_odom;
    nav_msgs::Path corrected_path;

    corrected_path.header.frame_id = map_frame;
    update_pose_mutex.lock();
    for (int i = 0; i < recent_index; i++)
    {
        frame_pose &p = frames[i];

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map_frame;
        pose_stamped.header.stamp = ros::Time::now();

        pose_stamped.pose.position.x = p.changed_odoemtry(0, 3);
        pose_stamped.pose.position.y = p.changed_odoemtry(1, 3);
        pose_stamped.pose.position.z = p.changed_odoemtry(2, 3);

        // Set the orientation from the transformation matrix
        Eigen::Matrix3d rotation_matrix = p.changed_odoemtry.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation_matrix);
        pose_stamped.pose.orientation.x = quaternion.x();
        pose_stamped.pose.orientation.y = quaternion.y();
        pose_stamped.pose.orientation.z = quaternion.z();
        pose_stamped.pose.orientation.w = quaternion.w();

        // publish
        corrected_path.poses.push_back(pose_stamped);

        if (i == recent_index - 1)
        {
            corrected_odom.header.frame_id = map_frame;
            corrected_odom.header.stamp = ros::Time::now();

            corrected_odom.pose.pose.position.x = p.changed_odoemtry(0, 3);
            corrected_odom.pose.pose.position.y = p.changed_odoemtry(1, 3);
            corrected_odom.pose.pose.position.z = p.changed_odoemtry(2, 3);

            corrected_odom.pose.pose.orientation.x = quaternion.x();
            corrected_odom.pose.pose.orientation.y = quaternion.y();
            corrected_odom.pose.pose.orientation.z = quaternion.z();
            corrected_odom.pose.pose.orientation.w = quaternion.w();
        }
    }

    update_pose_mutex.unlock();
    pubodom.publish(corrected_odom);
    pubpath.publish(corrected_path);
}

void path_thread()
{
    ros::Rate rate(10);
    while (ros::ok()&& !stop_flag)
    {
        rate.sleep();
        publish_path(pubodom, pubpath);
    }
}

void inital_thread()
{
    ros::Rate rate(10.0);
    while (ros::ok()&& !stop_flag)
    {
        rate.sleep();

        if (frames.size() > 50)
        {
            std::cout << "Start! Current Floor is  "<< current_floor << std::endl;
            bool success = false;
            Eigen::Matrix4d icp_transformation = Eigen::Matrix4d::Identity();


            double score = UROP_localization(success, icp_transformation, cnt);
        
            /*if (success)
            {
                update_icp_trans.lock();
                icp_transformation_result = icp_transformation;
                update_icp_trans.unlock();
            }*/
           cnt++;

            std:cout<<" "<<std::endl;
        }
    }
}

void input_thread()
{
    ros::Rate rate(0.1);
    while (ros::ok()&& !stop_flag)
    {
        rate.sleep();

        std::string input;
        std::cout << "Enter x, y, z (or press enter to skip, 'R' to reset): ";
        std::getline(std::cin, input);

        if (input == "R")
        {
            update_icp_trans.lock();
            initialized = false;
            update_icp_trans.unlock();
            std::cout << "Initialization reset (initialized = false)" << std::endl;
        }else if(input == "F"){

            failed_bool = true;
            std::cout<<"Failed Rotation starts"<<std::endl;

        }else if (!input.empty())
        {
            float x, y, z;
            std::stringstream ss(input);
            ss >> x >> y >> z;
            std::cout << "Received x: " << x << ", y: " << y << ", z: " << z << std::endl;

            update_icp_trans.lock();
            initialized = true;
            failed_bool = false;
            changed_floor_input = true;
            icp_transformation_result(0, 3) = x;
            icp_transformation_result(1, 3) = y;
            icp_transformation_result(2, 3) = z;
            update_icp_trans.unlock();
        }
    }
}


void floor_cbk(const boost::shared_ptr<const std_msgs::Int32>& msg)
{
    if(current_floor != msg->data)
    {
        /*stop_flag = true;
        if (pub_initial.joinable()) pub_initial.join();
        stop_flag = false;*/

        floor_update.lock();
        frames.clear();
        frames.shrink_to_fit();
        recent_index =0;
        std::cout<<"frame is reset, size is: "<<frames.size()<<std::endl;
        //map_cloud.reset();        
        floor_update.unlock();



        update_icp_trans.lock();
        initialized = false;
        icp_transformation_result = Eigen::Matrix4d::Identity();
        UROP_floor_change = true;
        failed_count = 0;
        update_icp_trans.unlock();



        current_floor = msg->data;
        std::cout << "Floor is "<<current_floor<<std::endl;
        std::string pcd_file;

        if(current_floor == 1) pcd_file = floor_1;
        else if(current_floor == 2) pcd_file = floor_2;
        else pcd_file = floor_3;

        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *map_cloud) == -1)
        {
            ROS_ERROR("Couldn't read file %s", pcd_file.c_str());
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_vis(new pcl::PointCloud<pcl::PointXYZI>());
        *map_vis = *map_cloud;
        voxelize_entire_map(map_vis);
        

        // Calculate min/max for x, y, z
        //Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*map_vis, min_pt, max_pt);

        std::cout << "Min X: " << min_pt[0] << ", Max X: " << max_pt[0] << std::endl;
        std::cout << "Min Y: " << min_pt[1] << ", Max Y: " << max_pt[1] << std::endl;
        std::cout << "Min Z: " << min_pt[2] << ", Max Z: " << max_pt[2] << std::endl;

        // Calculate the center of the entire map
        double center_x = (min_pt[0] + max_pt[0]) / 2.0;
        double center_y = (min_pt[1] + max_pt[1]) / 2.0;
        double center_z = (min_pt[2] + max_pt[2]) / 2.0;

        //std::cout << "Map Center: (" << center_x << ", " << center_y << "," << center_z << ")" << std::endl;

        center_trans = make_eigen_matrix(center_x, center_y, 0);
        map_div_points.push_back(Eigen::Matrix4d::Identity());
        map_div_points.push_back(center_trans);

        /*update_icp_trans.lock();
        icp_transformation_result = center_trans;
        update_icp_trans.unlock();*/

        //map size
        map_x_size = max_pt[0]-min_pt[0];
        map_y_size = max_pt[1]-min_pt[1];
        map_z_size = max_pt[2]-min_pt[2];
    

        //divide map size
        map_div_points.push_back(make_eigen_matrix(center_x+(map_x_size/4), center_y+(map_y_size/4),0));
        map_div_points.push_back(make_eigen_matrix(center_x-(map_x_size/4), center_y+(map_y_size/4),0));
        map_div_points.push_back(make_eigen_matrix(center_x-(map_x_size/4), center_y-(map_y_size/4),0));
        map_div_points.push_back(make_eigen_matrix(center_x+(map_x_size/4), center_y-(map_y_size/4),0));


        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*map_vis, map_msg);
        map_msg.header.frame_id = map_frame;
        pubmap.publish(map_msg);


        /*pub_initial =std::thread(inital_thread);
        pub_path = std::thread{path_thread};
        input = std::thread(input_thread);*/
    // for vis
        pcl::PointCloud<pcl::PointXYZI>::Ptr changed_Cloud(new pcl::PointCloud<pcl::PointXYZI>(1,1));
        sensor_msgs::PointCloud2 chan_msgs;
        pcl::toROSMsg(*changed_Cloud, chan_msgs);
        chan_msgs.header.frame_id = map_frame;
        return;

    }

    return;
}

void robot_pose_cbk(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg)
{
    geometry_msgs::Pose current_pose;
    current_pose = msg->pose;

    update_icp_trans.lock();
    initialized = true;
    failed_bool = false;
    failed_count = 0;
    changed_floor_input = true;
    UROP_change_posecbk_input = true;
    icp_transformation_result(0, 3) = current_pose.position.x;
    icp_transformation_result(1, 3) = current_pose.position.y;
    icp_transformation_result(2, 3) = current_pose.position.z;
    update_icp_trans.unlock();



    std::cout<<"recieved rostopic pose: "<<current_pose.position.x<<" "<<current_pose.position.y<<" "<<current_pose.position.z<<std::endl;   

}

void synchronizedCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud, const nav_msgs::Odometry::ConstPtr &odom)
{
    // position
    double pos_x = odom->pose.pose.position.x;
    double pos_y = odom->pose.pose.position.y;
    double pos_z = odom->pose.pose.position.z;

    // orientation
    double ori_x = odom->pose.pose.orientation.x;
    double ori_y = odom->pose.pose.orientation.y;
    double ori_z = odom->pose.pose.orientation.z;
    double ori_w = odom->pose.pose.orientation.w;

    // Eigen R/T transformation
    Eigen::Quaterniond q(ori_w, ori_x, ori_y, ori_z);
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
    transformation_matrix(0, 3) = pos_x;
    transformation_matrix(1, 3) = pos_y;
    transformation_matrix(2, 3) = pos_z;

    try
    {

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*pointcloud, *pcl_cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setMeanK(150); // 각 포인트의 주변에서 k가의 이웃 포인트를 고려... k값을 늘리면 더 종교
        sor.setStddevMulThresh(1.0); // 표준편차만큼 떨어져있는 것을 노이즈로 고려... 값이 작을 수록 크게 벗어난 포인트를 더 많이 제거.. 너무 낮추면 유용한 데이터도 소실
        sor.filter(*filtered_cloud); //noise remove code

        frame_pose current_frame(transformation_matrix, filtered_cloud);
        floor_update.lock();
        frames.push_back(current_frame);
        floor_update.unlock();
        recent_index++;

        // ROS_DEBUG("Image timestamp: %f", image->header.stamp.toSec());
        ROS_DEBUG("PointCloud timestamp: %f", pointcloud->header.stamp.toSec());
        ROS_DEBUG("Odometry timestamp: %f", odom->header.stamp.toSec());
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snu_localization");
    ros::NodeHandle nh;

    // parameters
    //std::string pcd_file;
    nh.param<std::string>("floor_1", floor_1, "/home/hyss/localization/snu_local/scnas.pcd");
    nh.param<std::string>("floor_2", floor_2, "/home/hyss/localization/snu_local/src/snu_localization/map/noise_removed/303_stairs_1f_pgo_noise.pcd");
    nh.param<std::string>("floor_3", floor_3, "/home/hyss/localization/snu_local/src/snu_localization/map/noise_removed/303_stairs_2f_pgo_noise.pcd");
    
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, true);
    
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    // imu_lidar_extrinsic
    nh.param<vector<double>>("I_L_extrinsic", imu_lidar_matrix_vector, vector<double>());
    // voxel size
    nh.param<double>("scan_voxel_size", scan_voxel_size, 0.2);
    nh.param<double>("map_voxel_size", map_voxel_size, 0.2);
    nh.param<double>("map_entire_voxel_size",map_entire_voxel_size,0.4);
    nh.param<double>("NDT_voxel_size", NDT_voxel_size, 0.5);
    // IL_extrinsic matrix to eigen matrix!
    if (imu_lidar_matrix_vector.size() == 16)
    {
        imu_lidar_matrix << imu_lidar_matrix_vector[0], imu_lidar_matrix_vector[1], imu_lidar_matrix_vector[2], imu_lidar_matrix_vector[3],
            imu_lidar_matrix_vector[4], imu_lidar_matrix_vector[5], imu_lidar_matrix_vector[6], imu_lidar_matrix_vector[7],
            imu_lidar_matrix_vector[8], imu_lidar_matrix_vector[9], imu_lidar_matrix_vector[10], imu_lidar_matrix_vector[11],
            imu_lidar_matrix_vector[12], imu_lidar_matrix_vector[13], imu_lidar_matrix_vector[14], imu_lidar_matrix_vector[15];
    }
    else
    {
        ROS_ERROR("Invalid size for IL_extrinsic matrix vector!");
    }

    std::cout << CV_VERSION << std::endl;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(floor_1, *map_cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s", floor_1.c_str());
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_vis(new pcl::PointCloud<pcl::PointXYZI>());
    *map_vis = *map_cloud;
    voxelize_entire_map(map_vis);
    

    // Calculate min/max for x, y, z
    //Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*map_vis, min_pt, max_pt);

    std::cout << "Min X: " << min_pt[0] << ", Max X: " << max_pt[0] << std::endl;
    std::cout << "Min Y: " << min_pt[1] << ", Max Y: " << max_pt[1] << std::endl;
    std::cout << "Min Z: " << min_pt[2] << ", Max Z: " << max_pt[2] << std::endl;

    // Calculate the center of the entire map
    double center_x = (min_pt[0] + max_pt[0]) / 2.0;
    double center_y = (min_pt[1] + max_pt[1]) / 2.0;
    double center_z = (min_pt[2] + max_pt[2]) / 2.0;

    //std::cout << "Map Center: (" << center_x << ", " << center_y << "," << center_z << ")" << std::endl;

    center_trans = make_eigen_matrix(center_x, center_y, 0);
    map_div_points.push_back(Eigen::Matrix4d::Identity());
    map_div_points.push_back(center_trans);

    /*update_icp_trans.lock();
    icp_transformation_result = center_trans;
    update_icp_trans.unlock();*/

    //map size
    map_x_size = max_pt[0]-min_pt[0];
    map_y_size = max_pt[1]-min_pt[1];
    map_z_size = max_pt[2]-min_pt[2];
 

    //divide map size
    map_div_points.push_back(make_eigen_matrix(center_x+(map_x_size/4), center_y+(map_y_size/4),0));
    map_div_points.push_back(make_eigen_matrix(center_x-(map_x_size/4), center_y+(map_y_size/4),0));
    map_div_points.push_back(make_eigen_matrix(center_x-(map_x_size/4), center_y-(map_y_size/4),0));
    map_div_points.push_back(make_eigen_matrix(center_x+(map_x_size/4), center_y-(map_y_size/4),0));

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // publishers
    pubinitodom = nh.advertise<nav_msgs::Odometry>("Initial_odometry", 100000);
    pubmap = nh.advertise<sensor_msgs::PointCloud2>("localization_map", 100000, true);
    pubinit_cloud = nh.advertise<sensor_msgs::PointCloud2>("Initial_points", 100000);
    pubcrop_cloud = nh.advertise<sensor_msgs::PointCloud2>("crop_cloud", 100000);
    pubchange = nh.advertise<sensor_msgs::PointCloud2>("icp_changed_results", 100000);

    pubodom = nh.advertise<nav_msgs::Odometry>("localized_odom", 100000);
    pubpath = nh.advertise<nav_msgs::Path>("localized_path", 100000);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_vis, map_msg);
    map_msg.header.frame_id = map_frame;
    pubmap.publish(map_msg);

    // subscribers


    robot_signal_current_floor = nh.subscribe<std_msgs::Int32>("multi_floor_planner/current_floor",1,floor_cbk);
    robot_signal_estimate_pose = nh.subscribe<geometry_msgs::PoseStamped>("multi_floor_planner/estimate_pose",1,robot_pose_cbk);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/cloud_registered", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub(nh, "/Odometry", 1);
    // sync callbacks....
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointcloud_sub, odometry_sub);
    sync.registerCallback(boost::bind(&synchronizedCallback, _1, _2));

    // ROS 루프
    pub_initial =std::thread(inital_thread);
    pub_path = std::thread{path_thread};
    input = std::thread(input_thread);
    


    ros::spin();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // pcd save!
    /*if (map_vis->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        ROS_INFO("Current scan is saved to /PCD/%s\n", file_name.c_str());
        cout << "current scan is saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *map_vis);
    }*/

    return 0;
}
