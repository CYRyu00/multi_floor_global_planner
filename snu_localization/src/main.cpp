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

//BASIC SETTINGS.... NO change
string root_dir = ROOT_DIR;
///MAP FRAME
std::string map_frame = "map";
double map_entire_voxel_size;
//IMU-LIDAR EXTRINSIC
vector<double> imu_lidar_matrix_vector;


/////////////////////////////////// 저장 공간
// for saving all the pcd
pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);

// frames
std::vector<frame_pose> frames; // 1_frame={odom,pointcloud}
/////////////////////////////////////////////////////////////////////////////

///FLOOR 관리/////////////
//floor
int current_floor=1;
std::string floor_1;
std::string floor_2;
std::string floor_3;


/////////ROS PUBLISHER SUBSCRIBER/////////////////////////
ros::Subscriber robot_signal_current_floor;
ros::Subscriber robot_signal_estimate_pose;

// for initial estimation
ros::Publisher pubinitodom;
ros::Publisher scan_cloud;
ros::Publisher scan_result;
ros::Publisher pubmap; //303동 Map publish
ros::Publisher map_crop;
// lio result -> convert into localization coord.
ros::Publisher pubodom;
ros::Publisher pubpath;
//////////////////////////////////////////////////////////////////////////////////////////


///LOCAZLIATION TOOL
bool initialized_bool = false;
bool floor_changed_bool = false;

int icp_failed_count =0;
int icp_succ_cout = 0;
Eigen::Matrix4d icp_transformation_result = Eigen::Matrix4d::Identity();
Eigen::Matrix4d floor_changed_odom = Eigen::Matrix4d::Identity();
bool floor_saftey=false;;


////MUTEX CONTROL//////
std::mutex frame_mutex;
std::mutex localization_pose_mutex;
std::mutex map_mutex;

//Thread
std::thread pub_path;
std::thread pub_initial;



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
    frame_mutex.lock();
    localization_pose_mutex.lock();
    for (int i = 0; i < frames.size(); i++)
    {
        frames.at(i).changed_odoemtry = icp_transformation_result * frames.at(i).transformation_matrix;
    }
    localization_pose_mutex.unlock();
    frame_mutex.unlock();

    nav_msgs::Odometry corrected_odom;
    nav_msgs::Odometry initial_pose;
    nav_msgs::Path corrected_path;

    corrected_path.header.frame_id = map_frame;

    frame_mutex.lock();
    for (int i = 0; i < frames.size(); i++)
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

        if(i==0)
        {
            initial_pose.header.stamp = ros::Time::now();
            initial_pose.header.frame_id = map_frame;
            initial_pose.child_frame_id = "base_link";

            initial_pose.pose.pose.position.x = p.changed_odoemtry(0, 3);
            initial_pose.pose.pose.position.y = p.changed_odoemtry(1, 3);
            initial_pose.pose.pose.position.z = p.changed_odoemtry(2, 3);

            initial_pose.pose.pose.orientation.x = quaternion.x();
            initial_pose.pose.pose.orientation.y = quaternion.y();
            initial_pose.pose.pose.orientation.z = quaternion.z();
            initial_pose.pose.pose.orientation.w = quaternion.w();           
        }

        if (i == frames.size() - 1)
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
    frame_mutex.unlock();
    
    pubinitodom.publish(initial_pose);
    pubodom.publish(corrected_odom);
    pubpath.publish(corrected_path);
}

void path_thread()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        rate.sleep();
        publish_path(pubodom, pubpath);
    }
}
int cnt;
void inital_thread()
{
    ros::Rate rate(3.0);

    while (ros::ok())
    {
        rate.sleep();

        if (frames.size() > 30)
        {
            
            localization_pose_mutex.lock();
            if(floor_saftey){
                icp_transformation_result = floor_changed_odom;
                floor_saftey = false;
            }
            Eigen::Matrix4d pose_temp = icp_transformation_result;
            localization_pose_mutex.unlock();
            bool success;

            if(!initialized_bool)
            {
                std::cout << "Start! Current Floor is  "<< current_floor << std::endl;
                success = UROP_localization(pose_temp, cnt);
                
                localization_pose_mutex.lock();
                if(success)
                {
                    icp_transformation_result = pose_temp;
                    icp_failed_count =0;
                    icp_succ_cout++;
                    if(icp_succ_cout >10)
                    {
                        initialized_bool = true;
                        icp_succ_cout =0;
                        floor_changed_bool = false;
                        std::cout<<"Initialized Completed!"<<std::endl;
                    }
     
                }
                else
                {
                    icp_failed_count++;
                    icp_succ_cout=0;
                    if(icp_failed_count >5)
                    {
                        initialized_bool= false;
                        icp_failed_count = 0;
                    }
                }
                localization_pose_mutex.unlock();
                std::cout<<" "<<std::endl;

            }else
            {
                if(cnt%50==0){
                    std::cout << "Start! Current Floor is  "<< current_floor << std::endl;
                    success = UROP_localization(pose_temp, cnt);

                    localization_pose_mutex.lock();
                    if(success)
                    {
                        icp_transformation_result = pose_temp;
                        icp_failed_count =0;
                        icp_succ_cout++;

                        if(icp_failed_count >10)
                        {
                            initialized_bool= false;
                            icp_failed_count = 0;
                        }
                    }
                    else
                    {
                        icp_failed_count++;
                        icp_succ_cout=0;
                        if(icp_succ_cout >10)
                        {
                            initialized_bool = true;
                            icp_succ_cout =0;
                        }
                    }
                    localization_pose_mutex.unlock();
                    std::cout<<" "<<std::endl;

                }
            }

            cnt++;
        }
    }
}


void floor_cbk(const boost::shared_ptr<const std_msgs::Int32>& msg)
{
    if(current_floor != msg->data)
    {
        //FRAME RESET -> FLOOR is changed...!
        frame_mutex.lock();
        std::vector<frame_pose> temp;
        
        int start_index = std::max(0, static_cast<int>(frames.size()) - 10);
        for(int i = start_index; i < frames.size(); i++)
        {
            temp.push_back(frames.at(i));
        }

        frames.clear();
        frames.shrink_to_fit();
        std::cout<<"frame is reset, size is: "<<frames.size()<<std::endl;

        if(temp.size()>0){

            for(int i=0;i<temp.size();i++){
                
                if(i>=temp.size()) continue;

                frames.push_back(temp.at(i));
            }
        }


        Eigen::Matrix4d new_init =Eigen::Matrix4d::Identity();
        if(frames.size()>0)
            new_init = frames.at(frames.size()-1).transformation_matrix;

        frame_mutex.unlock();


        /// Localization RESET..!!!
        localization_pose_mutex.lock();
        initialized_bool = false;
        //new_init= icp_transformation_result* new_init;
        if(frames.size()>0){
            icp_transformation_result = new_init.inverse();
            floor_changed_odom = new_init.inverse();
            floor_saftey=true;
        } //Eigen::Matrix4d::Identity();
        floor_changed_bool = true;
        icp_failed_count = 0;
        localization_pose_mutex.unlock();




/////////////////////////// Floor subscribe ////////////////////////////////////////////////
        current_floor = msg->data;
        std::cout << "Floor is "<<current_floor<<std::endl;
        std::string pcd_file;

        if(current_floor == 1) pcd_file = floor_1;
        else if(current_floor == 2) pcd_file = floor_2;
        else pcd_file = floor_3;
        map_mutex.lock();
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *map_cloud) == -1)
        {
            ROS_ERROR("Couldn't read file %s", pcd_file.c_str());
            return;
        }
        map_mutex.unlock();

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_vis(new pcl::PointCloud<pcl::PointXYZI>());
        *map_vis = *map_cloud;
        voxelize_pcd(map_vis,map_entire_voxel_size);
        
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*map_vis, map_msg);
        map_msg.header.frame_id = map_frame;
        pubmap.publish(map_msg);
///////////////////////////////////////////////////////////////////////////////////////////////        
    }
    return;
}

void robot_pose_cbk(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg)
{
    geometry_msgs::Pose current_pose;
    current_pose = msg->pose;

    localization_pose_mutex.lock();
    initialized_bool = true;
    icp_failed_count = 0;
    icp_transformation_result(0, 3) = current_pose.position.x;
    icp_transformation_result(1, 3) = current_pose.position.y;
    icp_transformation_result(2, 3) = current_pose.position.z;
    localization_pose_mutex.unlock();

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

        ////NOISE REMOVE.....
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setMeanK(150); // 각 포인트의 주변에서 k가의 이웃 포인트를 고려... k값을 늘리면 더 종교
        sor.setStddevMulThresh(1.0); // 표준편차만큼 떨어져있는 것을 노이즈로 고려... 값이 작을 수록 크게 벗어난 포인트를 더 많이 제거.. 너무 낮추면 유용한 데이터도 소실
        sor.filter(*filtered_cloud); //noise remove code
        //////////////////////////////////////////////////////////////////

        frame_pose current_frame(transformation_matrix, filtered_cloud);

        frame_mutex.lock();
        frames.push_back(current_frame);
        frame_mutex.unlock();

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
    voxelize_pcd(map_vis, map_entire_voxel_size);


    // publishers
    pubinitodom = nh.advertise<nav_msgs::Odometry>("Initial_odometry", 100000);
    pubmap = nh.advertise<sensor_msgs::PointCloud2>("MAP", 100000, true); //303동 MAP
    scan_cloud = nh.advertise<sensor_msgs::PointCloud2>("Scan_points", 100000);
    scan_result = nh.advertise<sensor_msgs::PointCloud2>("Scan_icp_results", 100000);
    map_crop = nh.advertise<sensor_msgs::PointCloud2>("crop_maps", 100000);

    //FAST-LIO RESULT convert into localization coordinate(Localization results)
    pubodom = nh.advertise<nav_msgs::Odometry>("localized_odom", 100000);
    pubpath = nh.advertise<nav_msgs::Path>("localized_path", 100000);

    //MAP Publish
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_vis, map_msg);
    map_msg.header.frame_id = map_frame;
    pubmap.publish(map_msg);

      
    // UROP topics... subscribes
    robot_signal_current_floor = nh.subscribe<std_msgs::Int32>("multi_floor_planner/current_floor",1,floor_cbk);
    robot_signal_estimate_pose = nh.subscribe<geometry_msgs::PoseStamped>("multi_floor_planner/estimate_pose",1,robot_pose_cbk);

    //FAST-LIO RESULTS....
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/cloud_registered", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub(nh, "/Odometry", 1);
    // sync callbacks....
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointcloud_sub, odometry_sub);
    sync.registerCallback(boost::bind(&synchronizedCallback, _1, _2));

    // ROS 루프
    pub_initial =std::thread(inital_thread);
    pub_path = std::thread{path_thread};

    ros::spin();
    
    return 0;
}
