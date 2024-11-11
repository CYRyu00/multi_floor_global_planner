#include "localization.hpp"


/////VOX size
// scan voxel_size
double scan_voxel_size=0; //scan
double map_voxel_size=0; //icp 할 때 map 그때그때
double NDT_voxel_size=0;


bool UROP_localization(Eigen::Matrix4d& pose_transformation, int input_size)
{
    map_mutex.lock();
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_cloud = make_cloud_submap(0,500*input_size);
    map_mutex.unlock();
    
    bool icp_result = false;
    double score;
    int size = aggregated_cloud->points.size();
    
    


 //////////////SCAN과 MAP 준비////////////////////////////////////////////////////   
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    *scan = *aggregated_cloud;
    *map = *map_cloud;

    if(floor_changed_bool && current_floor ==2)
    {
        crop_map_around_origin(map, 8.0, 20.0, 5.0);
        //floor_changed_bool = false;

        sensor_msgs::PointCloud2 curr_msgs_temp;
        pcl::toROSMsg(*map, curr_msgs_temp);
        curr_msgs_temp.header.frame_id = map_frame;       
        map_crop.publish(curr_msgs_temp); 
    }
    
    
    if(floor_changed_bool && current_floor ==3)
    {
        crop_map_around_origin(map, 20.0, 20.0, 10.0);
        //floor_changed_bool = false;

        sensor_msgs::PointCloud2 curr_msgs_temp;
        pcl::toROSMsg(*map, curr_msgs_temp);
        curr_msgs_temp.header.frame_id = map_frame;       
        map_crop.publish(curr_msgs_temp); 
    }
    
    voxelize_pcd(scan,scan_voxel_size);
    voxelize_pcd(map,map_voxel_size);

    for(int i=0; i<size;i++)
    {
        Transformation_points(&scan->points[i], &scan->points[i], pose_transformation);
    }
    
    sensor_msgs::PointCloud2 curr_msgs;
    pcl::toROSMsg(*scan, curr_msgs);
    curr_msgs.header.frame_id = map_frame;       
    scan_cloud.publish(curr_msgs); 
////////////////////////////////////////////////////////////////////////////////


////////////////////Perfom GICP/////////////////////////
    Eigen::Matrix4d icp_result_matrix;
    icp_result_matrix = performGICP(scan,map,icp_result,score);
///////////////////////////////////////////////////////////////////

    if(icp_result)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr changed_Cloud(new pcl::PointCloud<pcl::PointXYZI>(size,1));
        
        for(int i=0; i<size;i++)
        {
            Transformation_points(&scan->points[i], &changed_Cloud->points[i], icp_result_matrix);
        }

        pose_transformation = icp_result_matrix * pose_transformation;

        sensor_msgs::PointCloud2 chan_msgs;
        pcl::toROSMsg(*changed_Cloud, chan_msgs);
        chan_msgs.header.frame_id = map_frame;

        scan_result.publish(chan_msgs);
        //pubinitodom.publish(odom);

    }else
    {
        ROS_WARN("ICP did not converge.");
    }

    return icp_result;

}


pcl::PointCloud<pcl::PointXYZI>::Ptr make_cloud_submap(int index, int submap_size)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Matrix4d matrix;
    for(int i=-submap_size; i<=submap_size; i++)
    {
        int curr = index + i;
        if(curr<0 || curr>=frames.size())
            continue;
        
        *pointcloud += *frames.at(curr).lidar_cloud;
    }


    return pointcloud;
}

void voxelize_pcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double voxel_size)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_pcd;
    voxel_pcd.setLeafSize(voxel_size,voxel_size,voxel_size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr before_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *before_cloud = *cloud;

    voxel_pcd.setInputCloud(before_cloud);
    voxel_pcd.filter(*cloud);

    return;
}

void crop_map_around_origin(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double x, double y, double z)
{
    // 필터 설정
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    
    // X축 필터링
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);
    pass.filter(*cloud);

    // Y축 필터링
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);
    pass.filter(*cloud);

    // Z축 필터링
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-7*z, z);
    pass.filter(*cloud);
}
