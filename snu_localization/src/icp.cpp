#include "icp.hpp"


Eigen::Matrix4d performICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score)
{
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaxCorrespondenceDistance(2000); 
    icp.setMaximumIterations(200);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*Final);
    score = icp.getFitnessScore();
    
    
    if(icp.hasConverged() && score < 1.0){
        icp_success = true;
        
        Eigen::Matrix4d matrix = icp.getFinalTransformation().cast<double>();
        
        return matrix;
    }else{
        icp_success = false;
        std::cout<<"ICP not converged the score is "<<score<<std::endl;
        Eigen::Matrix4d matrix_failed = Eigen::Matrix4d::Identity();
        return matrix_failed;
    }
}

Eigen::Matrix4d performGICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score)
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    
    gicp.setMaxCorrespondenceDistance(500);
    gicp.setMaximumIterations(600);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setRANSACIterations(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>());
    gicp.align(*Final);
    score = gicp.getFitnessScore();

    if(gicp.hasConverged() && score < 0.5){
        icp_success = true;
        Eigen::Matrix4d matrix = gicp.getFinalTransformation().cast<double>();
        return matrix;
    }else{
        icp_success = false;
        std::cout << "GICP did not converge, the score is " << score << std::endl;
        Eigen::Matrix4d matrix_failed = Eigen::Matrix4d::Identity();
        return matrix_failed;
    }
}

Eigen::Matrix4d performGICP_for_Test(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score)
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    
    gicp.setMaxCorrespondenceDistance(500);
    gicp.setMaximumIterations(800);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setRANSACIterations(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>());
    gicp.align(*Final);
    score = gicp.getFitnessScore();

        // Log the attempt number and score to a file
    std::string file_path = "/home/hyss/localization/snu_local/icp_scores.txt";
    std::ofstream outfile(file_path, std::ios_base::app);
    if (!outfile)
    {
        std::cerr << "Error opening file for writing!" << std::endl;
    }
    else
    {
        outfile << result_cnt << " " << score << std::endl;
        outfile.close();
        result_cnt++;
    }


    if(gicp.hasConverged() && score < 0.3){
        icp_success = true;
        Eigen::Matrix4d matrix = gicp.getFinalTransformation().cast<double>();
        return matrix;
    }else{
        icp_success = false;
        std::cout << "GICP did not converge, the score is " << score << std::endl;
        Eigen::Matrix4d matrix_failed = Eigen::Matrix4d::Identity();
        return matrix_failed;
    }
}

Eigen::Matrix4d performNDT(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score)
{
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setInputSource(source_cloud);
    ndt.setInputTarget(target_cloud);
    
    // Parameters
    ndt.setResolution(5.0); // Adjust resolution for balance between speed and accuracy
    ndt.setStepSize(0.2);    // Control the step size for each iteration
    ndt.setTransformationEpsilon(1e-6);
    ndt.setMaximumIterations(400);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>());
    ndt.align(*Final);
    score = ndt.getFitnessScore();

    if(ndt.hasConverged() && score < 2.0){
        icp_success = true;
        Eigen::Matrix4d matrix = ndt.getFinalTransformation().cast<double>();
        return matrix;
    }else{
        icp_success = false;
        std::cout << "NDT did not converge, the score is " << score << std::endl;
        Eigen::Matrix4d matrix_failed = Eigen::Matrix4d::Identity();
        return matrix_failed;
    }
}

Eigen::Matrix4d performBEVICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, 
                              bool& icp_success, double& score)
{

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    
    icp.setMaxCorrespondenceDistance(500);
    icp.setMaximumIterations(500);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-10);
    icp.setRANSACIterations(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>());
    icp.align(*Final);
    score = icp.getFitnessScore();

    if (icp.hasConverged() && score < 0.8) { 
        icp_success = true;
        Eigen::Matrix4f matrix = icp.getFinalTransformation();
        return matrix.cast<double>();
    } else {
        icp_success = false;
        std::cout << "BEV ICP did not converge, the score is " << score << std::endl;
        return Eigen::Matrix4d::Identity();
    }

}



Eigen::Matrix4d initialize_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, 
                                pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud,
                                bool &icp_result,
                                const std::vector<Eigen::Matrix4d>& divisions)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

    double best_icp_score =200.0;
    icp_result = false; 

    for(int i=0; i<divisions.size();i++)
    {
        Eigen::Matrix4d temp_trans = divisions.at(i);
        int size = source_cloud ->points.size();

        for(int i=0 ; i<4; i++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>(size,1));
            Eigen::Matrix4d rot = make_rot_matrix(90*i);

            for(int i=0; i<size;i++)
            {
                Transformation_points(&source_cloud->points[i], &temp_cloud->points[i], temp_trans*rot);
            }
            sensor_msgs::PointCloud2 curr_msgs;
            pcl::toROSMsg(*temp_cloud, curr_msgs);
            curr_msgs.header.frame_id = map_frame;       
            pubinit_cloud.publish(curr_msgs); 


            bool icp_success = false;
            double score = 200.0;
 

            Eigen::Matrix4d current_result = performNDT(temp_cloud, map_cloud, icp_success, score);

            if(icp_success && score < best_icp_score)
            {
                best_icp_score = score;
                result = current_result*temp_trans*rot;
                initialized = true;
                icp_result = true;
                std::cout<<"Initialize updated: "<<best_icp_score<<std::endl;
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr changed_Cloud(new pcl::PointCloud<pcl::PointXYZI>(size,1));

                for(int i=0; i<size;i++)
                {
                    Transformation_points(&source_cloud->points[i], &changed_Cloud->points[i], result);
                }

                sensor_msgs::PointCloud2 chan_msgs;
                pcl::toROSMsg(*changed_Cloud, chan_msgs);
                chan_msgs.header.frame_id = map_frame;

                pubchange.publish(chan_msgs);
            }
        }

    }

    if(!icp_result)
    {
        std::cout<<"Initailized failed"<<std::endl;
        initialized = false;
    }else{
        std::cout<<"Initialized successed"<<std::endl;
    }

    return result;
}



Eigen::Matrix4d rotated_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud,
                            bool& icp_result,
                            Eigen::Matrix4d result_matrix,
                            int divide)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

    double best_icp_score =200.0;
    icp_result = false; 
    int size = source_cloud ->points.size();

    for(int i=0; i<divide;i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>(size,1));
        Eigen::Matrix4d rot = make_rot_matrix(360/divide*i);

        for(int i=0; i<size;i++)
        {
            Transformation_points(&source_cloud->points[i], &temp_cloud->points[i], result_matrix*rot);
        }
        sensor_msgs::PointCloud2 curr_msgs;
        pcl::toROSMsg(*temp_cloud, curr_msgs);
        curr_msgs.header.frame_id = map_frame;       
        pubinit_cloud.publish(curr_msgs);

        bool icp_success = false;
        double score = 200.0;

        Eigen::Matrix4d current_result = performGICP(temp_cloud, map_cloud, icp_success, score); 
        
        if(icp_success && score < best_icp_score)
        {
            best_icp_score = score;
            result = current_result*result_matrix*rot;
            icp_result = true;
            std::cout<<"Initialize rotated succeed: "<<best_icp_score<<std::endl;
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr changed_Cloud(new pcl::PointCloud<pcl::PointXYZI>(size,1));

            for(int i=0; i<size;i++)
            {
                Transformation_points(&source_cloud->points[i], &changed_Cloud->points[i], result);
            }

            sensor_msgs::PointCloud2 chan_msgs;
            pcl::toROSMsg(*changed_Cloud, chan_msgs);
            chan_msgs.header.frame_id = map_frame;

            pubchange.publish(chan_msgs);
        }
    }  

    if(!icp_result)
    {
        std::cout<<"Initailized Rotation failed"<<std::endl;
    }else{
        std::cout<<"Initialized Rotation successed"<<std::endl;
    }

    return result;
}

