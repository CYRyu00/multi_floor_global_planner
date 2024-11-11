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






