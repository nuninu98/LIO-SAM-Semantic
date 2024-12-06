#include "DataType.h"

namespace LIO_SAM_SEMANTIC{
    Detection::Detection(){

    }

    Detection::Detection(const cv::Rect& roi, const cv::Mat& mask, const string& name): roi_(roi), mask_(mask), name_(name){

    }

    Detection::Detection(const Detection& d): roi_(d.roi_), mask_(d.mask_), name_(d.name_), Q_(d.Q_){

    }

    Detection& Detection::operator=(const Detection& d){
        roi_ = d.roi_;
        mask_ = d.mask_.clone();
        name_ = d.name_;
        Q_ = d.Q_;
        return *this;
    }

    Detection::~Detection(){

    }

    cv::Rect Detection::getROI_CV() const{
        return roi_;
    }

    gtsam_quadrics::AlignedBox2 Detection::getROI() const{
        double xmin =roi_.x;
        double xmax = roi_.x + roi_.width;
        double ymin = roi_.y;
        double ymax = roi_.y + roi_.height;
        return gtsam_quadrics::AlignedBox2(xmin, ymin, xmax, ymax);
    }

    cv::Mat Detection::getMask() const{
        return mask_;
    }

    string Detection::getClassName() const{
        return name_;
    }

    void Detection::calcInitQuadric(const cv::Mat& depth_scaled, const cv::Mat& mask, const Eigen::Matrix3d& K){
        cv::Mat depth_masked;
        depth_scaled.copyTo(depth_masked, mask);        
        pcl::PointCloud<pcl::PointXYZ> cloud;
        vector<pcl::PointXYZ> sort_pt;
        for(int r = roi_.y; r < roi_.y+ roi_.height; ++r){
            for(int c = roi_.x; c < roi_.x + roi_.width; ++c){
                float depth = depth_masked.at<float>(r, c);
                if(isnanf(depth) || depth < 1.0e-4){
                    continue;
                }
                pcl::PointXYZ pt;
                pt.x = (c - K(0, 2)) * depth / K(0, 0);
                pt.y = (r - K(1, 2)) * depth / K(1, 1);
                pt.z = depth;
                //cloud.push_back(pt);
                sort_pt.push_back(pt);
            }
        }
        // std::cout<<"2222"<<std::endl;
        sort(sort_pt.begin(), sort_pt.end(), [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
            return p1.z < p2.z;
        });
        // std::cout<<"3333"<<std::endl;
        int max_ = sort_pt.size() * 0.95;
        for(int i = 0; i < max_; ++i){
            cloud.push_back(sort_pt[i]);
        }
        if(cloud.size() < 10){
            Q_ = gtsam_quadrics::ConstrainedDualQuadric(gtsam::Pose3(), gtsam::Vector3(0, 0, 0));
            return;
        }
        // std::cout<<"5555"<<std::endl;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud, centroid);
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(cloud, min_pt, max_pt);
        // std::cout<<"6666"<<std::endl;
        Eigen::Vector3f center = (max_pt.getVector3fMap() + min_pt.getVector3fMap())/2.0;
        // std::cout<<"777"<<std::endl;
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(cloud, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	    Eigen::Vector3f eigenValuesPCA  = eigen_solver.eigenvalues();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 
        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

        Eigen::Matrix3f eigenVectorsPCA1;
        eigenVectorsPCA1.col(0) = eigenVectorsPCA.col(2);
        eigenVectorsPCA1.col(1) = eigenVectorsPCA.col(1);
        eigenVectorsPCA1.col(2) = eigenVectorsPCA.col(0);
        eigenVectorsPCA = eigenVectorsPCA1;

        Eigen::Vector3f ea = (eigenVectorsPCA).eulerAngles(2, 1, 0); //yaw pitch roll
        Eigen::AngleAxisf keep_Z_Rot(ea[0], Eigen::Vector3f::UnitZ());
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translate(center);  
        transform.rotate(keep_Z_Rot);
         
        pcl::PointCloud<pcl::PointXYZ> transformedCloud;
        pcl::transformPointCloud(cloud, transformedCloud, transform.inverse());
        pcl::PointXYZ min_pt_T, max_pt_T;
        pcl::getMinMax3D(transformedCloud, min_pt_T, max_pt_T);
        Eigen::Vector3f center_new = (max_pt_T.getVector3fMap() + min_pt_T.getVector3fMap()) / 2;
        Eigen::Vector3f box_dim;
        box_dim = max_pt_T.getVector3fMap() - min_pt_T.getVector3fMap();
        Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
        transform2.translate(center_new);
        Eigen::Affine3f transform3 = transform * transform2;

        gtsam::Pose3 pose(transform3.matrix().cast<double>());
        Q_= gtsam_quadrics::ConstrainedDualQuadric(pose, box_dim.cast<double>()/ 2.0);
        //std::cout<<"9999"<<std::endl;
        // return Q;
    }

    gtsam_quadrics::ConstrainedDualQuadric Detection::Q() const{
        return Q_;
    }

    //=================Detection Group===========
    DetectionGroup::DetectionGroup(const DetectionGroup& dg): stamp(dg.stamp), detections(dg.detections), view(dg.view){

    }

    DetectionGroup& DetectionGroup::operator=(const DetectionGroup& dg){
        stamp = dg.stamp;
        detections = dg.detections;
        view = dg.view;
        return *this;
    }
    //============================================
}