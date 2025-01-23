#include <DataType.h>

namespace LIO_SAM_SEMANTIC{
    Detection::Detection(){

    }

    Detection::Detection(const cv::Rect& roi,  const string& name): roi_(roi), name_(name){
        cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    Detection::Detection(const Detection& d): roi_(d.roi_), name_(d.name_){
        cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*(d.cloud)));
    }

    Detection& Detection::operator=(const Detection& d){
        roi_ = d.roi_;
        name_ = d.name_;
        *cloud = *(d.cloud);
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


    string Detection::getClassName() const{
        return name_;
    }

    void Detection::calcCloud(const cv::Mat& depth_scaled, const cv::Mat& mask, const Eigen::Matrix3d& K){
        cloud->clear();
        cv::Mat depth_masked;
        depth_scaled.copyTo(depth_masked, mask);        
        vector<pcl::PointXYZI> sort_pt;
        pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
        for(int r = roi_.y; r < roi_.y+ roi_.height; ++r){
            for(int c = roi_.x; c < roi_.x + roi_.width; ++c){
                float depth = depth_masked.at<float>(r, c);
                if(isnanf(depth) || depth < 1.0e-4){
                    continue;
                }
                pcl::PointXYZI pt;
                pt.x = (c - K(0, 2)) * depth / K(0, 0);
                pt.y = (r - K(1, 2)) * depth / K(1, 1);
                pt.z = depth;
                //cloud.push_back(pt);
                sort_pt.push_back(pt);
            }
        }
        // std::cout<<"2222"<<std::endl;
        sort(sort_pt.begin(), sort_pt.end(), [](const pcl::PointXYZI& p1, const pcl::PointXYZI& p2){
            return p1.z < p2.z;
        });
        // std::cout<<"3333"<<std::endl;
        int max_ = sort_pt.size() * 0.8;
        int min_ = sort_pt.size() * 0.2;
        for(int i = min_; i < max_; ++i){
            cloud->push_back(sort_pt[i]);
        }
        // for(int i = min_; i < max_; ++i){
        //     input->push_back(sort_pt[i]);
        // }
        // if(input->empty()){
        //     return;
        // }
        // //===============Filtering=========
        // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        // tree->setInputCloud(input);
        // vector<pcl::PointIndices> cluster_indices;
        // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        // ec.setClusterTolerance(0.2);
        // ec.setMinClusterSize(100);
        // ec.setSearchMethod (tree);
        // ec.setInputCloud (input);
        // ec.extract (cluster_indices);
        // sort(cluster_indices.begin(), cluster_indices.end(), [](const pcl::PointIndices& i1, const pcl::PointIndices& i2){
        //     return i1.indices.size() > i2.indices.size();
        // });
        // if(cluster_indices.empty()){
        //     return;
        // }
        // for(const auto& idx : cluster_indices[0].indices){
        //     cloud->push_back(input->points[idx]);
        // }

        // if(cloud.size() < 10){
        //     Q_ = gtsam_quadrics::ConstrainedDualQuadric(gtsam::Pose3(), gtsam::Vector3(0, 0, 0));
        //     return;
        // }
        // Eigen::Vector4f centroid;
        // pcl::compute3DCentroid(cloud, centroid);
        // pcl::PointXYZI min_pt, max_pt;
        // pcl::getMinMax3D(cloud, min_pt, max_pt);
        // Eigen::Vector3f center = (max_pt.getVector3fMap() + min_pt.getVector3fMap())/2.0;
        // Eigen::Matrix3f covariance;
        // pcl::computeCovarianceMatrixNormalized(cloud, centroid, covariance);
        // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        // Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	    // Eigen::Vector3f eigenValuesPCA  = eigen_solver.eigenvalues();
        // eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 
        // eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        // eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

        // Eigen::Matrix3f eigenVectorsPCA1;
        // eigenVectorsPCA1.col(0) = eigenVectorsPCA.col(2);
        // eigenVectorsPCA1.col(1) = eigenVectorsPCA.col(1);
        // eigenVectorsPCA1.col(2) = eigenVectorsPCA.col(0);
        // eigenVectorsPCA = eigenVectorsPCA1;

        // Eigen::Vector3f ea = (eigenVectorsPCA).eulerAngles(2, 1, 0); //yaw pitch roll
        // Eigen::AngleAxisf keep_Z_Rot(ea[0], Eigen::Vector3f::UnitZ());
        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        // transform.translate(center);  
        // transform.rotate(keep_Z_Rot);
         
        // pcl::PointCloud<pcl::PointXYZI> transformedCloud;
        // pcl::transformPointCloud(cloud, transformedCloud, transform.inverse());
        // pcl::PointXYZI min_pt_T, max_pt_T;
        // pcl::getMinMax3D(transformedCloud, min_pt_T, max_pt_T);
        // Eigen::Vector3f center_new = (max_pt_T.getVector3fMap() + min_pt_T.getVector3fMap()) / 2;
        // Eigen::Vector3f box_dim;
        // box_dim = max_pt_T.getVector3fMap() - min_pt_T.getVector3fMap();
        // box_dim = box_dim.cwiseAbs();
        // Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
        // transform2.translate(center_new);
        // Eigen::Affine3f transform3 = transform * transform2;

        // gtsam::Pose3 pose(transform3.matrix().cast<double>());
        // Q_= gtsam_quadrics::ConstrainedDualQuadric(pose, box_dim.cast<double>()/ 2.0);
    
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