#ifndef __LIO_SAM_SEMANTIC_DATA_TYPE_H__
#define __LIO_SAM_SEMANTIC_DATA_TYPE_H__

#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/DualConic.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>
#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <yolo_protocol/YoloResult.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
using namespace std; 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, yolo_protocol::YoloResult> yolo_sync_pol;

namespace LIO_SAM_SEMANTIC{

    double center2Surf(const gtsam_quadrics::ConstrainedDualQuadric& dQ, Eigen::Vector3d dir){
        Eigen::Vector4d d_norm = Eigen::Vector4d::Zero();
        d_norm.block<3, 1>(0, 0) = dir.normalized();
        Eigen::Vector4d Xc = Eigen::Vector4d::Ones();
        Xc.block<3, 1>(0, 0) = dQ.centroid();
        Eigen::Matrix4d Q = dQ.matrix().inverse();
        double a = d_norm.transpose() * Q * d_norm;
        double b = 2.0 * d_norm.transpose()*Q*Xc;
        double c = Xc.transpose()*Q*Xc;
        double t1 = (-b + sqrt(b*b -4.0*a*c))/(2.0*a);
        return abs(t1);
    }

    bool intersects(const gtsam_quadrics::ConstrainedDualQuadric& dQ1, const gtsam_quadrics::ConstrainedDualQuadric& dQ2){
        Eigen::Vector3d c1 = dQ1.centroid();
        Eigen::Vector3d c2 = dQ2.centroid();
        Eigen::Vector3d dir = c1 - c2;
        double center_dist = dir.norm();
        double d1 = center2Surf(dQ1, dir);
        double d2 = center2Surf(dQ2, dir);
        return (center_dist <= d1 + d2);
    }

    gtsam_quadrics::ConstrainedDualQuadric calcQuadric(const pcl::PointCloud<pcl::PointXYZI>& cloud){
        if(cloud.size() < 10){
            return gtsam_quadrics::ConstrainedDualQuadric(gtsam::Pose3(), gtsam::Vector3(0, 0, 0));
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
        pcl::PCA<pcl::PointXYZI> pca;
        pca.setInputCloud(cloud_ptr);

        Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
        Eigen::Vector3f mean = pca.getMean().head<3>();

        // Transform point cloud to PCA-aligned space
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = eigenvectors.transpose();
        transform.block<3, 1>(0, 3) = -eigenvectors.transpose() * mean;

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud_ptr, *transformed_cloud, transform);

        // Compute min and max in aligned space
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
        Eigen::Vector3f trans = eigenvectors * ((max_pt.head<3>() + min_pt.head<3>()) / 2.0f) + mean;
        gtsam::Rot3 R(eigenvectors.cast<double>());
        gtsam::Pose3 pose(R, trans.cast<double>());
        Eigen::Vector3d radii = ((max_pt.head<3>() - min_pt.head<3>()).cwiseAbs() / 2.0f).cast<double>();
        return gtsam_quadrics::ConstrainedDualQuadric(pose, radii);
    }
    
    class Detection{
        public: 
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        private:
            cv::Rect roi_;
            string name_;
            
        public:             
            Detection();

            Detection(const cv::Rect& roi, const string& name);

            Detection(const Detection& d);

            Detection& operator=(const Detection& d);

            ~Detection();   

            cv::Rect getROI_CV() const;
            gtsam_quadrics::AlignedBox2 getROI() const;

            string getClassName() const;

            void calcCloud(const cv::Mat& depth_scaled, const cv::Mat& mask, const Eigen::Matrix3d& K);

    };


    struct DetectionGroup{
        double stamp;
        vector<Detection> detections;
        cv::Mat view;

        DetectionGroup(): stamp(-1.0){

        }

        DetectionGroup(const DetectionGroup& dg);

        DetectionGroup& operator=(const DetectionGroup& dg);
    };
}

#endif