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
using namespace std; 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, yolo_protocol::YoloResult> yolo_sync_pol;

namespace LIO_SAM_SEMANTIC{
    class Detection{
        public: 
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Detection();

            Detection(const cv::Rect& roi, const cv::Mat& mask, const string& name);

            Detection(const Detection& d);

            ~Detection();   

            cv::Rect getROI_CV() const;
            gtsam_quadrics::AlignedBox2 getROI() const;

            cv::Mat getMask() const;

            string getClassName() const;

            void calcInitQuadric(const cv::Mat& depth_scaled, const cv::Mat& mask, const Eigen::Matrix3d& K);

        private:
            cv::Rect roi_;
            cv::Mat mask_;
            string name_;
            gtsam_quadrics::ConstrainedDualQuadric Q_;
    };

    struct DetectionGroup{
        double stamp;
        vector<Detection> detections;

        DetectionGroup(): stamp(-1.0){

        }

        DetectionGroup(const DetectionGroup& dg);

        DetectionGroup& operator=(const DetectionGroup& dg);
    };
}

#endif