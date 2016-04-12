/*
 * FiducialFrontEnd.h
 *
 *  Created on: Jan 28, 2016
 *      Author: mklingen
 */

#ifndef FIDUCIALFRONTEND_H_
#define FIDUCIALFRONTEND_H_

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <arm_slam_calib/Landmark.h>
#include <apriltags/AprilTagDetections.h>
#include <opencv2/features2d/features2d.hpp>
#include <gtsam/geometry/Pose3.h>
#include <brisk/brisk.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace gtsam
{

    class FrontEnd
    {
        public:

            class FeatureMatchParams
            {
                public:
                    FeatureMatchParams() :
                        minX(0), minY(0), maxX(9999), maxY(400),
                        hammingThreshold(60.0f), ratioThreshold(1.25f),
                        reprojectionThreshold(150.0f),
                        drawGoodFeatures(true),
                        drawGoodMatches(true),
                        drawCandidates(false),
                        drawBadFeatures(false),
                        drawBadMatches(false),
                        drawGrid(true),
                        doFeatureMatching(true)
                    {

                    }

                    void InitializeNodeHandle(const ros::NodeHandle& nodeHandle);

                    double minX;
                    double minY;
                    double maxX;
                    double maxY;
                    double hammingThreshold;
                    double ratioThreshold;
                    double reprojectionThreshold;
                    bool drawGoodFeatures;
                    bool drawGoodMatches;
                    bool drawCandidates;
                    bool drawBadFeatures;
                    bool drawBadMatches;
                    bool drawGrid;
                    bool doFeatureMatching;
            };

            enum Mode
            {
                Mode_Checkerboard,
                Mode_Apriltags,
                Mode_Features
            };

            FrontEnd(const ros::NodeHandle& nh, const Mode& mode_, const FrontEnd::FeatureMatchParams& params);
            virtual ~FrontEnd();

            void CreateFeatureMatchers();

            void SubscribeSynchronizedCameraAndDepth(const std::string& rgbTopic, const std::string& rgbInfoTopic,
                                                     const std::string& depthTopic, const std::string& depthInfoTopic);
            void SubscribeCamera(const std::string& topic);
            void SubscribeAprilTags(const std::string& topic);
            void SubscribeDepth(const std::string& topic);

            inline bool HasNewData() { return hasNewData; }
            inline bool HasNewApriltags() { return hasNewData; }
            void OnDepthAndRGB(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth);
            void OnRGBInfo(const sensor_msgs::CameraInfoConstPtr& info);
            void OnDepthInfo(const sensor_msgs::CameraInfoConstPtr& info);
            void OnCamera(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info);
            void OnDepth(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info);
            void OnApriltags(const apriltags::AprilTagDetectionsConstPtr& detections);

            bool GetCameraCalibration(Cal3_S2& K);
            bool GetDepthCalibration(Cal3_S2& K);
            bool GetNewLandmarks(const gtsam::Pose3& cameraPose, ros::Time& timeOut,
                    const std::vector<gtsam::Landmark>& visibleLandmarks,
                    std::vector<gtsam::Landmark>& landmarksOut);

            bool GetNewLandmarksCheckerboard(const gtsam::Pose3& cameraPose, ros::Time& timeOut,
                    const std::vector<gtsam::Landmark>& visibleLandmarks,
                    std::vector<gtsam::Landmark>& landmarksOut);
            bool GetNewLandmarksApriltags(const gtsam::Pose3& cameraPose, ros::Time& timeOut,
                    const std::vector<gtsam::Landmark>& visibleLandmarks,
                    std::vector<gtsam::Landmark>& landmarksOut);
            bool GetNewLandmarksFeatures(const gtsam::Pose3& cameraPose, ros::Time& timeOut,
                    const std::vector<gtsam::Landmark>& visibleLandmarks,
                    std::vector<gtsam::Landmark>& landmarksOut);

            void SetCheckerboard(double squareSize, int numCellX, int numCellsY);

            inline const cv::Mat& GetLastImage() { return lastCVImage; }

            ros::Time GetLastImageStamp() { return lastImage->header.stamp; }

            cv_bridge::CvImagePtr GetLandmarkDisplay() { return landmarkDisplay; }

            bool MaskKeypoint(const cv::KeyPoint& kpt);

            bool GeneratePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

            inline Mode GetMode() { return mode; }

        protected:
            sensor_msgs::ImageConstPtr lastImage;
            sensor_msgs::CameraInfoConstPtr lastCameraInfo;
            sensor_msgs::ImageConstPtr lastDepthImage;
            sensor_msgs::CameraInfoConstPtr lastDepthInfo;
            apriltags::AprilTagDetectionsConstPtr lastAprilTags;
            tf::StampedTransform lastDepthTransform;
            bool hasNewData;
            ros::NodeHandle nodeHandle;
            image_transport::ImageTransport imageTransport;
            image_transport::CameraSubscriber cameraSubscriber;
            image_transport::CameraSubscriber depthSubscriber;

            typedef message_filters::sync_policies::ApproximateTime<
               sensor_msgs::Image, sensor_msgs::Image
             > SyncPolicy;
            std::shared_ptr<message_filters::Synchronizer< SyncPolicy > > sync;

            ros::Subscriber rgbInfoSubscriber;
            ros::Subscriber depthInfoSubscriber;
            std::shared_ptr<image_transport::SubscriberFilter> rgbSubscriberFilter;
            std::shared_ptr<image_transport::SubscriberFilter> depthSubscriberFilter;
            tf::TransformListener tfListener;
            ros::Subscriber aprilTagsSubscriber;
            double squareSize;
            int numCellsX;
            int numCellsY;
            Mode mode;
            bool hasNewAprilTags;
            cv::Mat lastCVImage;
            std::shared_ptr<cv::ORB> orb;
            std::shared_ptr<brisk::HarrisScaleSpaceFeatureDetector> briskDetector;
            std::shared_ptr<brisk::BriskDescriptorExtractor> briskExtractor;
            std::shared_ptr<cv::BFMatcher> featureMatcher;
            size_t maxID;
            cv_bridge::CvImagePtr landmarkDisplay;
            FeatureMatchParams params;
            bool hasDepthTransform;
    };

} /* namespace gtsam */

#endif /* FIDUCIALFRONTEND_H_ */
