/*
 * FiducialFrontEnd.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: mklingen
 */

#include <arm_slam_calib/FrontEnd.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/affine.hpp>
#include <gtsam/geometry/PinholeCamera.h>
#include <arm_slam_calib/Timer.h>
using namespace timer;

namespace gtsam
{

    FrontEnd::FrontEnd(const ros::NodeHandle& nh, const Mode& mode_, const FrontEnd::FeatureMatchParams& params_) :
            nodeHandle(nh), imageTransport(nh),
            tfListener(nh), hasNewData(false),
            numCellsX(0), numCellsY(0),
            squareSize(0.0), mode(mode_),
            hasNewAprilTags(false), maxID(0), params(params_),
            hasDepthTransform(false)
    {
        //TriangulateUnitTest();
    }

    FrontEnd::~FrontEnd()
    {

    }

    void FrontEnd::CreateFeatureMatchers()
    {
        orb = std::make_shared<cv::ORB>(32, 1.0f, 1, 31, 0);
        briskDetector = std::make_shared<brisk::HarrisScaleSpaceFeatureDetector>(30, 0, 200, 400);;
        briskExtractor = std::make_shared<brisk::BriskDescriptorExtractor>(true, true, brisk::BriskDescriptorExtractor::Version::briskV2);
        featureMatcher = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING);
    }

    void FrontEnd::SubscribeAprilTags(const std::string& topic)
    {
        aprilTagsSubscriber = nodeHandle.subscribe(topic, 10, &FrontEnd::OnApriltags, this);
    }

    void FrontEnd::OnApriltags(const apriltags::AprilTagDetectionsConstPtr& detections)
    {
        if (detections->detections.size() > 0)
        {
            ROS_INFO("Got %lu april tags", detections->detections.size());
        }

        lastAprilTags = detections;
        hasNewAprilTags = true;
    }

    void FrontEnd::SubscribeSynchronizedCameraAndDepth(const std::string& rgbTopic, const std::string& rgbInfoTopic,
                                             const std::string& depthTopic, const std::string& depthInfoTopic)
    {
        rgbSubscriberFilter.reset(new image_transport::SubscriberFilter(imageTransport, rgbTopic, 10));
        depthSubscriberFilter.reset(new image_transport::SubscriberFilter(imageTransport, depthTopic, 10));
        sync.reset(new message_filters::Synchronizer< SyncPolicy >(SyncPolicy(10), *rgbSubscriberFilter, *depthSubscriberFilter));
        sync->registerCallback(&FrontEnd::OnDepthAndRGB, this);
        rgbInfoSubscriber = nodeHandle.subscribe(rgbInfoTopic, 10,  &FrontEnd::OnRGBInfo, this);
        depthInfoSubscriber = nodeHandle.subscribe(depthInfoTopic, 10, &FrontEnd::OnDepthInfo, this);
    }

    void FrontEnd::SubscribeCamera(const std::string& topic)
    {
        cameraSubscriber = imageTransport.subscribeCamera(topic, 10, &FrontEnd::OnCamera, this);
        hasNewData = false;
        lastCameraInfo.reset();
    }

    void FrontEnd::SubscribeDepth(const std::string& topic)
    {
        depthSubscriber = imageTransport.subscribeCamera(topic, 10, &FrontEnd::OnDepth, this);
        lastDepthInfo.reset();
    }

    void FrontEnd::OnDepthAndRGB(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth)
    {
        if (lastCameraInfo.get() && lastDepthInfo.get())
        {
            OnCamera(rgb, lastCameraInfo);
            OnDepth(depth, lastDepthInfo);
        }

    }

    void FrontEnd::OnRGBInfo(const sensor_msgs::CameraInfoConstPtr& info)
    {
        lastCameraInfo = info;
    }

    void FrontEnd::OnDepthInfo(const sensor_msgs::CameraInfoConstPtr& info)
    {
        lastDepthInfo = info;
    }

    void FrontEnd::OnDepth(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
    {
        lastDepthImage = image;
        lastDepthInfo = info;

        try
        {
            if (lastImage.get())
            {
                tfListener.lookupTransform(lastDepthImage->header.frame_id,
                        lastDepthImage->header.stamp,
                        lastImage->header.frame_id,
                        lastImage->header.stamp, lastImage->header.frame_id, lastDepthTransform);

                hasDepthTransform = true;
            }
        }
        catch (tf::ExtrapolationException& e)
        {

        }
        catch (tf::LookupException& e)
        {

        }
        catch (tf::ConnectivityException& e)
        {

        }


    }

    bool FrontEnd::GeneratePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {

        if (!lastDepthImage.get() || !lastImage.get() || !hasDepthTransform)
        {
            return false;
        }

        Timer::Tick("GeneratePointCloud");
        Eigen::Isometry3d relativeTransform;
        relativeTransform.translation() = Eigen::Vector3d(lastDepthTransform.getOrigin().x(),
                                                          lastDepthTransform.getOrigin().y(),
                                                          lastDepthTransform.getOrigin().z());
        relativeTransform.linear() = Eigen::Quaterniond(lastDepthTransform.getRotation().w(),
                                                        lastDepthTransform.getRotation().x(),
                                                        lastDepthTransform.getRotation().y(),
                                                        lastDepthTransform.getRotation().z()).toRotationMatrix();

        Cal3_S2 colorCal;
        Cal3_S2 depthCal;
        GetCameraCalibration(colorCal);
        GetDepthCalibration(depthCal);

        gtsam::PinholeCamera<Cal3_S2> colorCam(gtsam::Pose3::identity(), colorCal);
        gtsam::PinholeCamera<Cal3_S2> depthCam(gtsam::Pose3(relativeTransform.matrix()), depthCal);


        cv_bridge::CvImagePtr depthImage;
        cv_bridge::CvImagePtr colorImage;
        bool floatDepth = false;
         try
         {
             colorImage = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::BGR8);
         }
         catch (cv_bridge::Exception& e)
         {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return false;
         }
         try
         {
             if (lastDepthImage->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
             {
                 depthImage = cv_bridge::toCvCopy(lastDepthImage, sensor_msgs::image_encodings::TYPE_16UC1);
             }
             else if (lastDepthImage->encoding == sensor_msgs::image_encodings::MONO16)
             {
                 depthImage = cv_bridge::toCvCopy(lastDepthImage, sensor_msgs::image_encodings::MONO16);
             }
             else if (lastDepthImage->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
             {
                 depthImage = cv_bridge::toCvCopy(lastDepthImage, sensor_msgs::image_encodings::TYPE_32FC1);
                 floatDepth = true;
             }
             else
             {
                 ROS_ERROR("Unrecognized depth image format %s", lastDepthImage->encoding.c_str());
                 return false;
             }
         }
         catch (cv_bridge::Exception& e)
         {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return false;
         }

         cloud->points.clear();
         float d = 0;
         uint16_t bytes = 0;
         bool valid = false;
         cv::Vec3b color;
         pcl::PointXYZRGB pt;
         for (int r = 0; r < depthImage->image.rows; r+=8)
         {
             for (int c = 0; c < depthImage->image.cols; c+=8)
             {

                 if (floatDepth)
                 {
                     d = depthImage->image.at<float>(r, c);
                     valid = !std::isnan(d) && d > 0;
                 }
                 else
                 {
                     bytes = depthImage->image.at<uint16_t>(r, c);
                     d = (1.0f / 1000.0f) * bytes;
                     valid = bytes > 0;
                 }

                 if (!valid)
                 {
                     continue;
                 }

                 gtsam::Point3 pC = depthCam.backproject(gtsam::Point2(c, r), d);

                 std::pair<gtsam::Point2, bool> colorUV = colorCam.projectSafe(pC);

                 if (!colorUV.second || (colorUV.first.x() < 0 ||
                      colorUV.first.y() < 0 || colorUV.first.x() > colorCal.px() * 2 ||
                      colorUV.first.y() > colorCal.py() * 2))
                 {
                     continue;
                 }

                 color = colorImage->image.at<cv::Vec3b>(colorUV.first.y(), colorUV.first.x());
                 pt.x = pC.x();
                 pt.y = pC.y();
                 pt.z = pC.z();
                 pt.r = color.val[2];
                 pt.g = color.val[1];
                 pt.b = color.val[0];
                 cloud->points.push_back(pt);
             }
         }
         lastDepthImage.reset();
         Timer::Tock("GeneratePointCloud");
         return true;
    }

    void FrontEnd::SetCheckerboard(double squareSize_,int numCellsX_, int numCellsY_)
    {
        numCellsX = numCellsX_;
        numCellsY = numCellsY_;
        squareSize = squareSize_;
    }

    void FrontEnd::OnCamera(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
    {
        lastImage = image;
        lastCameraInfo = info;
        hasNewData = true;

        cv_bridge::CvImagePtr cvImage;
         try
         {
             cvImage = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::BGR8);
         }
         catch (cv_bridge::Exception& e)
         {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return;
         }

         lastCVImage = cvImage->image;
    }

    bool FrontEnd::GetCameraCalibration(Cal3_S2& calibration)
    {
        if (!lastCameraInfo.get())
            return false;

        calibration = Cal3_S2(lastCameraInfo->K[0], lastCameraInfo->K[4], 0.0, lastCameraInfo->K[2], lastCameraInfo->K[5]);
        return true;
    }

    bool FrontEnd::GetDepthCalibration(Cal3_S2& K)
    {
        if (!lastDepthInfo.get())
            return false;

        K = Cal3_S2(lastDepthInfo->K[0], lastDepthInfo->K[4], 0.0, lastDepthInfo->K[2], lastDepthInfo->K[5]);
        return true;
    }

    bool FrontEnd::GetNewLandmarksCheckerboard(const gtsam::Pose3& cameraPose, ros::Time& timeOut, const std::vector<gtsam::Landmark>& visibleLandmarks, std::vector<gtsam::Landmark>& landmarksOut)
    {
        cv_bridge::CvImagePtr cvImage;
         try
         {
             cvImage = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::MONO8);
         }
         catch (cv_bridge::Exception& e)
         {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return false;
         }

         timeOut = lastImage->header.stamp;

         ROS_INFO("Searching for %d by %d chessboard", numCellsX, numCellsY);
         cv::Size chessDims(numCellsX, numCellsY);
         std::vector<cv::Vec2f> corners;
         if(!cv::findChessboardCorners(cvImage->image, chessDims, corners))
         {
             ROS_ERROR("Didn't find any chessboard corners!");
             return false;
         }


         cv::cornerSubPix(cvImage->image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
         cv::Mat chessImage = cvImage->image;

         std::vector<cv::Vec3f> objectPoints;
         for (int y = 0; y < numCellsY; y++)
         {
             for (int x = 0; x < numCellsX; x++)
             {
                 objectPoints.push_back(cv::Vec3f(x * squareSize, y * squareSize, 0.0f));
             }
         }

         cv::Mat distCoeffs(4,1,cv::DataType<float>::type);

         distCoeffs.at<float>(0) = lastCameraInfo->D[0];
         distCoeffs.at<float>(1) = lastCameraInfo->D[1];
         distCoeffs.at<float>(2) = lastCameraInfo->D[2];
         distCoeffs.at<float>(3) = lastCameraInfo->D[3];

         cv::Mat intrinsics(3, 3, cv::DataType<float>::type);
         int k = 0;
         for (int i = 0; i < 3; i++)
         {
             for (int j = 0; j < 3; j++)
             {
                 intrinsics.at<float>(i, j) = lastCameraInfo->K.elems[k];
                 k++;
             }
         }

         std::cout << intrinsics<< std::endl;
         cv::Mat rotation(3,1,cv::DataType<float>::type);
         cv::Mat translation(3,1,cv::DataType<float>::type);
         cv::solvePnP(objectPoints, corners, intrinsics, distCoeffs, rotation, translation, false, CV_EPNP);
         std::cout << rotation << std::endl;
         std::cout << translation << std::endl;
         cv::Vec3f rotationVec((float)rotation.at<double>(0, 0), (float)rotation.at<double>(1, 0), (float)rotation.at<double>(2, 0));
         cv::Vec3f translationVec((float)translation.at<double>(0, 0), (float)translation.at<double>(1, 0), (float)translation.at<double>(2, 0));
         std::cout << rotationVec << std::endl;
         std::cout << translationVec << std::endl;
         cv::Affine3f transform(rotationVec, translationVec);
         std::cout << transform.matrix << std::endl;
         ROS_INFO("Got %lu points", objectPoints.size());
         for (size_t i = 0; i < objectPoints.size(); i++)
         {
             const cv::Vec3f& pt = objectPoints.at(i);
             cv::Vec3f landmarkPos = transform * pt;
             Landmark landmark;
             landmark.id = i;
             landmark.position = gtsam::Point3(landmarkPos.val[0], landmarkPos.val[1], landmarkPos.val[2]);
             const cv::Vec2f& uv = corners.at(i);
             landmark.observations.push_back(gtsam::Point2(uv.val[0], uv.val[1]));
             landmarksOut.push_back(landmark);
         }

         hasNewData = false;
         return true;
    }

    bool  FrontEnd::GetNewLandmarksApriltags(const gtsam::Pose3& pose, ros::Time& timeOut,  const std::vector<gtsam::Landmark>& visibleLandmarks, std::vector<gtsam::Landmark>& landmarksOut)
    {
        if (!lastAprilTags.get())
        {
            hasNewAprilTags = false;
            return false;
        }

        if (lastAprilTags->detections.size() == 0)
        {
            hasNewAprilTags = false;
            lastAprilTags.reset();
            return false;
        }

        try
        {
            landmarkDisplay = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }


        timeOut = lastAprilTags->header.stamp;

        const auto& detections = lastAprilTags->detections;

        Cal3_S2 cal;
        GetCameraCalibration(cal);
        PinholeCamera<Cal3_S2> camera(gtsam::Pose3::identity(), cal);
        for (size_t i = 0; i < detections.size(); i++)
        {
            const auto& detection = detections.at(i);
            if (detection.id > 40)
            {
                continue;
            }

            if (detection.pose.position.z < 0)
            {
                continue;
            }

            gtsam::Landmark landmark;
            landmark.id = detection.id;
            landmark.position = gtsam::Point3(detection.pose.position.x, detection.pose.position.y, detection.pose.position.z);
            landmark.observations.push_back(gtsam::Point2(detection.corners2d.at(0).x, detection.corners2d.at(0).y));
            landmark.isTriangulated = true;
            cv::circle(landmarkDisplay->image, cv::Point2f(detection.corners2d.at(0).x, detection.corners2d.at(0).y), 5, cv::Scalar(0, 255, 0), 2);
            std::stringstream ss;
            ss << detection.id;
            cv::putText(landmarkDisplay->image, ss.str(),
                    cv::Point2f(detection.corners2d.at(0).x + 5, detection.corners2d.at(0).y + 5),
                    cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
            //landmark.observations.push_back(uv);
            landmarksOut.push_back(landmark);
        }
        ROS_INFO("Detected %lu apriltags", landmarksOut.size());
        hasNewAprilTags = false;
        lastAprilTags.reset();
        return landmarksOut.size() > 0;

    }


    bool FrontEnd::MaskKeypoint(const cv::KeyPoint& kpt)
    {
        return kpt.pt.x > params.maxX || kpt.pt.y > params.maxY ||
               kpt.pt.x < params.minX || kpt.pt.y < params.minY;
    }

    bool FrontEnd::GetNewLandmarksFeatures(const gtsam::Pose3& cameraPose, ros::Time& timeOut,  const std::vector<gtsam::Landmark>& visibleLandmarks, std::vector<gtsam::Landmark>& landmarksOut)
    {

         Timer::Tick("GetNewLandmarks");
         gtsam::Cal3_S2 calib;
         GetCameraCalibration(calib);
         gtsam::PinholeCamera<gtsam::Cal3_S2> cam(cameraPose, calib);
         std::vector< std::pair<gtsam::Point2, bool> > projections;
         cv_bridge::CvImagePtr cvImage;
         cv_bridge::CvImagePtr colorImage;
         try
         {
             cvImage = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::MONO8);
             colorImage = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::BGR8);
             landmarkDisplay = colorImage;
         }
         catch (cv_bridge::Exception& e)
         {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return false;
         }

         timeOut = lastImage->header.stamp;


         if (params.drawGrid)
         {
             for (float x = -3; x < 3; x += 0.1)
             {
                 for (float y = -3; y < 3; y += 0.1)
                 {
                     gtsam::Point3 gridPoint(x, y, -1.0);

                     try
                     {
                         std::pair<gtsam::Point2, bool> proj = cam.projectSafe(gridPoint);

                         if (proj.second)
                         {
                             cv::circle(landmarkDisplay->image, cv::Point2f(proj.first.x(), proj.first.y()), 1, cv::Scalar(255, 255, 255), 1);
                         }
                     }
                     catch(CheiralityException& e)
                     {

                     }
                 }
             }
         }

         if (!params.doFeatureMatching)
         {
             Timer::Tock("GetNewLandmarks");
             return false;
         }

         std::vector<cv::KeyPoint> keypoints;
         cv::Mat descriptors;
         cv::Mat mask;
         ROS_INFO("Detecting...");
         briskDetector->detect(cvImage->image, keypoints);
         briskExtractor->compute(cvImage->image, keypoints, descriptors);
         if (!keypoints.size())
         {
             Timer::Tock("GetNewLandmarks");
             return false;
         }
         ROS_INFO("Detected %lu keypoints with %d x %d descriptors", keypoints.size(), descriptors.rows, descriptors.cols);

         ROS_INFO("Matching %lu visible landmarks.", visibleLandmarks.size());
         std::vector<cv::DMatch> matches;
         std::map<int, float> ratios;
         std::map<int, float> imgDists;


         if (visibleLandmarks.size() > 0)
         {
             cv::Mat visibleDescriptors(cv::Size(descriptors.cols, visibleLandmarks.size()), descriptors.type());
             for (size_t k = 0; k < visibleLandmarks.size(); k++)
             {
                 visibleLandmarks.at(k).descriptor.copyTo(visibleDescriptors.row(k));
                 try
                 {
                     std::pair<gtsam::Point2, bool> projection = cam.projectSafe(visibleLandmarks.at(k).position);
                     if (projection.second && params.drawCandidates)
                     {
                         cv::circle(landmarkDisplay->image, cv::Point2f(projection.first.x(), projection.first.y()), 1, cv::Scalar(0, 255, 255), 1);
                     }

                     projections.push_back(projection);
                 }
                 catch(CheiralityException& e)
                 {
                     projections.push_back(std::make_pair(gtsam::Point2(), false));
                 }
             }

             brisk::BruteForceMatcher bfMatcher;
             std::vector<std::vector<cv::DMatch> > knnMatchesA;
             std::vector<std::vector<cv::DMatch> > knnMatchesB;
             std::vector<std::vector<cv::DMatch> > knnMatches;
             bfMatcher.knnMatch(descriptors, visibleDescriptors, knnMatchesA, 2);
             bfMatcher.knnMatch(visibleDescriptors, descriptors, knnMatchesB, 2);

             for (size_t a = 0; a < knnMatchesA.size(); a++)
             {
                 const std::vector<cv::DMatch>& matchA = knnMatchesA.at(a);
                 if (matchA.size() == 0) continue;

                 for (size_t b = 0; b < knnMatchesB.size(); b++)
                 {
                     if (knnMatchesB.at(b).size() == 0) continue;

                     if (knnMatchesB.at(b).at(0).queryIdx == matchA.at(0).trainIdx &&
                         knnMatchesB.at(b).at(0).trainIdx == matchA.at(0).queryIdx)
                     {
                         knnMatches.push_back(matchA);
                         break;
                     }
                 }
             }

             ROS_WARN("There are %lu symmetric matches", knnMatches.size());

             for (size_t m = 0; m < knnMatches.size(); m++)
             {
                 if (knnMatches[m].size() > 1)
                 {
                     size_t query = knnMatches[m].at(0).queryIdx;
                     size_t train = knnMatches[m].at(0).trainIdx;
                     ratios[query] =  knnMatches[m].at(1).distance / knnMatches[m].at(0).distance;
                     float imgDist = (gtsam::Point2(keypoints.at(query).pt.x, keypoints.at(query).pt.y) - projections.at(train).first).norm();
                     imgDists[query] = imgDist;
                     matches.push_back(knnMatches[m].at(0));
                 }
                 else
                 {
                     ratios[knnMatches[m].at(0).queryIdx]  = (100.0f);
                     imgDists[knnMatches[m].at(0).queryIdx]  = (10000);
                     matches.push_back(knnMatches[m].at(0));
                 }
             }
         }

         ROS_INFO("Associating %lu matches", matches.size());
         std::map<int, int> matchMap;
         std::map<int, int> reverseMatchMap;
         size_t numSuccessfulMatches = 0;
         size_t numRejections = 0;
         for (size_t i = 0; i < matches.size(); i++)
         {
             //ROS_INFO("%d -> %lu : %f, %f, %f", matches[i].queryIdx, visibleLandmarks[matches[i].trainIdx].id, matches[i].distance, ratios[matches[i].queryIdx], imgDists[matches[i].queryIdx]);
             if    (reverseMatchMap.find(matches[i].trainIdx) == reverseMatchMap.end() &&
                     matches[i].distance < params.hammingThreshold &&
                     ratios[matches[i].queryIdx] > params.ratioThreshold &&
                     imgDists[matches[i].queryIdx] < params.reprojectionThreshold)
             {
                 matchMap[matches[i].queryIdx] = matches[i].trainIdx;
                 reverseMatchMap[matches[i].trainIdx] = matches[i].queryIdx;
                 numSuccessfulMatches++;

                 if (params.drawGoodFeatures)
                 {
                     cv::circle(landmarkDisplay->image, keypoints.at(matches[i].queryIdx).pt, keypoints.at(matches[i].queryIdx).size  * 0.5f, cv::Scalar(0, 255, 0), 2);
                     std::stringstream ss;
                     ss << visibleLandmarks[matches[i].trainIdx].id;
                     cv::putText(landmarkDisplay->image, ss.str(),
                             cv::Point(keypoints.at(matches[i].queryIdx).pt.x + 5, keypoints.at(matches[i].queryIdx).pt.y + keypoints.at(matches[i].queryIdx).size  * 0.5f + 5),
                             cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
                 }


                 if (params.drawGoodMatches)
                 {
                     std::pair<gtsam::Point2, bool> matchProjection = projections.at(matches[i].trainIdx);
                     if (matchProjection.second)
                     {
                         cv::Scalar color = cv::Scalar(0, 255, 0);

                         if (!visibleLandmarks.at(matches[i].trainIdx).isInGraph)
                         {
                             color = cv::Scalar(255, 255, 0);
                         }

                         cv::line(landmarkDisplay->image, keypoints.at(matches[i].queryIdx).pt, cv::Point2f(matchProjection.first.x(), matchProjection.first.y()), color, 1);
                     }
                 }
             }
             else
             {
                 if (params.drawBadFeatures)
                 {
                     cv::circle(landmarkDisplay->image, keypoints.at(matches[i].queryIdx).pt, keypoints.at(matches[i].queryIdx).size * 0.5f, cv::Scalar(0, 0, 255), 1);
                 }

                 if (params.drawBadMatches)
                 {
                     std::pair<gtsam::Point2, bool> matchProjection = projections.at(matches[i].trainIdx);
                     if (matchProjection.second)
                     {
                         cv::line(landmarkDisplay->image, keypoints.at(matches[i].queryIdx).pt, cv::Point2f(matchProjection.first.x(), matchProjection.first.y()), cv::Scalar(0, 0, 255), 1);
                     }
                 }
                 numRejections++;
             }
         }

         ROS_WARN("%lu successful matches. %lu rejections.", numSuccessfulMatches, numRejections);

         for (size_t i = 0; i < keypoints.size(); i++)
         {
             if (matchMap.find((int)i) == matchMap.end())
             {
                 matchMap[(int)i] = -1;
             }
         }

         Cal3_S2 cal;
         GetCameraCalibration(cal);

         for (auto it = matchMap.begin(); it != matchMap.end(); it++)
         {
             int myIdx = it->first;
             int matchIdx = it->second;
             cv::KeyPoint keypt = keypoints.at(myIdx);
             Landmark landmark;
             landmark.isTriangulated = false;
             if (matchIdx == -1)
             {
                 landmark.id = maxID;
                 descriptors.row(myIdx).copyTo(landmark.descriptor);
                 cv::Vec3b cb = colorImage->image.at<cv::Vec3b>(keypoints[myIdx].pt);
                 landmark.color = gtsam::Point3(cb.val[2] / 255.0f, cb.val[1] / 255.0f, cb.val[0] / 255.0f);
                 landmark.isTriangulated = false;
                 landmark.isNew = true;
                 landmark.isInGraph = false;
                 maxID++;
             }
             else
             {
                 const Landmark& matchLandmark = visibleLandmarks[matchIdx];
                 landmark.isTriangulated = matchLandmark.isTriangulated;
                 landmark.isNew = false;
                 landmark.id = matchLandmark.id;
                  matchLandmark.descriptor.copyTo(landmark.descriptor);
             }

             landmark.observations.push_back(gtsam::Point2(keypt.pt.x, keypt.pt.y));
             if (!landmark.isTriangulated)
             {
                 gtsam::Point2 uv = cal.calibrate(gtsam::Point2(keypt.pt.x, keypt.pt.y));
                 double depth = 2.5;
                 landmark.position = gtsam::Point3(uv.x() * depth, uv.y() * depth, depth);
             }
             landmarksOut.push_back(landmark);
         }

         hasNewData = false;
         Timer::Tock("GetNewLandmarks");
         return true;
    }




    bool FrontEnd::GetNewLandmarks(const gtsam::Pose3& cameraPose, ros::Time& timeOut,  const std::vector<gtsam::Landmark>& visibleLandmarks, std::vector<gtsam::Landmark>& landmarksOut)
    {
        if (mode == Mode_Checkerboard)
        {
            return HasNewData() && GetNewLandmarksCheckerboard(cameraPose, timeOut, visibleLandmarks, landmarksOut);
        }
        else if (mode == Mode_Apriltags)
        {
            return HasNewApriltags() && GetNewLandmarksApriltags(cameraPose, timeOut, visibleLandmarks, landmarksOut);
        }
        else
        {
            return HasNewData() && GetNewLandmarksFeatures(cameraPose, timeOut, visibleLandmarks, landmarksOut);
        }
    }

    void FrontEnd::FeatureMatchParams::InitializeNodeHandle(const ros::NodeHandle& nh)
    {
        nh.param("mask_min_x", minX, 0.0);
        nh.param("mask_max_x", maxX, 99999.0);
        nh.param("mask_min_y", minY, 0.0);
        nh.param("mask_max_y", maxY, 99999.0);
        nh.param("do_feature_matching", doFeatureMatching, true);
        nh.param("hamming_threshold", hammingThreshold, 60.0);
        nh.param("ratio_threshold", ratioThreshold, 1.25);
        nh.param("draw_good_features", drawGoodFeatures, true);
        nh.param("draw_good_matches", drawGoodMatches, true);
        nh.param("draw_candidates", drawCandidates, false);
        nh.param("draw_bad_features", drawBadFeatures, false);
        nh.param("draw_bad_matches", drawBadMatches, false);
        nh.param("draw_grid", drawGrid, true);
    }

} /* namespace gtsam */
