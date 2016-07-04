// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Inference and optimization
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// SFM-specific factors
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h> // does calibration !
#include <gtsam/slam/ProjectionFactor.h>

#include <dart/dart.h>
#include <stdio.h>
#include <ros/ros.h>
#include <fstream>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <arm_slam_calib/EncoderFactor.h>
#include <arm_slam_calib/RobotProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>

#include <map>

#include <arm_slam_calib/ArmSlamCalib.h>
#include <arm_slam_calib/FrontEnd.h>

#include <arm_slam_calib/Timer.h>

using namespace timer;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_calib_sim");
    ros::NodeHandle nh("~");

    tf::TransformBroadcaster transformBroadcaster;


    gtsam::ArmSlamCalib::Params params;
    params.InitializeNodehandleParams(nh);

    gtsam::ArmSlamCalib calib(nh, std::make_shared<std::mutex>(), params);

    std::vector<std::string> joints;
    joints.push_back("mico_joint_1");
    joints.push_back("mico_joint_2");
    joints.push_back("mico_joint_3");
    joints.push_back("mico_joint_4");
    joints.push_back("mico_joint_5");
    joints.push_back("mico_joint_6");
    nh.param("free_joints", joints, joints);

    std::string cameraName = "mico_end_effector";
    nh.param("camera_mount_link", cameraName, cameraName);

    std::string urdf = "package://ada_description/robots/mico.urdf";
    nh.param("urdf", urdf, urdf);

    calib.InitRobot(urdf, joints, cameraName);

    std::string rgbTopic = "/camera/rgb/image_rect_color";
    std::string rgbInfo = "/camera/rgb/camera_info";
    std::string depthtopic = "/camera/depth/image_rect_raw";
    std::string depthInfo = "/camera/depth/camera_info";
    nh.param("rgb_topic", rgbTopic, rgbTopic);
    nh.param("rgb_info", rgbInfo, rgbInfo);
    nh.param("depth_topic", depthtopic, depthtopic);
    nh.param("depth_info", depthInfo, depthInfo);
    // TODO!!
    //calib.InitializeFiducial("/camera/rgb/image_rect_color", 0.025, 7, 9);
    //calib.InitializeFiducial("/camera/rgb/image_rect_color", 0.115, 7, 5);
    //calib.InitializeAprilTags("/camera/rgb/image_raw", "/apriltags/detections");
    calib.InitializeFeatures(rgbTopic, rgbInfo, depthtopic, depthInfo);
    //calib.InitializeUnsynchronizedDepth("/camera/depth/image_rect_raw");
    std::string jointStateTopic = "/mico_arm_driver/out/joint_state";
    nh.param("joint_state_topic", jointStateTopic, jointStateTopic);
    // TODO!!;
    calib.InitializeJointRecorder(jointStateTopic);

    ros::Rate hz(60);

    ROS_INFO("Warming up viewer...");
    for (size_t i = 0; i < 10; i++)
    {
        hz.sleep();
        calib.UpdateViewer();
        ros::spinOnce();
    }



    ROS_INFO("Starting loop.");

    double keyframeThreshold = 0.05;
    nh.param("keyframe_threshold", keyframeThreshold, keyframeThreshold);
    bool firstIter = false;
    gtsam::Vector lastConfig = calib.GetLatestJointAngles();
    size_t iter = 0;
    bool drawLandmarks = true;
    bool drawTraj = false;
    bool drawObs = false;
    bool drawCamera = true;
    bool drawPointCloud = true;
    bool drawMarginal = true;
    nh.param("draw_landmarks", drawLandmarks, drawLandmarks);
    nh.param("draw_trajectory", drawTraj, drawTraj);
    nh.param("draw_observation", drawObs, drawObs);
    nh.param("draw_extrinsic_marginal", drawMarginal, drawMarginal);
    nh.param("draw_pointclouds", drawPointCloud, drawPointCloud);

    std::ofstream offsetFile("offsets.txt", std::ios::out);
    std::ofstream trajFile("traj.txt", std::ios::out);
    while (ros::ok())
    {
        gtsam::Vector currentConfig = calib.GetLatestJointAngles();
        for(size_t j = 0; j < 6; j++)
        {
            trajFile << currentConfig(j);

            if (j < 5)
            {
                trajFile << " ";
            }
        }
        trajFile << std::endl;

        if ((lastConfig - currentConfig).norm() > keyframeThreshold || firstIter)
        {
            if(calib.RealFrontEndStep())
            {
                if (!firstIter && params.doOptimize)
                {
                    calib.OptimizeStep();

                    std::vector<double> errors;
                    calib.CalculateCurrentErrors(errors);
                    if (errors.size() > 3)
                    {
                        std::sort(errors.begin(), errors.end());
                        double median = errors.at(errors.size() / 2);
                        ROS_WARN("Median error: %f",  median);
                    }
                    if (params.saveImages)
                    {
                        calib.ShowReprojectionErrors();
                    }
                    //calib.SaveGraph();

                    //TODO:
                    gtsam::Vector err = calib.ComputeLatestJointAngleOffsets(0);

                    for(size_t j = 0; j < 6; j++)
                    {
                        offsetFile << err(j);

                        if (j < 5)
                        {
                            offsetFile << " ";
                        }
                    }
                    offsetFile << std::endl;
                }
            }

            lastConfig = currentConfig;
            firstIter = false;
            std::cout << "Extrinsic: " << calib.GetCurrentExtrinsic() << std::endl;
            std::cout << "Intrinsic: " << calib.GetIntrinsic().matrix() << std::endl;
        }

        iter++;
        calib.PublishLastPointCloud();

        tf::StampedTransform tf;
        tf.child_frame_id_ = "/camera_rgb_optical_frame";
        tf.frame_id_ = "/map";
        tf.stamp_ = ros::Time::now();

        gtsam::Pose3 cameraPose = calib.GetCameraPose(currentConfig);

        tf::Vector3 origin(cameraPose.translation().x(), cameraPose.translation().y(), cameraPose.translation().z());
        gtsam::Matrix3 R = cameraPose.rotation().matrix();
        tf.setOrigin(origin);
        gtsam::Quaternion q(cameraPose.rotation().toQuaternion());
        q.normalize();
        tf::Quaternion quat(q.x(), q.y(), q.z(), q.w());
        tf.setRotation(quat);
        transformBroadcaster.sendTransform(tf);
        calib.GetArm()->setPositions(currentConfig);
        //calib.DrawState(iter, 0, calib.initialEstimate, 0.0, 0.8, 0.8, 1.0);
        calib.DrawState(iter, 1, calib.currentEstimate, 0.8, 0.0, 0.0, 1.0, drawLandmarks, drawTraj, drawObs, drawCamera, drawPointCloud, drawMarginal);
        calib.GetArm()->setPositions(currentConfig);
        calib.UpdateViewer();

        hz.sleep();
        ros::spinOnce();
        Timer::PrintStats();
    }
    offsetFile.close();
}

