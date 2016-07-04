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
#include <r3/util/CatkinResourceRetriever.h>

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
    params.simulated = true;

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
    std::string trajFile = "";
    nh.param("rgb_topic", rgbTopic, rgbTopic);
    nh.param("rgb_info", rgbInfo, rgbInfo);
    nh.param("depth_topic", depthtopic, depthtopic);
    nh.param("depth_info", depthInfo, depthInfo);
    nh.param("trajectory_file", trajFile, trajFile);

    if (trajFile == "")
    {
        calib.CreateSimTrajectory();
    }
    else
    {
        calib.LoadSimTrajectory(trajFile);
    }
    for (size_t i = 0; i < 1000; i++)
    {
        calib.SimulateImageStep(0);
    }

    // TODO!!
    //calib.InitializeFiducial("/camera/rgb/image_rect_color", 0.025, 7, 9);
    //calib.InitializeFiducial("/camera/rgb/image_rect_color", 0.115, 7, 5);
    //calib.InitializeAprilTags("/camera/rgb/image_raw", "/apriltags/detections");
    calib.InitializeFeatures(rgbTopic, rgbInfo, depthtopic, depthInfo);
    //calib.InitializeUnsynchronizedDepth("/camera/depth/image_rect_raw");
    std::string jointStateTopic = "/sim_joints";
    nh.param("joint_state_topic", jointStateTopic, jointStateTopic);
    // TODO!!;
    calib.InitializeJointRecorder(jointStateTopic);

    ros::Rate hz(30);

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
    bool drawTraj = true;
    bool drawObs = false;
    bool drawCamera = true;
    bool drawPointCloud = true;
    bool drawMarginal = true;
    bool printTiming = false;

    nh.param("draw_landmarks", drawLandmarks, drawLandmarks);
    nh.param("draw_trajectory", drawTraj, drawTraj);
    nh.param("draw_observation", drawObs, drawObs);
    nh.param("draw_extrinsic_marginal", drawMarginal, drawMarginal);
    nh.param("draw_pointclouds", drawPointCloud, drawPointCloud);
    nh.param("print_timing", printTiming, printTiming);

    bool savePointclouds = false;
    std::string pointcloudFile = "point_cloud.pcd";
    nh.param("save_pointcloud", savePointclouds, savePointclouds);
    nh.param("save_pointcloud_file", pointcloudFile, pointcloudFile);
    bool writeErrors = false;
    nh.param("save_stats", writeErrors, writeErrors);
    std::string errorDir = ".";
    nh.param("stats_dir", errorDir, errorDir);
    std::string postfix = "";
    nh.param("stats_postfix", postfix, postfix);

    std::ofstream landmarkFile("landmarks.txt", std::ios::out);
    std::ofstream offsetFile("offsets.txt", std::ios::out);
    std::ofstream reprojectionFile("reproj_error.txt", std::ios::out);
    std::ofstream extrinsicFile("extrinsic_errors.txt", std::ios::out);
    std::ofstream relativeTrajectory("relative_trajectory.txt", std::ios::out);

    Eigen::Isometry3d initialPose = Eigen::Isometry3d(calib.GetCameraPose(calib.GetSimTrajectory().at(0), calib.GetSimExtrinsic()).matrix());
    double initialTime = ros::Time::now().toSec();

    size_t simIter = 0;
    while (ros::ok())
    {
        calib.SimulateImageStep(simIter);
        simIter++;

        if (simIter >= calib.GetSimTrajectory().size())
            break;

        gtsam::Vector currentConfig = calib.GetLatestJointAngles();
        if ((lastConfig - currentConfig).norm() > keyframeThreshold || firstIter)
        {
            if(calib.RealFrontEndStep() && !firstIter)
            {
                calib.OptimizeStep();

                if (params.saveImages)
                {
                    calib.ShowReprojectionErrors();
                }

                if (writeErrors)
                {
                    std::vector<double> errors;
                    calib.CalculateCurrentErrors(errors);

                    if (errors.size() > 3)
                    {
                        std::sort(errors.begin(), errors.end());
                        double median = errors.at(errors.size() / 2);
                        reprojectionFile << median << std::endl;
                        ROS_WARN("Median error: %f",  median);
                    }

                    //calib.SaveGraph();

                    gtsam::Vector err = calib.ComputeLatestJointAngleOffsets(0);

                    for(size_t j = 0; j < 6; j++)
                    {
                        offsetFile << err(j);

                        offsetFile << " ";
                    }
                    gtsam::Vector err_sim = calib.ComputeLatestGroundTruthSimOffsets(0);

                    for(size_t j = 0; j < 6; j++)
                    {
                        offsetFile << err_sim(j);

                        if (j < 5)
                        {
                            offsetFile << " ";
                        }
                    }

                    offsetFile << std::endl;


                    gtsam::Pose3 extCur = calib.currentEstimate.at<gtsam::Pose3>(gtsam::Symbol('K', 0));
                    gtsam::Quaternion extCurQ = extCur.rotation().toQuaternion();
                    gtsam::Quaternion simExtQ =  calib.GetSimExtrinsic().rotation().toQuaternion();
                    extrinsicFile << calib.GetSimExtrinsic().translation().x() << " "
                                     <<  calib.GetSimExtrinsic().translation().y() << " "
                                     <<  calib.GetSimExtrinsic().translation().z() << " "
                                     << simExtQ.x() << " "
                                     << simExtQ.y() << " "
                                     << simExtQ.z() << " "
                                     << simExtQ.w() << " ";

                    extrinsicFile << extCur.translation().x() << " "
                                     << extCur.translation().y() << " "
                                     << extCur.translation().z() << " "
                                     << extCurQ.x() << " "
                                     << extCurQ.y() << " "
                                     << extCurQ.z() << " "
                                     << extCurQ.w();
                    extrinsicFile << std::endl;
                }

                calib.PrintLandmarkStats(landmarkFile);
            }

            lastConfig = currentConfig;
            firstIter = false;
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
        calib.DrawState(iter, 0, calib.initialEstimate, 0.0, 0.8, 0.8, 1.0, false, true, false, false, false, false);
        calib.DrawState(iter, 1, calib.groundTruth,     0.0, 0.8, 0.0, 1.0, false, true, false, false, false, false);
        calib.DrawState(iter, 2, calib.currentEstimate, 0.8, 0.0, 0.0, 1.0, drawLandmarks, drawTraj, drawObs, drawCamera, drawPointCloud, drawMarginal);
        calib.GetArm()->setPositions(calib.GetSimTrajectory().at(simIter));
        calib.UpdateViewer();

        hz.sleep();
        ros::spinOnce();

        if (printTiming)
            Timer::PrintStats();
    }

    if (savePointclouds)
    {
        calib.SaveStitchedPointClouds(pointcloudFile);
    }

    if (writeErrors)
    {
        calib.PrintSimErrors(errorDir, "");
    }

    auto traj = calib.GetTrajectory();

    for (size_t trajIter = 0; trajIter < traj.size(); trajIter++)
    {
        try
        {
            Eigen::Isometry3d gtPose = initialPose.inverse() * Eigen::Isometry3d(calib.GetGroundTruthPose(trajIter).matrix());
            Eigen::Isometry3d currentPose = initialPose.inverse() * Eigen::Isometry3d(calib.GetEstimatePose(trajIter).matrix());
            Eigen::Isometry3d initEst = initialPose.inverse() * Eigen::Isometry3d(calib.GetInitialPose(trajIter).matrix());

            double time = calib.GetTime(trajIter) - initialTime;
            relativeTrajectory << time << " " << gtPose.translation()(0) <<
                                          " " << gtPose.translation()(1) <<
                                          " " << gtPose.translation()(2) <<
                                          " " << currentPose.translation()(0) <<
                                          " " << currentPose.translation()(1) <<
                                          " " << currentPose.translation()(2) <<
                                          " " << initEst.translation()(0) <<
                                          " " << initEst.translation()(1) <<
                                          " " << initEst.translation()(2) << std::endl;
        }
        catch (gtsam::ValuesKeyDoesNotExist& ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }

    offsetFile.close();
}

