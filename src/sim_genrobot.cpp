/*
 * touch_filter_sim.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: mklingen
 */

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
#include <arm_slam_calib/RobotGenerator.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <map>

#include <arm_slam_calib/ArmSlamCalib.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_calib_sim");
    ros::NodeHandle nh("~");
    int num_dofs = 6;
    nh.param("num_dofs", num_dofs, num_dofs);
    std::string dataDir = "~/.ros/";
    nh.param("data_dir", dataDir, dataDir);
    std::string dataPostfix = "";
    nh.param("data_postfix", dataPostfix, dataPostfix);

    int seed = 0;

    nh.param("seed", seed, seed);
    srand(seed);

    gtsam::ArmSlamCalib::Params params;
    params.encoderNoiseLevel = 0.03;
    params.optimizationMode = gtsam::ArmSlamCalib::ISAM;
    params.simulated = true;
    params.InitializeNodehandleParams(nh);
    gtsam::Pose3 gt_pose = gtsam::Pose3(gtsam::Rot3::rodriguez(0, 1.57, 0), gtsam::Point3(0, 0, 0));
    params.extrinsicInitialGuess = gtsam::Pose3(gtsam::Rot3::rodriguez(0.0, 1.57, 0.0), gtsam::Point3(0.1, 0.1, 0.1));

    gtsam::ArmSlamCalib calib(nh, std::make_shared<std::mutex>(), params);
    calib.SetSimExtrinsic(gt_pose);
    size_t base_dofs = 6;
    std::vector<std::string> joints;
    dart::dynamics::SkeletonPtr genSkeleton = dart::RobotGenerator::GenerateRobot(num_dofs, 0.5, 0.7, 0.01, 0.1, false, base_dofs);
    for (size_t i = 0; i < genSkeleton->getNumDofs(); i++)
    {
        joints.push_back(genSkeleton->getDof(i)->getName());
    }
    std::string cameraName = genSkeleton->getBodyNode(genSkeleton->getNumBodyNodes() - 1)->getName();
    calib.InitRobot(genSkeleton, joints, cameraName);
    calib.CreateSimulation("");

    ros::Rate hz(200);


    std::ofstream errorstream(dataDir + std::string("/error") + dataPostfix + std::string(".txt"));
    size_t iters = calib.GetParams().trajectorySize;
    bool drawLandmark = true;
    bool drawObs = false;
    bool drawCamera = true;
    bool drawTraj = true;

    std::ofstream offsetFile(dataDir + std::string("/offsets.txt") + dataPostfix + std::string(".txt"), std::ios::out);
    std::ofstream reprojectionFile(dataDir + std::string("/reproj_error.txt") + dataPostfix + std::string(".txt"), std::ios::out);
    std::ofstream extrinsicFile(dataDir + std::string("/extrinsic_errors.txt") + dataPostfix + std::string(".txt"), std::ios::out);

    for(size_t i = 0; i < iters; i++)
    {
        calib.SimulationStep(i);
        calib.UpdateViewer();
        if (i > 1 && i % 2 == 0)
        {
            calib.OptimizeStep();
        }

        calib.DrawState(i, 0, calib.initialEstimate, 0.0f, 0.8f, 0.8f, 1.0f,  false, true, false, false);
        calib.DrawState(i, 1, calib.currentEstimate, 0.8f, 0.0f, 0.0f, 1.0f,  drawLandmark, drawTraj, drawObs, drawCamera);
        calib.DrawState(i, 2, calib.groundTruth, 0.0f, 0.8f, 0.0f, 1.0f,  drawLandmark, drawTraj, drawObs, drawCamera);

        std::cout << calib.currentEstimate.at<gtsam::RobotConfig>(gtsam::Symbol('q', i)).getQ().transpose() << std::endl;
        std::cout << calib.groundTruth.at<gtsam::RobotConfig>(gtsam::Symbol('q', i)).getQ().transpose() << std::endl;

        gtsam::Vector err = calib.ComputeLatestJointAngleOffsets(i);

        for(size_t j = 0; j < 6; j++)
        {
            offsetFile << err(j);

            offsetFile << " ";
        }
        gtsam::Vector err_sim = calib.ComputeLatestGroundTruthSimOffsets(i);

        for(size_t j = 0; j < 6; j++)
        {
            offsetFile << err_sim(j);

            if (j < 5)
            {
                offsetFile << " ";
            }
        }

        offsetFile << std::endl;

        std::vector<double> errors;
        calib.CalculateCurrentErrors(errors);

        if (errors.size() > 3)
        {
            std::sort(errors.begin(), errors.end());
            double median = errors.at(errors.size() / 2);
            reprojectionFile << median << std::endl;
            ROS_WARN("Median error: %f",  median);
        }


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




        hz.sleep();
        ros::spinOnce();

        gtsam::ArmSlamCalib::CalibrationError error = calib.ComputeError(calib.groundTruth, calib.currentEstimate);
        errorstream << error.landmarkError << " " << error.extrinsicError << " " << error.jointAngleError << std::endl;
    }

    errorstream.close();
    extrinsicFile.close();
    offsetFile.close();
    calib.PrintSimErrors(dataDir, dataPostfix);
}

