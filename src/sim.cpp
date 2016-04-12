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
    gtsam::ArmSlamCalib::Params params;
    params.simulated = true;
    params.optimizationMode = gtsam::ArmSlamCalib::ISAM;
    gtsam::ArmSlamCalib calib(nh, std::make_shared<std::mutex>(), params);

    std::vector<std::string> joints;
    joints.push_back("mico_joint_1");
    joints.push_back("mico_joint_2");
    joints.push_back("mico_joint_3");
    joints.push_back("mico_joint_4");
    joints.push_back("mico_joint_5");
    joints.push_back("mico_joint_6");

    std::string cameraName = "mico_end_effector";
    calib.InitRobot("package://ada_description/robots/mico.urdf", joints, cameraName);
    calib.CreateSimulation();

    ros::Rate hz(30);

    for (size_t i = 0; i < 100; i++)
    {
        hz.sleep();
        calib.UpdateViewer();
        ros::spinOnce();
    }

    std::ofstream errorstream("error.txt");
    size_t iters = calib.GetParams().trajectorySize;
    bool drawLandmark = true;
    bool drawObs = false;
    bool drawCamera = false;
    bool drawTraj = true;



    std::ofstream offsetFile("offsets.txt", std::ios::out);
    std::ofstream reprojectionFile("reproj_error.txt", std::ios::out);
    std::ofstream extrinsicFile("extrinsic_errors.txt", std::ios::out);


    for(size_t i = 0; i < iters; i++)
    {
        calib.SimulationStep(i);
        if (i > 1)
        {
            calib.OptimizeStep();
        }
        calib.DrawState(i, 0, calib.initialEstimate, 0.0f, 0.8f, 0.8f, 1.0f,  drawLandmark, drawTraj, drawObs, drawCamera);
        calib.DrawState(i, 1, calib.currentEstimate, 0.8f, 0.0f, 0.0f, 1.0f,  drawLandmark, drawTraj, drawObs, drawCamera);
        calib.DrawState(i, 2, calib.groundTruth, 0.0f, 0.8f, 0.0f, 1.0f,  drawLandmark, drawTraj, drawObs, drawCamera);
        calib.UpdateViewer();

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
    calib.PrintSimErrors(".", "");

    while(ros::ok())
    {
        calib.DrawState(params.trajectorySize, 0, calib.initialEstimate, 0, 0.8, 0.8, 1.0, drawLandmark, drawTraj, drawObs, drawCamera);
        calib.DrawState(params.trajectorySize, 1, calib.currentEstimate, 0.8, 0.0, 0.0, 1.0, drawLandmark, drawTraj, drawObs, drawCamera);
        calib.DrawState(params.trajectorySize, 2, calib.groundTruth, 0.0, 0.8, 0.0, 1.0, drawLandmark, drawTraj, drawObs, drawCamera);
        calib.UpdateViewer();
        hz.sleep();
        ros::spinOnce();
    }

    /*
    const std::shared_ptr<r3::util::CatkinResourceRetriever> resourceRetriever = std::make_shared<r3::util::CatkinResourceRetriever>();

    dart::common::ResourcePtr resource = resourceRetriever->retrieve("package://ada_description/robots/mico.urdf");

    if (!resource)
    {
        std::cerr << "Could not get resource\n";
    }
    else
    {
        if(resourceRetriever->exists("package://ada_description/robots/mico.urdf"))
        {
            std::cout << "URI exists\n";
        }
    }
    dart::utils::DartLoader urdfLoader;
    const dart::dynamics::SkeletonPtr skeleton= urdfLoader.parseSkeleton("package://ada_description/robots/mico.urdf", resourceRetriever);

    if (!skeleton)
    {
       std::cerr << "Failed loading robot\n";
       return 1;
    }

    dart::simulation::WorldPtr world = std::make_shared<dart::simulation::World>();
    world->addSkeleton(skeleton);

    dart::dynamics::SkeletonPtr box =  CreateBox("box", Eigen::Vector3d(1, 1, 0.01), Eigen::Vector3d(0, 0, -0.1), 1.0);

    dart::rviz::InteractiveMarkerViewer viewer("dart_markers");
    viewer.addSkeleton(skeleton);
    viewer.addSkeleton(box);

    std::vector<std::string> joints;
    joints.push_back("mico_joint_1");
    joints.push_back("mico_joint_2");
    joints.push_back("mico_joint_3");
    joints.push_back("mico_joint_4");
    joints.push_back("mico_joint_5");
    joints.push_back("mico_joint_6");

    std::vector<dart::dynamics::DegreeOfFreedom*> dofs;

    for (size_t i = 0; i < joints.size(); i++)
    {
        dofs.push_back(skeleton->getDof(joints.at(i)));
    }

    std::string cameraLink = "mico_end_effector";
    dart::dynamics::BodyNode* cameraBody = skeleton->getBodyNode(cameraLink);

    dart::dynamics::GroupPtr arm = dart::dynamics::Group::create("arm", dofs, false, true);

    // 1. Create a factor graph container and add factors to it
    NonlinearFactorGraph graph;
    Values initialEstimate;

    size_t landmarksX = 10;
    size_t landmarksY = 10;
    double landmarkSizeX = 5;
    double landmarkSizeY = 5;
    std::vector<gtsam::Point3> landmarks;
    for (size_t x = 0; x < landmarksX; x++)
    {
        for (size_t y = 0; y < landmarksY; y++)
        {
            double lx = x * (landmarkSizeX / landmarksX) - landmarkSizeX * 0.5f;
            double ly = y * (landmarkSizeY / landmarksY) - landmarkSizeY * 0.5f;
            double lz = (sin(lx * 2) + cos(ly * 2) + 2) * (pow(lx, 2) + pow(ly, 2)) * 0.1;
            landmarks.push_back(gtsam::Point3(lx * 2, ly * 2, lz));
        }
    }

    size_t trajectorySize = 50;
    Eigen::VectorXd q = arm->getPositions();

    Cal3_S2::shared_ptr calib = boost::make_shared<Cal3_S2>(500, 500, 0.0, 640/2, 480/2);

    Vector encoderNoiseSigma(joints.size());
    double noiseSigma = 0.1;
    for (size_t i = 0; i < joints.size(); i++)
    {
        encoderNoiseSigma(i) = noiseSigma;
    }
    noiseModel::Diagonal::shared_ptr encoderNoise = noiseModel::Diagonal::Sigmas(encoderNoiseSigma);

    std::vector<Eigen::VectorXd> trajectory(trajectorySize, Eigen::VectorXd::Zero(joints.size()));
    std::vector<Eigen::VectorXd> encoders(trajectorySize, Eigen::VectorXd::Zero(joints.size()));
    for (size_t i = 0; i < trajectorySize; i++)
    {
        Eigen::VectorXd encoder = q;
        for (size_t k = 0; k < joints.size(); k++)
        {
            q(k) += -cos(i * 0.05 + k * 50) * 0.02;
            encoder(k) = q(k)  + Rand() * 0.1 - 0.05;
        }

        trajectory[i] = q;
        encoders[i] = encoder;
        initialEstimate.insert(Symbol('q', i), RobotConfig(encoder));
        graph.add(boost::make_shared<EncoderFactor>(Symbol('q', i), encoder, encoderNoise));
    }

    ros::Rate hz(60);

    noiseModel::Diagonal::shared_ptr calibrationPrior = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.01)));

    graph.add(PriorFactor<Pose3>(Symbol('K', 0), Pose3::identity(), calibrationPrior));
    initialEstimate.insert(Symbol('K', 0), Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.08, 0.0, 0.0)));

    noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas(Vector2::Constant(1.0));


    std::map<size_t, size_t> landmarkObserved;

    for (int t = 0; t < trajectorySize; t++)
    {
        arm->setPositions(trajectory[t]);
        PinholeCamera<Cal3_S2> camera(Pose3(cameraBody->getWorldTransform().matrix()), *calib);

        for (size_t l = 0; l < landmarks.size(); l++)
        {
            const Point3& landmark = landmarks.at(l);
            try
            {
                std::pair<Point2, bool> uv = camera.projectSafe(landmark);

                if (uv.second && uv.first.x() > 0 && uv.first.y() > 0 && uv.first.x() < 640 && uv.first.y() < 480)
                {
                    if (landmarkObserved.find(l) == landmarkObserved.end())
                    {
                        initialEstimate.insert(Symbol('l', l), landmarks.at(l).compose(gtsam::Point3(Rand() * 0.2 - 0.1, Rand() * 0.02 - .1, Rand() * .2 - .1)));
                        landmarkObserved[l] = 0;
                    }
                    landmarkObserved[l]++;
                    graph.add(boost::make_shared<RobotProjectionFactor<Cal3_S2> >(uv.first, measurementNoise,
                            Symbol('q', t), Symbol('l', l), Symbol('K', 0),
                            arm, cameraBody,  &robotMutex, calib, false, true));
                }
            }
            catch(CheiralityException& e)
            {
                continue;
            }
        }

        viewer.update();
        hz.sleep();
    }

    std::ofstream outStream("graph.dot");
    graph.saveGraph(outStream);
    outStream.close();

    LevenbergMarquardtParams params;
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
    params.maxIterations = 999999;

    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);

    Values result = optimizer.optimize();
    result.print("Final Result:\n");

    ros::Publisher vizPublisher = nh.advertise<visualization_msgs::MarkerArray>("viz", 10);

    while (ros::ok())
    {
        visualization_msgs::MarkerArray array;
        visualization_msgs::Marker landmarkViz;
        landmarkViz.pose.orientation.w = 1;
        landmarkViz.scale.x = 0.05;
        landmarkViz.scale.y = 0.05;
        landmarkViz.scale.z = 0.05;
        landmarkViz.header.frame_id = "/map";
        landmarkViz.header.stamp = ros::Time::now();
        landmarkViz.id = 1;
        landmarkViz.type = visualization_msgs::Marker::POINTS;
        for (size_t l = 0; l < landmarks.size(); l++)
        {
            if (landmarkObserved.find(l) != landmarkObserved.end())
            {
                Point3 trueLandmark = landmarks.at(l);
                Point3 estLandmark = result.at<gtsam::Point3>(Symbol('l', l));

                geometry_msgs::Point p1;
                p1.x = trueLandmark.x();
                p1.y = trueLandmark.y();
                p1.z = trueLandmark.z();


                geometry_msgs::Point p2;
                p2.x = estLandmark.x();
                p2.y = estLandmark.y();
                p2.z = estLandmark.z();

                std_msgs::ColorRGBA col1;
                col1.r = 0.0;
                col1.g = 0.8;
                col1.b = 0.0;
                col1.a = 1.0;

                std_msgs::ColorRGBA col2;
                col2.r = 0.8;
                col2.g = 0.0;
                col2.b = 0.0;
                col2.a = 1.0;

                landmarkViz.points.push_back(p1);
                landmarkViz.points.push_back(p2);
                landmarkViz.colors.push_back(col1);
                landmarkViz.colors.push_back(col2);
            }
        }

        array.markers.push_back(landmarkViz);


        visualization_msgs::Marker trajectoryViz;
        trajectoryViz.pose.orientation.w = 1;
        trajectoryViz.scale.x = 0.01;
        trajectoryViz.scale.y = 0.01;
        trajectoryViz.scale.z = 0.01;
        trajectoryViz.header.frame_id = "/map";
        trajectoryViz.header.stamp = ros::Time::now();
        trajectoryViz.id = 2;
        trajectoryViz.type = visualization_msgs::Marker::LINE_LIST;

        std_msgs::ColorRGBA traj1Color;
        traj1Color.a = 1.0;
        traj1Color.g = 0.8;

        std_msgs::ColorRGBA traj2Color;
        traj2Color.r = 0.8;
        traj2Color.a = 1.0;

        for (size_t t = 1; t < trajectorySize; t++)
        {
            arm->setPositions(trajectory[t]);
            Eigen::Isometry3d truePos = cameraBody->getWorldTransform();

            arm->setPositions(trajectory[t - 1]);
            Eigen::Isometry3d truePos0 = cameraBody->getWorldTransform();

            geometry_msgs::Point truePoint0;
            truePoint0.x = truePos0.translation().x();
            truePoint0.y = truePos0.translation().y();
            truePoint0.z = truePos0.translation().z();

            geometry_msgs::Point truePoint1;
            truePoint1.x = truePos.translation().x();
            truePoint1.y = truePos.translation().y();
            truePoint1.z = truePos.translation().z();

            trajectoryViz.colors.push_back(traj1Color);
            trajectoryViz.colors.push_back(traj1Color);

            trajectoryViz.points.push_back(truePoint0);
            trajectoryViz.points.push_back(truePoint1);
        }

        gtsam::Pose3 ext = result.at<gtsam::Pose3>(Symbol('K', 0));

        for (size_t t = 1; t < trajectorySize; t++)
        {
            RobotConfig estQ = result.at<RobotConfig>(Symbol('q', t));
            arm->setPositions(estQ.getQ());
            Eigen::Isometry3d estPos = cameraBody->getWorldTransform() * Eigen::Isometry3d(ext.matrix()) ;

            RobotConfig estQ0 = result.at<RobotConfig>(Symbol('q', t - 1));
            arm->setPositions(estQ0.getQ());
            Eigen::Isometry3d estPos0 =   cameraBody->getWorldTransform() * Eigen::Isometry3d(ext.matrix()) ;

            geometry_msgs::Point truePoint0;
            truePoint0.x = estPos0.translation().x();
            truePoint0.y = estPos0.translation().y();
            truePoint0.z = estPos0.translation().z();

            geometry_msgs::Point truePoint1;
            truePoint1.x = estPos.translation().x();
            truePoint1.y = estPos.translation().y();
            truePoint1.z = estPos.translation().z();

            trajectoryViz.colors.push_back(traj2Color);
            trajectoryViz.colors.push_back(traj2Color);

            trajectoryViz.points.push_back(truePoint0);
            trajectoryViz.points.push_back(truePoint1);
        }

        array.markers.push_back(trajectoryViz);

        vizPublisher.publish(array);

        hz.sleep();
    }
    return 0;
    */
}

