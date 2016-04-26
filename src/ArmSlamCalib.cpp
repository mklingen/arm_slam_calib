/*
 * ArmSlamCalib.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: mklingen
 */

#include <arm_slam_calib/Utils.h>
#include <arm_slam_calib/ArmSlamCalib.h>
#include <arm_slam_calib/RelativePoseAdapter.h>
#include <arm_slam_calib/RobotConfig.h>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac/Ransac.hpp>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <opengv/triangulation/methods.hpp>
#include <pcl/common/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <aikido/rviz/SkeletonMarker.hpp>
#include <arm_slam_calib/Timer.h>

using namespace utils;
using namespace gtsam;
using namespace timer;


void RansacUnitTest()
{
    gtsam::Cal3_S2 cal(640, 480, 0, 640/2, 480/2);
    gtsam::PinholeCamera<gtsam::Cal3_S2> camA = gtsam::PinholeCamera<gtsam::Cal3_S2>::Lookat(gtsam::Point3(0, 0, -1), gtsam::Point3(0, 0, 0), gtsam::Point3(0, 1, 0), cal);
    gtsam::PinholeCamera<gtsam::Cal3_S2> camB = gtsam::PinholeCamera<gtsam::Cal3_S2>::Lookat(gtsam::Point3(1, 0, -1), gtsam::Point3(0, 0, 0), gtsam::Point3(0, 1, 0), cal);;
    std::vector<Landmark> groundTruthLandmarks;
    std::vector<gtsam::Point2> obsA;
    std::vector<gtsam::Point2> obsB;

    for (float dx = -0.1; dx < 0.1; dx += 0.05)
    {
        for (float dy = -0.1; dy < 0.1; dy += 0.05)
        {
            for (float dz = -0.1; dz < 0.1; dz += 0.05)
            {
                Landmark l;
                l.position = gtsam::Point3(dx, dy, dz);
                groundTruthLandmarks.push_back(l);
            }
        }
    }


    for (size_t i = 0; i < groundTruthLandmarks.size(); i++)
    {
        const Landmark& l = groundTruthLandmarks.at(i);
        obsA.push_back(camA.project2(l.position));
        obsB.push_back(camB.project2(l.position));
    }

    if (groundTruthLandmarks.size() >= 8)
    {
       opengv::relative_pose::RelativePoseAdapter adapter(camA.pose(), camB.pose(), cal, groundTruthLandmarks, obsA, obsB);
       typedef opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem CentralRelativePoseSacProblem;
       opengv::sac::Ransac<CentralRelativePoseSacProblem> ransac;
       boost::shared_ptr<CentralRelativePoseSacProblem> problem(new CentralRelativePoseSacProblem(adapter, CentralRelativePoseSacProblem::STEWENIUS));
       ransac.sac_model_ = problem;
       ransac.threshold_ = 9;     //(1.0 - cos(0.5/600));
       ransac.max_iterations_ = 50;


       if (ransac.computeModel(3))
       {
           // assess success
           size_t rel_pose_inliers = ransac.inliers_.size();
           float rel_pose_ratio = float(rel_pose_inliers) / float(groundTruthLandmarks.size());

           if (rel_pose_inliers > 5 && rel_pose_ratio > 0.5f)
           {
               ROS_WARN("GOT %lu ransac inliers!", rel_pose_inliers);
               Eigen::Matrix4d relTransform = Eigen::Matrix4d::Identity();
               relTransform.topLeftCorner<3, 4>() = ransac.model_coefficients_;
               double motionEstimation = (camB.pose().translation() - camA.pose().translation()).norm();
               ROS_WARN("Motion estimation %f", motionEstimation);
               Eigen::Vector3d trans = Eigen::Vector3d(relTransform.topRightCorner<3, 1>()).normalized() * motionEstimation;
               opengv::rotation_t rotation2 = camA.pose().rotation().matrix() * relTransform.topLeftCorner<3, 3>();
               opengv::translation_t position2 = camA.pose().rotation().matrix() * (trans) + camA.pose().translation().vector();

               gtsam::Pose3 newPoseB = gtsam::Pose3(gtsam::Rot3(rotation2), gtsam::Point3(position2));
               std::cout << "Ground truth transofm: " << std::endl;
               std::cout << camB.pose() << std::endl;
               std::cout << "Computed world transform: " << std::endl;
               std::cout << newPoseB << std::endl;
               std::vector<Landmark> inlierLandmarks;
               std::vector<gtsam::Point2> inlinerPointsA;
               std::vector<gtsam::Point2> inlierPointsB;
               for (size_t k = 0; k < ransac.inliers_.size(); k++)
               {
                   size_t inlier = (size_t)ransac.inliers_.at(k);
                   inlierLandmarks.push_back(groundTruthLandmarks.at(inlier));
                   inlinerPointsA.push_back(obsA.at(inlier));
                   inlierPointsB.push_back(obsB.at(inlier));
               }
               opengv::relative_pose::RelativePoseAdapter tringulationAdapter(camA.pose(), newPoseB, cal, inlierLandmarks, inlinerPointsA, inlierPointsB);

               for (size_t j = 0; j < inlierLandmarks.size(); j++)
               {
                   const Landmark& landmark = inlierLandmarks.at(j);
                   opengv::point_t pt = opengv::triangulation::triangulate(tringulationAdapter, j);
                   gtsam::Point3 wpt = camA.pose().transform_from(gtsam::Point3(pt));
                   ROS_INFO("New landmark position is %f %f %f vs %f %f %f", wpt.x(), wpt.y(), wpt.z(), landmark.position.x(), landmark.position.y(), landmark.position.z());
               }
               //exit(-1);
           }
           else
           {
               ROS_ERROR("There were only %lu/%lu inliers.", rel_pose_inliers, groundTruthLandmarks.size());
           }
       }
       else
       {
           ROS_ERROR("Ransac failed.");
       }

    }
    //exit(-1);
}

void TriangulateUnitTest()
{
    ArmSlamCalib::Params params;

    gtsam::Point3 testPoint(-3, 2.4, 0.413828);

    /*
    gtsam::Pose3 testPoseA(
            Rot3(-0.391663, -0.62161, -0.678381,
                  0.310805, -0.783327, 0.53833,
                 -0.866025, -1.26799e-07, 0.5),
   gtsam::Point3(-0.0277404, 0.392229, 0.169273));


    gtsam::Pose3 testPoseB(
            Rot3(-0.282321, -0.825336, -0.488995,
                  0.412668, -0.564642, 0.714762,
                  -0.866025, -1.26799e-07, 0.5),
                  gtsam::Point3(0.0894103, 0.382909, 0.169273));
    */

    gtsam::Pose3 testPoseA(gtsam::Rot3::identity(), gtsam::Point3(-3, 2.4, 0));


    gtsam::Pose3 testPoseB(gtsam::Rot3::identity(), gtsam::Point3(-2, 1.4, 0.1));

    Cal3_S2 cal(params.fx, params.fy, 0, params.cx, params.cy);
    gtsam::PinholeCamera<Cal3_S2> camA(testPoseA, cal);
    gtsam::PinholeCamera<Cal3_S2> camB(testPoseB, cal);
    gtsam::Point2 uv1 = camA.project(testPoint);
    gtsam::Point2 uv2 = camB.project(testPoint);
    bool valid = false;
    gtsam::Point3 triangulated = utils::Triangulate(uv1, uv2, testPoseA, testPoseB, valid, cal);
    gtsam::Point3 triangulated_World = testPoseA.transform_from(triangulated);
    gtsam::Point3 gt_cam = testPoseA.transform_to(testPoint);
    std::cout << triangulated << std::endl;
    std::cout << gt_cam << std::endl;
    std::cout << triangulated_World << std::endl;
    std::cout << testPoint << std::endl;
    exit(-1);
}



namespace gtsam
{
    dart::dynamics::SkeletonPtr CreateBox(const std::string& name, const Eigen::Vector3d& dimensions, const Eigen::Vector3d& pos, const double& mass)
    {
        dart::dynamics::SkeletonPtr skeleton = dart::dynamics::Skeleton::create(name);
        dart::dynamics::ShapePtr shape = std::make_shared<dart::dynamics::BoxShape>(dimensions);

        dart::dynamics::BodyNode::Properties bodyProps;
        bodyProps.mName = name;
        bodyProps.mInertia.setMass(mass);

        dart::dynamics::FreeJoint::Properties jointProp;
        jointProp.mName = name;
        jointProp.mT_ParentBodyToJoint.translation() = pos;
        auto p = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr, jointProp, bodyProps);
        p.second->createShapeNodeWith<dart::dynamics::VisualAddon>(shape);

        return skeleton;
    }


    ArmSlamCalib::ArmSlamCalib(const ros::NodeHandle& nh_,  const std::shared_ptr<std::mutex>& robotMutex_, const ArmSlamCalib::Params& noiseParams_) :
            nh(nh_), robotMutex(robotMutex_), cameraBody(0x0), params(noiseParams_), maxID(0), perlin(nh.param("seed", 0)), simExtrinsic(gtsam::Pose3::identity())
    {
        //TriangulateUnitTest();
        //RansacUnitTest();
        double cauchy = 3.0;
        nh.param("cauchy_multiplier", cauchy, cauchy);
        cauchyEstimator = boost::make_shared<gtsam::noiseModel::mEstimator::Cauchy>(cauchy);
        graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
        newGraph = boost::make_shared<gtsam::NonlinearFactorGraph>();
        vizPublisher = nh.advertise<visualization_msgs::MarkerArray>("viz", 10);
        displayImgPublisher = nh.advertise<sensor_msgs::Image>("display", 10);
        pointCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("pointclouds", 1);
        latestCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("latest_point_cloud", 1);
        numObservationsThisIter = 0;
        encoders = Eigen::aligned_vector<gtsam::Vector>();
        trajectory = Eigen::aligned_vector<gtsam::Vector>();

        if (params.optimizationMode == ISAM)
        {
            ISAM2Params isamParams;
            ISAM2GaussNewtonParams gaussParams;
            isamParams.setOptimizationParams(gaussParams);
            isam = boost::make_shared<ISAM2>(isamParams);
        }
    }

    ArmSlamCalib::~ArmSlamCalib()
    {

    }

    void ArmSlamCalib::InitRobot(dart::dynamics::SkeletonPtr skel_, const std::vector<std::string>& dofs_, const std::string& cameraLink_)
    {
        dofs = dofs_;

        for (size_t i = 0; i < dofs_.size(); i++)
        {
            std::cout << dofs_[i] << " ";
        }
        std::cout << std::endl;

        skeleton = skel_;
        world = std::make_shared<dart::simulation::World>();
        world->addSkeleton(skeleton);
        viewer = std::make_shared<aikido::rviz::InteractiveMarkerViewer>("dart_markers");

        viewer->addSkeleton(skeleton);

        std::vector<dart::dynamics::DegreeOfFreedom*> dofVec;

        for (size_t i = 0; i < dofs.size(); i++)
        {
           dofVec.push_back(skeleton->getDof(dofs.at(i)));
        }

        cameraBody = skeleton->getBodyNode(cameraLink_);

        arm = dart::dynamics::Group::create("arm", dofVec, false, true);

        //dart::dynamics::SkeletonPtr box =  CreateBox("box", Eigen::Vector3d(1, 1, 0.01), Eigen::Vector3d(0, 0, -0.1), 1.0);
        //world->addSkeleton(box);
        //viewer->addSkeleton(box);

        if (params.drawEstimateRobot)
        {
           estimateSkeleton = skeleton->clone();
           estimateSkeleton->setName("estimate_ada");
           std::vector<dart::dynamics::DegreeOfFreedom*> dofVec2;

           for (size_t i = 0; i < dofs.size(); i++)
           {
               dofVec2.push_back(estimateSkeleton->getDof(dofs.at(i)));
           }

           estimateArm = dart::dynamics::Group::create("estimateArm", dofVec2, false, true);

           aikido::rviz::SkeletonMarkerPtr marker = viewer->addSkeleton(estimateSkeleton);
           marker->SetColor(Eigen::Vector4d(0.5, 1.0, 0.5, 0.5));
        }
    }

    void ArmSlamCalib::InitRobot(const std::string& urdf, const std::vector<std::string>& dofs_, const std::string& cameraLink_)
    {
        dofs = dofs_;
        dart::utils::DartLoader urdfLoader;

        const std::shared_ptr<aikido::util::CatkinResourceRetriever> resourceRetriever = std::make_shared<aikido::util::CatkinResourceRetriever>();

        dart::common::ResourcePtr resource = resourceRetriever->retrieve(urdf);

        if (!resource)
        {
            std::cerr << "Could not get resource\n";
        }
        else
        {
            if(resourceRetriever->exists(urdf))
            {
                std::cout << "URI exists\n";
            }
        }

        skeleton= urdfLoader.parseSkeleton(urdf, resourceRetriever);
        InitRobot(skeleton, dofs_, cameraLink_);
    }

    void ArmSlamCalib::SimulationStep(size_t iter)
    {
        Vector q = trajectory[iter];
        Vector enc = encoders[iter];
        AddValue(Symbol('q', iter), RobotConfig(enc, arm));
        groundTruth.insert(Symbol('q', iter), RobotConfig(q, arm));
        AddFactor(boost::make_shared<EncoderFactor>(Symbol('q', iter), enc, encoderNoise, params.useDeadBand, params.deadBandSize));
        //SimulateObservations(iter);
        SimulateObservationsTriangulate(iter);
    }

    void ArmSlamCalib::Params::InitializeNodehandleParams(const ros::NodeHandle& nh)
    {
        nh.param("encoder_noise_level", encoderNoiseLevel, encoderNoiseLevel);
        nh.param("extrinsic_noise_level", extrinsicNoiseLevel, extrinsicRotNoiseLevel);
        nh.param("extrinsic_rot_noise_level", extrinsicRotNoiseLevel, extrinsicRotNoiseLevel);
        nh.param("landmark_noise_level", landmarkNoiseLevel, landmarkNoiseLevel);
        nh.param("projection_noise_level", projectionNoiseLevel, projectionNoiseLevel);
        nh.param("drift_noise_level", driftNoise, driftNoise);
        nh.param("use_drift_noise", addDriftNoise, addDriftNoise);
        nh.param("do_slam", doOptimize, doOptimize);
        nh.param("fx", fx, fx);
        nh.param("fy", fy, fy);
        nh.param("cx", cx, cx);
        nh.param("cy", cy, cy);
        int tsize = (int)trajectorySize;
        nh.param("trajectory_length", tsize, tsize);
        trajectorySize = tsize;

        std::vector<double> extTranslation;
        std::vector<double> extRotation;
        extTranslation.push_back(extrinsicInitialGuess.translation().x());
        extTranslation.push_back(extrinsicInitialGuess.translation().y());
        extTranslation.push_back(extrinsicInitialGuess.translation().z());
        std::vector<double> piRot;
        piRot.push_back(extrinsicInitialGuess.rotation().roll());
        piRot.push_back(extrinsicInitialGuess.rotation().pitch());
        piRot.push_back(extrinsicInitialGuess.rotation().yaw());
        nh.param("extrinsic_initial_guess_translation", extTranslation, piRot);
        nh.param("extrinsic_initial_guess_rotation", extRotation, piRot);

        nh.param("run_ransac", runRansac, runRansac);
        nh.param("save_images", saveImages, saveImages);
        nh.param("generate_stitched_point_clouds", generateStitchedPointClouds, generateStitchedPointClouds);
        nh.param("generate_current_point_cloud", generateCurrentPointCloud, generateCurrentPointCloud);
        nh.param("compute_extrinsic_marginals", computeExtrinsicMarginals, computeExtrinsicMarginals);
        nh.param("draw_estimate_robot", drawEstimateRobot, drawEstimateRobot);
        nh.param("use_encoder_deadband", useDeadBand, useDeadBand);
        nh.param("encoder_deadband", deadBandSize, deadBandSize);
        nh.param("add_sim_perlin_noise", addSimPerlinNoise, addSimPerlinNoise);
        nh.param("sim_perlin_noise_magnitude", simPerlinMagnitude, simPerlinMagnitude);
        nh.param("sim_perlin_noise_frequency", simPerlinFrequency, simPerlinFrequency);

        std::string modeString = "ISAM";
        nh.param("optimization_mode",  modeString, modeString);

        if (modeString == "ISAM")
        {
            optimizationMode = gtsam::ArmSlamCalib::ISAM;
        }
        else if (modeString == "BatchLM")
        {
            optimizationMode = gtsam::ArmSlamCalib::BatchLM;
        }
        else if (modeString == "BatchDogLeg")
        {
            optimizationMode = gtsam::ArmSlamCalib::BatchDogleg;
        }

        gtsam::Rot3 rot = gtsam::Rot3::rodriguez(extRotation[0], extRotation[1], extRotation[2]);
        gtsam::Point3 trans(extTranslation[0], extTranslation[1], extTranslation[2]);
        extrinsicInitialGuess = gtsam::Pose3(rot, trans);
    }

    void ArmSlamCalib::OptimizeStep()
    {
        if (!params.doOptimize) return;

        printf("Observed %lu landmarks\n", numObservationsThisIter);
        if (numObservationsThisIter > 1)
        {
            Timer::Tick("Optimize");

            switch(params.optimizationMode)
            {
                case BatchDogleg:
                {
                    DoglegParams doglegParams;
                    DoglegOptimizer optimizer(*graph, currentEstimate, doglegParams);

                    try
                    {
                        currentEstimate = optimizer.optimize();
                        numObservationsThisIter = 0;
                        ROS_INFO("Total error: %f", optimizer.error());
                    }
                    catch(gtsam::IndeterminantLinearSystemException& e)
                    {
                        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter;
                        ROS_ERROR("Indeterminate: %s", keyFormatter(e.nearbyVariable()).c_str());
                    }
                    break;
                }
                case BatchLM:
                {
                    LevenbergMarquardtParams params;
                    params.linearSolverType = LevenbergMarquardtParams::SEQUENTIAL_CHOLESKY;
                    LevenbergMarquardtOptimizer optimizer(*graph, currentEstimate, params);
                    currentEstimate = optimizer.optimize();
                    numObservationsThisIter = 0;
                    ROS_INFO("Total error: %f", optimizer.error());
                    break;
                }
                case ISAM:
                {
                    ISAM2& isam2 = *isam;
                    const VariableIndex& index = isam2.getVariableIndex();

                    gtsam::Values newValues;

                    for (auto it = currentEstimate.begin(); it != currentEstimate.end(); it++)
                    {
                        if (index.find(it->key) == index.end())
                        {
                            newValues.insert(it->key, it->value);
                        }
                    }

                    try
                    {
                        isam2.update(*newGraph, newValues);
                        currentEstimate = isam2.calculateBestEstimate();
                        numObservationsThisIter = 0;
                        newGraph->resize(0);

                        if (params.computeExtrinsicMarginals)
                        {
                            extrinsicMarginals = isam2.marginalCovariance(Symbol('K', 0));
                        }

                        /*
                        try
                        {
                            //auto jacobian = graph->linearize(isam->getLinearizationPoint())->jacobian();
                            //std::cout << jacobian.first << std::endl;
                            //Eigen::FullPivLU<gtsam::Matrix> luDecomp(jacobian.first);
                            //Eigen::EigenSolver<gtsam::Matrix> eigSolver(jacobian.first);
                            //luDecomp.setThreshold(1e-5);
                            //std::cerr << "Rank of the jacobian is " << luDecomp.rank() << "/" << jacobian.first.cols() << std::endl;
                            //std::cout << (eigSolver.eigenvalues()) << std::endl;
                        }
                        catch (std::exception& e)
                        {

                        }
                        */

                    }
                    catch(gtsam::IndeterminantLinearSystemException& e)
                    {
                        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter;
                        ROS_ERROR("Indeterminate: %s", keyFormatter(e.nearbyVariable()).c_str());
                    }
                    break;
                }
            }

            for (auto it = landmarksObserved.begin(); it != landmarksObserved.end(); it++)
            {
                Landmark& lmk = it->second;
                if (currentEstimate.find(Symbol('l', it->first)) != currentEstimate.end())
                {
                    lmk.position = currentEstimate.at<gtsam::Point3>(Symbol('l', it->first));
                }
            }
            Timer::Tock("Optimize");
        }

    }

    void ArmSlamCalib::CreateSimTrajectory()
    {
        Eigen::VectorXd q = arm->getPositions();
        Eigen::VectorXd qDot =  Eigen::VectorXd::Ones(arm->getNumDofs()) * 0.001f;
        Eigen::VectorXd offset = Eigen::VectorXd::Ones(arm->getNumDofs()) * 0.25;

        std::default_random_engine generator(nh.param("seed", 0));
        std::normal_distribution<double> distribution(0, params.encoderNoiseLevel);

        for (size_t i = 0; i < params.trajectorySize; i++)
        {
            Eigen::VectorXd encoder = q;
            Eigen::VectorXd noise = GetPerlinNoise(q + offset * (i * 0.01), 1.0f, 0.01f);

            for (size_t k = 0; k < dofs.size(); k++)
            {
                noise(k) += distribution(generator) * 0.01;
            }

            qDot += noise;
            q += qDot;

            for (size_t k = 0; k < dofs.size(); k++)
            {
                if (q(k) > arm->getPositionUpperLimit(k) || q(k) < arm->getPositionLowerLimit(k))
                {
                    qDot(k) *= -0.5;
                }

                q(k) = fmax(fmin(q(k), arm->getPositionUpperLimit(k)), arm->getPositionLowerLimit(k));
                encoder(k) = q(k) + distribution(generator);
            }
            simTrajectory.push_back(q);
            simEncoders.push_back(encoder);
        }
        groundTruth.insert(Symbol('K', 0), simExtrinsic);
        simJointPublisher = nh.advertise<sensor_msgs::JointState>("/sim_joints", 10);
        simJointPublisher_groundtruth = nh.advertise<sensor_msgs::JointState>("/sim_joints_groundtruth", 10);
        simEEPublisher = nh.advertise<geometry_msgs::PoseStamped>("/in/pose", 10);
    }

    void ArmSlamCalib::SimulateImageStep(size_t iter)
    {
        if (iter >= simTrajectory.size()) return;

        sensor_msgs::JointState jointState;
        sensor_msgs::JointState jointState_groundtruth;
        ros::Time stamp = ros::Time::now();
        jointState.header.stamp = stamp;
        jointState_groundtruth.header.stamp = stamp;
        gtsam::Vector enc = simEncoders.at(iter);
        gtsam::Vector q = simTrajectory.at(iter);
        for (size_t j = 0; j < arm->getNumDofs(); j++)
        {
            jointState.position.push_back(enc(j));
            jointState.name.push_back(arm->getDof(j)->getName());
            jointState_groundtruth.position.push_back(q(j));
            jointState_groundtruth.name.push_back(arm->getDof(j)->getName());
        }

        simJointPublisher.publish(jointState);
        simJointPublisher_groundtruth.publish(jointState_groundtruth);
        arm->setPositions(q);
        Eigen::Isometry3d eePose(cameraBody->getWorldTransform().matrix() * simExtrinsic.matrix());
        Eigen::Quaterniond quat(eePose.linear());
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.frame_id = "/sim_pose";
        poseMsg.header.stamp = stamp;
        poseMsg.pose.position.x = eePose.translation()(0);
        poseMsg.pose.position.y = eePose.translation()(1);
        poseMsg.pose.position.z = eePose.translation()(2);
        poseMsg.pose.orientation.w = quat.w();
        poseMsg.pose.orientation.x = quat.x();
        poseMsg.pose.orientation.y = quat.y();
        poseMsg.pose.orientation.z = quat.z();
        simEEPublisher.publish(poseMsg);

        tf::StampedTransform stamped;
        stamped.child_frame_id_ = "/sim_pose";
        stamped.frame_id_ = "/map";
        stamped.stamp_ = stamp;
        stamped.setOrigin(tf::Vector3(poseMsg.pose.position.x, poseMsg.pose.position.y, poseMsg.pose.position.z));
        stamped.setRotation(tf::Quaternion(poseMsg.pose.orientation.x, poseMsg.pose.orientation.y, poseMsg.pose.orientation.z, poseMsg.pose.orientation.w));
        simTransformBroadcaster.sendTransform(stamped);
        ros::spinOnce();
    }

    void ArmSlamCalib::CreateSimulation(const std::string& trajectoryFile)
    {
        for (size_t x = 0; x < params.numLandmarksX; x++)
        {
            for (size_t y = 0; y < params.numLandmarksY; y++)
            {
                double lx = x * (params.landmarkSizeX / params.numLandmarksX) - params.landmarkSizeX * 0.5f;
                double ly = y * (params.landmarkSizeY / params.numLandmarksY) - params.landmarkSizeY * 0.5f;
                double lz = utils::Rand(-5, 5);
                simLandmarks.push_back(gtsam::Point3(lx * 2, ly * 2, lz));
            }
        }


        Eigen::VectorXd q = arm->getPositions();

        calib = boost::make_shared<Cal3_S2>(params.fx, params.fy, 0.0, params.cx, params.cy);

        Vector encoderNoiseSigma(dofs.size());
        for (size_t i = 0; i < dofs.size(); i++)
        {
            encoderNoiseSigma(i) = params.encoderNoiseLevel;
        }
        encoderNoise = noiseModel::Diagonal::Sigmas(encoderNoiseSigma);
        driftNoise = noiseModel::Diagonal::Sigmas(gtsam::Vector::Ones(dofs.size()) * params.driftNoise);

        trajectory = Eigen::aligned_vector<gtsam::Vector>(params.trajectorySize, gtsam::Vector::Zero(dofs.size()));
        encoders = Eigen::aligned_vector<gtsam::Vector>(params.trajectorySize, gtsam::Vector::Zero(dofs.size()));

        calibrationPrior = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(params.extrinsicNoiseLevel), Vector3::Constant(params.extrinsicRotNoiseLevel)));
        AddFactor(boost::make_shared<PriorFactor<Pose3> >(Symbol('K', 0), params.extrinsicInitialGuess, calibrationPrior));

        std::cout << "Initial pose: " << params.extrinsicInitialGuess << std::endl;
        std::cout << "Sim pose: " << simExtrinsic << std::endl;
        Pose3 calibInit = params.extrinsicInitialGuess;
        AddValue(Symbol('K', 0), calibInit);

        if (trajectoryFile != "")
        {
            LoadSimTrajectory(trajectoryFile);
        }
        else
        {
            CreateSimTrajectory();
        }

        trajectory = simTrajectory;
        encoders = simEncoders;

        measurementNoise = noiseModel::Robust::Create(cauchyEstimator, noiseModel::Diagonal::Sigmas(Vector2::Constant(params.projectionNoiseLevel)));
        landmarkPrior = noiseModel::Robust::Create(cauchyEstimator, noiseModel::Diagonal::Sigmas(Vector3::Constant(params.landmarkNoiseLevel)));
    }

    void ArmSlamCalib::SetTrajectory(const Eigen::aligned_vector<gtsam::Vector>& trajectory_)
    {
        trajectory = trajectory_;
    }

    void ArmSlamCalib::SetEncoders(const Eigen::aligned_vector<gtsam::Vector>& encoders_)
    {
        encoders = encoders_;
    }

    void ArmSlamCalib::SetLandmarks(const std::vector<gtsam::Point3>& landmarks_)
    {
        simLandmarks = landmarks_;
    }

    gtsam::Vector ArmSlamCalib::ComputeLatestGroundTruthSimOffsets(size_t t)
    {
        if (encoders.size() == 0)
        {
            return gtsam::Vector6::Zero();
        }
        if (t > 0)
        {
            return Diff(trajectory.at(t), encoders.at(t));
        }
        else
        {
            return Diff(trajectory.at(trajectory.size() - 1), encoders.at(encoders.size() - 1));
        }

   }

    gtsam::Vector ArmSlamCalib::ComputeLatestJointAngleOffsets(size_t t)
    {
        if (encoders.size() == 0)
        {
            return gtsam::Vector6::Zero();
        }

        if (t == 0)
        {
            t = encoders.size() - 1;
        }

        Symbol q_t = Symbol('q', t);

        if (currentEstimate.find(q_t) != currentEstimate.end() && initialEstimate.find(q_t) != initialEstimate.end())
        {
            gtsam::Vector currQ = currentEstimate.at<gtsam::RobotConfig>(q_t).getQ();
            gtsam::Vector initQ = initialEstimate.at<gtsam::RobotConfig>(q_t).getQ();

            return Diff(currQ, initQ);
        }

        else return gtsam::Vector6::Zero();

    }

    void ArmSlamCalib::ShowReprojectionError(gtsam::RobotProjectionFactor<Cal3_S2>& projectionFactor, const gtsam::Vector& q, const gtsam::Point3& landmark, const gtsam::Pose3& pose, size_t landmarkId, cv::Mat& imgIn)
    {
        cv::Point measurement;
        measurement.x = projectionFactor.getMeasurement().x();
        measurement.y = projectionFactor.getMeasurement().y();
        cv::Mat img;
        imgIn.copyTo(img);
        cv::circle(img, measurement, 10, cv::Scalar(255, 0, 0), 1);
        cv::Point projectionCv;
        gtsam::Point2 projection = projectionFactor.project(q, landmark, pose);
        projectionCv.x = projection.x();
        projectionCv.y = projection.y();
        cv::circle(img, projectionCv, 5, cv::Scalar(0, 255, 255), 1);
        cv::line(img, projectionCv, measurement, cv::Scalar(255, 255, 255), 1);
        std::stringstream ss;
        ss << landmarkId;
        cv::putText(img, ss.str(), cv::Point(measurement.x, measurement.y + 50), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
        //cv::imshow("Projection Error", img);
        std::stringstream filename;
        filename << "./img/" << landmarkId << "_" << projectionFactor.getTrajIndex() << ".png";
        cv::imwrite(filename.str(), img);
        //cv::waitKey(100);
    }

    void ArmSlamCalib::Optimize()
    {
        /*
        LevenbergMarquardtParams params;
        params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
        params.maxIterations = 999999;

        LevenbergMarquardtOptimizer optimizer(*newGraph, initialEstimate, params);
        currentEstimate = optimizer.optimize();
        */
    }

    bool ArmSlamCalib::ShouldCreateNewLandmark(const Landmark& landmark)
    {
        if (landmarksObserved.find(landmark.id) != landmarksObserved.end())
        {
            return false;
        }

        if (frontEnd->GetMode() == FrontEnd::Mode_Apriltags)
        {
            return true;
        }

        return Rand(0, 1) < 0.1;
        /*
        const float distThreshold = 0.1f;

        for (auto it = landmarksObserved.begin(); it != landmarksObserved.end(); it++)
        {
            double dist = (it->second.position - landmark.position).norm();

            if (dist < distThreshold)
            {
                return false;
            }
        }

        return true;
        */
    }

    void ArmSlamCalib::DrawState(size_t iter, int id, const gtsam::Values& state, float r, float g, float b, float a,
            bool drawLandmarks, bool drawTraj, bool drawObs, bool drawCamera, bool drawPointCloud, bool drawMarginalExt)
    {
        Timer::Tick("Display");
        const int numMarkers = 5;
        gtsam::Vector armConfig = arm->getPositions();
        visualization_msgs::MarkerArray array;

        std_msgs::ColorRGBA goodCol;
        goodCol.r = 0;
        goodCol.g = 1;
        goodCol.b = 0;
        goodCol.a = 1;


        std_msgs::ColorRGBA col1;
        col1.r = r;
        col1.g = g;
        col1.b = b;
        col1.a = a;


        gtsam::Pose3 ext = state.at<gtsam::Pose3>(Symbol('K', 0));

        if (drawLandmarks)
        {
            visualization_msgs::Marker landmarkViz;
            landmarkViz.pose.orientation.w = 1;
            landmarkViz.scale.x = 0.02;
            landmarkViz.scale.y = 0.02;
            landmarkViz.scale.z = 0.02;
            landmarkViz.header.frame_id = "/map";
            landmarkViz.header.stamp = ros::Time::now();
            landmarkViz.id = id * numMarkers;
            landmarkViz.type = visualization_msgs::Marker::POINTS;
            for(auto it = landmarksObserved.begin(); it != landmarksObserved.end(); it++)
            {
                const Landmark& landmark = it->second;
                if (state.find(Symbol('l', landmark.id)) != state.end())
                {
                   Point3 estLandmark = state.at<gtsam::Point3>(Symbol('l', landmark.id));
                   geometry_msgs::Point p1;
                   p1.x = estLandmark.x();
                   p1.y = estLandmark.y();
                   p1.z = estLandmark.z();
                   landmarkViz.points.push_back(p1);
                   /*
                   std_msgs::ColorRGBA lcol;
                   lcol.r = landmark.color.x();
                   lcol.g = landmark.color.y();
                   lcol.b = landmark.color.z();
                   lcol.a = 1.0;
                   */

                   if (!landmark.isTriangulated)
                       landmarkViz.colors.push_back(col1);
                   else
                       landmarkViz.colors.push_back(col1);
                }
            }

            array.markers.push_back(landmarkViz);
        }

        if (drawTraj)
        {
            visualization_msgs::Marker trajectoryViz;
            trajectoryViz.pose.orientation.w = 1;
            trajectoryViz.scale.x = 0.0025;
            trajectoryViz.scale.y = 0.0025;
            trajectoryViz.scale.z = 0.0025;
            trajectoryViz.header.frame_id = "/map";
            trajectoryViz.header.stamp = ros::Time::now();
            trajectoryViz.id = id * numMarkers + 1;
            trajectoryViz.type = visualization_msgs::Marker::LINE_STRIP;

            for (size_t t = 0; t < iter; t++)
            {
                if (state.find(Symbol('q', t)) == state.end())
                {
                    continue;
                }
               RobotConfig estQ = state.at<RobotConfig>(Symbol('q', t));
               arm->setPositions(estQ.getQ());
               Eigen::Isometry3d estPos = cameraBody->getWorldTransform() * Eigen::Isometry3d(ext.matrix()) ;

               geometry_msgs::Point truePoint0;
               truePoint0.x = estPos.translation().x();
               truePoint0.y = estPos.translation().y();
               truePoint0.z = estPos.translation().z();

               trajectoryViz.colors.push_back(col1);
               trajectoryViz.points.push_back(truePoint0);
            }

            array.markers.push_back(trajectoryViz);
        }

        if (drawCamera)
        {
            arm->setPositions(armConfig);
            Eigen::Isometry3d estPos = cameraBody->getWorldTransform() * Eigen::Isometry3d(ext.matrix()) ;
            Eigen::Matrix3d R = estPos.linear();

            visualization_msgs::Marker cameraViz;
            cameraViz.type = visualization_msgs::Marker::LINE_LIST;
            cameraViz.scale.x = 0.01;
            cameraViz.scale.y = 0.01;
            cameraViz.scale.z = 0.01;
            cameraViz.header.frame_id = "/map";
            cameraViz.header.stamp = ros::Time::now();
            cameraViz.id = id * numMarkers + 2;
            cameraViz.pose.position.x = estPos.translation().x();
            cameraViz.pose.position.y = estPos.translation().y();
            cameraViz.pose.position.z = estPos.translation().z();
            cameraViz.color.a = 1.0;

            geometry_msgs:: Point origin;
            origin.x = 0;
            origin.y = 0;
            origin.z = 0;
            geometry_msgs::Point xPt;
            xPt.x = R(0, 0) * 0.15;
            xPt.y = R(1, 0) * 0.15;
            xPt.z = R(2, 0) * 0.15;
            geometry_msgs::Point yPt;
            yPt.x = R(0, 1) * 0.15;
            yPt.y = R(1, 1) * 0.15;
            yPt.z = R(2, 1) * 0.15;
            geometry_msgs::Point zPt;
            zPt.x = R(0, 2) * 0.15;
            zPt.y = R(1, 2) * 0.15;
            zPt.z = R(2, 2) * 0.15;
            std_msgs::ColorRGBA red;
            red.r = 1.0;
            red.g = 0.0;
            red.b = 0.0;
            red.a = 1.0;
            std_msgs::ColorRGBA green;
            green.r = 0.0;
            green.g = 1.0;
            green.b = 0.0;
            green.a = 1.0;
            std_msgs::ColorRGBA blue;
            blue.r = 0.0;
            blue.g = 0.0;
            blue.b = 1.0;
            blue.a = 1.0;
            cameraViz.colors.push_back(red);
            cameraViz.colors.push_back(red);
            cameraViz.colors.push_back(green);
            cameraViz.colors.push_back(green);
            cameraViz.colors.push_back(blue);
            cameraViz.colors.push_back(blue);
            cameraViz.points.push_back(origin);
            cameraViz.points.push_back(xPt);
            cameraViz.points.push_back(origin);
            cameraViz.points.push_back(yPt);
            cameraViz.points.push_back(origin);
            cameraViz.points.push_back(zPt);
            cameraViz.pose.orientation.w = 1;

            array.markers.push_back(cameraViz);
        }

        if (drawMarginalExt && extrinsicMarginals.rows() > 0)
        {
            arm->setPositions(armConfig);
            Eigen::Isometry3d estPos = cameraBody->getWorldTransform() * Eigen::Isometry3d(ext.matrix()) ;
            Eigen::Matrix3d R = estPos.linear();

            visualization_msgs::Marker marginalViz;
            marginalViz.type = visualization_msgs::Marker::SPHERE;
            marginalViz.scale.x = extrinsicMarginals(0, 0) * 5;
            marginalViz.scale.y = extrinsicMarginals(1, 1) * 5;
            marginalViz.scale.z = extrinsicMarginals(2, 2) * 5;
            marginalViz.header.frame_id = "/map";
            marginalViz.header.stamp = ros::Time::now();
            marginalViz.id = id * numMarkers + 3;
            marginalViz.pose.position.x = estPos.translation().x();
            marginalViz.pose.position.y = estPos.translation().y();
            marginalViz.pose.position.z = estPos.translation().z();
            marginalViz.color.a = 0.5;
            marginalViz.color.r = 1.0;
            marginalViz.color.g = 0.8;
            marginalViz.color.b = 0.2;
            array.markers.push_back(marginalViz);
        }

        if (drawObs)
        {
            visualization_msgs::Marker observationViz;
            observationViz.pose.orientation.w = 1;
            observationViz.scale.x = 0.005;
            observationViz.scale.y = 0.005;
            observationViz.scale.z = 0.005;
            observationViz.header.frame_id = "/map";
            observationViz.header.stamp = ros::Time::now();
            observationViz.id = id * numMarkers + 4;
            observationViz.type = visualization_msgs::Marker::LINE_LIST;
            std_msgs::ColorRGBA col2;
            col2.r = r;
            col2.g = g;
            col2.b = b;
            col2.a = a * 0.5f;

            for(auto it = landmarksObserved.begin(); it != landmarksObserved.end(); it++)
            {
                const Landmark& landmark = it->second;
                if (state.find(Symbol('l', landmark.id)) != state.end())
                {
                   Point3 estLandmark = state.at<gtsam::Point3>(Symbol('l', landmark.id));
                   geometry_msgs::Point p1;
                   p1.x = estLandmark.x();
                   p1.y = estLandmark.y();
                   p1.z = estLandmark.z();

                   for (size_t j = 0; j < landmark.configs.size(); j++)
                   {
                       size_t t = landmark.configs.at(j);
                       gtsam::Vector q = state.at<RobotConfig>(Symbol('q', t)).getQ();
                       arm->setPositions(q);
                       Eigen::Isometry3d estPos = cameraBody->getWorldTransform() * Eigen::Isometry3d(ext.matrix()) ;

                       geometry_msgs::Point p2;
                       p2.x = estPos.translation().x();
                       p2.y = estPos.translation().y();
                       p2.z = estPos.translation().z();

                       observationViz.points.push_back(p1);
                       observationViz.points.push_back(p2);
                       observationViz.colors.push_back(col2);
                       observationViz.colors.push_back(col2);
                   }
                }
            }

            array.markers.push_back(observationViz);
        }

        vizPublisher.publish(array);

        if (drawPointCloud)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr compositeCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
            pcl_conversions::toPCL(ros::Time::now(), compositeCloud->header.stamp);
            compositeCloud->header.frame_id = "/map";
            compositeCloud->is_dense = false;

            for (size_t t = 0; t < iter; t+=10)
            {
                if (state.find(Symbol('q', t)) == state.end())
                {
                    continue;
                }
               RobotConfig estQ = state.at<RobotConfig>(Symbol('q', t));
               gtsam::Pose3 estPose = GetCameraPose(estQ.getQ());
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pointClouds.at(t);
               pcl::PointCloud<pcl::PointXYZRGB> transformed;
               pcl::transformPointCloud(*cloud, transformed, estPose.matrix());
               compositeCloud->points.insert(compositeCloud->points.end(), transformed.begin(), transformed.end());
            }

            pointCloudPublisher.publish(compositeCloud);
        }
        arm->setPositions(armConfig);
        Timer::Tock("Display");
    }

    void ArmSlamCalib::PublishLastPointCloud()
    {
        if (encoders.size() == 0) return;
        size_t t = encoders.size() - 1;
        if (currentEstimate.find(Symbol('q', t)) == currentEstimate.end() || !lastPointCloud.get())
        {
            return;
        }
        RobotConfig estQ = currentEstimate.at<RobotConfig>(Symbol('q', t));
        gtsam::Pose3 estPose = GetCameraPose(estQ.getQ());
        pcl::PointCloud<pcl::PointXYZRGB> transformed;
        pcl::transformPointCloud(*lastPointCloud, transformed, estPose.matrix());
        pcl_conversions::toPCL(ros::Time::now(), transformed.header.stamp);
        transformed.header.frame_id = "/map";
        transformed.is_dense = false;
        latestCloudPublisher.publish(transformed);
    }

    void ArmSlamCalib::InitializeUnsynchronizedDepth(const std::string& depthTopic)
    {
        frontEnd->SubscribeDepth(depthTopic);
    }

    void ArmSlamCalib::InitializeFeatures(const std::string& imageTopic, const std::string& infoTopic, const std::string& depthTopic, const std::string& depthInfoTopic)
    {
        FrontEnd::FeatureMatchParams featureParams;
        featureParams.InitializeNodeHandle(nh);
        frontEnd = std::make_shared<FrontEnd>(nh, FrontEnd::Mode_Features, featureParams);
        frontEnd->CreateFeatureMatchers();

        if (infoTopic == "" || depthTopic == "" || depthInfoTopic == "")
        {
            frontEnd->SubscribeCamera(imageTopic);
        }
        else
        {
            frontEnd->SubscribeSynchronizedCameraAndDepth(imageTopic, infoTopic, depthTopic, depthInfoTopic);
        }

        ros::Rate sleepRate(30);


        ROS_INFO("Waiting for new data...");
        while(!frontEnd->HasNewData() && ros::ok())
        {
            if (params.simulated)
                SimulateImageStep(0);
            sleepRate.sleep();
            ros::spinOnce();
        }

        ROS_INFO("Waiting for camera calibration...");
        calib = boost::make_shared<Cal3_S2>(params.fx, params.fy, 0.0, params.cx, params.cy);
        // Camera calibration comes from ROS in this case, not from parameters!!
        if(!frontEnd->GetCameraCalibration(*calib))
        {
            ROS_ERROR("Unable to get camera calibration!");
        }
        calibrationPrior = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(params.extrinsicNoiseLevel), Vector3::Constant(params.extrinsicRotNoiseLevel)));
        AddFactor(boost::make_shared<PriorFactor<Pose3> >(Symbol('K', 0), params.extrinsicInitialGuess, calibrationPrior));
        AddValue(Symbol('K', 0), params.extrinsicInitialGuess);
        measurementNoise = noiseModel::Robust::Create(cauchyEstimator, noiseModel::Diagonal::Sigmas(Vector2::Constant(params.projectionNoiseLevel)));
        landmarkPrior = noiseModel::Robust::Create(cauchyEstimator, noiseModel::Diagonal::Sigmas(Vector3::Constant(params.landmarkNoiseLevel)));
        ROS_INFO("Initialized");
    }


    void ArmSlamCalib::InitializeAprilTags(const std::string& imageTopic, const std::string& aprilTagsTopic)
    {
         FrontEnd::FeatureMatchParams fParams;
         fParams.InitializeNodeHandle(nh);
         frontEnd = std::make_shared<FrontEnd>(nh, FrontEnd::Mode_Apriltags, fParams);
         frontEnd->SubscribeCamera(imageTopic);
         frontEnd->SubscribeAprilTags(aprilTagsTopic);

         ros::Rate sleepRate(30);

         ROS_INFO("Waiting for new data...");
         while(!frontEnd->HasNewData() && ros::ok())
         {
             sleepRate.sleep();
             ros::spinOnce();
         }

         calib = boost::make_shared<Cal3_S2>(params.fx, params.fy, 0.0, params.cx, params.cy);
         // Camera calibration comes from ROS in this case, not from parameters!!
         if(!frontEnd->GetCameraCalibration(*calib))
         {
             ROS_ERROR("Unable to get camera calibration!");
             exit(-1);
         }
         ROS_INFO("%f %f %f %f\n", calib->fx(), calib->fy(), calib->px(), calib->py());
         calibrationPrior = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(params.extrinsicNoiseLevel), Vector3::Constant(params.extrinsicRotNoiseLevel)));
         AddFactor(boost::make_shared<PriorFactor<Pose3> >(Symbol('K', 0), params.extrinsicInitialGuess, calibrationPrior));
         AddValue(Symbol('K', 0), params.extrinsicInitialGuess);
         groundTruth.insert(Symbol('K', 0), params.extrinsicInitialGuess);
         measurementNoise = noiseModel::Robust::Create(cauchyEstimator, noiseModel::Diagonal::Sigmas(Vector2::Constant(params.projectionNoiseLevel)));
         landmarkPrior = noiseModel::Robust::Create(cauchyEstimator, noiseModel::Diagonal::Sigmas(Vector3::Constant(params.landmarkNoiseLevel)));
         ROS_INFO("Initialized");
    }

    void ArmSlamCalib::InitializeFiducial(const std::string& imageTopic, double cellSize, int numCellsX, int numCellsY)
    {
        ROS_INFO("Initializing. %s, checker: %f %d %d", imageTopic.c_str(), cellSize, numCellsX, numCellsY);
        FrontEnd::FeatureMatchParams fParams;
        frontEnd = std::make_shared<FrontEnd>(nh, FrontEnd::Mode_Checkerboard, fParams);
        frontEnd->SetCheckerboard(cellSize, numCellsX, numCellsY);
        frontEnd->SubscribeCamera(imageTopic);

        ros::Rate sleepRate(30);

        ROS_INFO("Waiting for new data...");
        while(!frontEnd->HasNewData() && ros::ok())
        {
            sleepRate.sleep();
            ros::spinOnce();
        }

        calib = boost::make_shared<Cal3_S2>(params.fx, params.fy, 0.0, params.cx, params.cy);
        // Camera calibration comes from ROS in this case, not from parameters!!
        if(!frontEnd->GetCameraCalibration(*calib))
        {
            ROS_ERROR("Unable to get camera calibration!");
        }
        calibrationPrior = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(params.extrinsicNoiseLevel), Vector3::Constant(params.extrinsicRotNoiseLevel)));
        AddFactor(boost::make_shared<PriorFactor<Pose3> >(Symbol('K', 0), params.extrinsicInitialGuess, calibrationPrior));
        AddValue(Symbol('K', 0), params.extrinsicInitialGuess);
        measurementNoise = noiseModel::Robust::Create(cauchyEstimator, noiseModel::Diagonal::Sigmas(Vector2::Constant(params.projectionNoiseLevel)));
        landmarkPrior = noiseModel::Robust::Create(cauchyEstimator, noiseModel::Diagonal::Sigmas(Vector3::Constant(params.landmarkNoiseLevel)));
        ROS_INFO("Initialized");
    }

    gtsam::Vector ArmSlamCalib::Wrap(const gtsam::Vector& q)
    {
        gtsam::Vector out = q;

        for (size_t i = 0; i < arm->getNumDofs(); i++)
        {
            if (IsContinuous(i))
            {
                out(i) = utils::WrapCircle(q(i));
            }
        }
        return out;
    }

    gtsam::Vector ArmSlamCalib::Diff(const gtsam::Vector& q1, const gtsam::Vector& q2)
    {
        gtsam::Vector out = q1 - q2;

        for (size_t i = 0; i < arm->getNumDofs(); i++)
        {
            if (IsContinuous(i))
            {
                out(i) = utils::AngleDiff(q1(i), q2(i));
            }
        }
        return out;
    }

    bool ArmSlamCalib::IsContinuous(size_t jointIndex)
    {
        dart::dynamics::DegreeOfFreedom* dof = arm->getDof(jointIndex);
        return fabs(dof->getPositionLowerLimit() - dof->getPositionUpperLimit()) < 1e-5;
    }

    void ArmSlamCalib::InitializeJointRecorder(const std::string& topic)
    {
        std::map<std::string, bool> continuousMap;
        for (size_t i = 0; i < arm->getNumDofs(); i++)
        {
            dart::dynamics::DegreeOfFreedom* dof = arm->getDof(i);

            if (fabs(dof->getPositionLowerLimit() - dof->getPositionUpperLimit()) < 1e-5)
            {
                ROS_INFO("Joint %s is continuous.", dof->getName().c_str());
                continuousMap[dof->getName()] = true;
            }
            else
            {
                ROS_INFO("Limits of %s are %f to %f", dof->getName().c_str(), dof->getPositionLimits().first, dof->getPositionLimits().second);
                continuousMap[dof->getName()] = false;
            }
        }

        jointRecorder = std::make_shared<joint_state_recorder::JointStateRecorder>(nh, continuousMap);
        jointRecorder->Subscribe(topic);

        if (params.simulated)
        {
            simJointRecorder = std::make_shared<joint_state_recorder::JointStateRecorder>(nh, continuousMap);
            simJointRecorder->Subscribe(topic + std::string("_groundtruth"));
        }
        Vector encoderNoiseSigma(dofs.size());
        for (size_t i = 0; i < dofs.size(); i++)
        {
            encoderNoiseSigma(i) = params.encoderNoiseLevel;
        }
        encoderNoise = noiseModel::Diagonal::Sigmas(encoderNoiseSigma);
        driftNoise = noiseModel::Diagonal::Sigmas(gtsam::Vector::Ones(dofs.size()) * params.driftNoise);

    }

    gtsam::Vector ArmSlamCalib::GetLatestJointAngles()
    {
        try
        {
            sensor_msgs::JointState joints = jointRecorder->GetLatest();
            gtsam::Vector q = gtsam::Vector::Zero(arm->getNumDofs());
            for(size_t i = 0; i < arm->getNumDofs(); i++)
            {
              dart::dynamics::DegreeOfFreedom* dof = arm->getDof(i);
              const std::string& name = dof->getName();

              for (size_t j = 0; j < joints.name.size(); j++)
              {
                  if (name == joints.name.at(j))
                  {
                      q(j) = joints.position.at(j);
                      break;
                  }
              }
            }
            return q;
        }
        catch(std::invalid_argument& e)
        {
            ROS_WARN("%s", e.what());
            return arm->getPositions();
        }
    }

    gtsam::Vector ArmSlamCalib::GetRecordedJointAngles(const ros::Time& stamp)
    {
        sensor_msgs::JointState joints = jointRecorder->InterpState(stamp);

        gtsam::Vector q = gtsam::Vector::Zero(arm->getNumDofs());
        for(size_t i = 0; i < arm->getNumDofs(); i++)
        {
          dart::dynamics::DegreeOfFreedom* dof = arm->getDof(i);
          const std::string& name = dof->getName();

          for (size_t j = 0; j < joints.name.size(); j++)
          {
              if (name == joints.name.at(j))
              {
                  q(j) = joints.position.at(j);
                  break;
              }
          }
        }
        return q;
    }

    gtsam::Vector ArmSlamCalib::GetSimRecordedJointAngles(const ros::Time& stamp)
    {
        sensor_msgs::JointState joints = simJointRecorder->InterpState(stamp);

        gtsam::Vector q = gtsam::Vector::Zero(arm->getNumDofs());
        for(size_t i = 0; i < arm->getNumDofs(); i++)
        {
          dart::dynamics::DegreeOfFreedom* dof = arm->getDof(i);
          const std::string& name = dof->getName();

          for (size_t j = 0; j < joints.name.size(); j++)
          {
              if (name == joints.name.at(j))
              {
                  q(j) = joints.position.at(j);
                  break;
              }
          }
        }
        return q;
    }

    ArmSlamCalib::CalibrationError ArmSlamCalib::ComputeError(const gtsam::Values& groundTruth, const gtsam::Values& current)
    {
        CalibrationError error;
        error.cameraPoseError = 0;
        error.extrinsicError = 0;
        error.jointAngleError = 0;
        error.landmarkError = 0;

        size_t measurements = 0;
        for(auto it = landmarksObserved.begin(); it != landmarksObserved.end(); it++)
        {
           const Landmark& landmark = it->second;
           if (groundTruth.find(Symbol('l', landmark.id)) != groundTruth.end() && current.find(Symbol('l', landmark.id)) != current.end())
           {
               Point3 trueLandmark = groundTruth.at<gtsam::Point3>(Symbol('l', landmark.id));
               Point3 currentLandmark = current.at<gtsam::Point3>(Symbol('l', landmark.id));
               error.landmarkError += (trueLandmark - currentLandmark).norm();
               measurements++;
           }
        }

        if (measurements > 0)
        {
            error.landmarkError /= measurements;
        }

        size_t configs = 0;

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            if (groundTruth.find(Symbol('q', i)) != groundTruth.end() && current.find(Symbol('q', i)) != current.end())
            {
                RobotConfig truePos = groundTruth.at<gtsam::RobotConfig>(Symbol('q', i));
                RobotConfig currentPos = current.at<gtsam::RobotConfig>(Symbol('q', i));
                error.jointAngleError += (truePos.getQ() - currentPos.getQ()).norm();
                configs++;
            }
        }

        if (configs > 0)
        {
            error.jointAngleError /= configs;
        }

        if (groundTruth.find(Symbol('K', 0)) != groundTruth.end() && current.find(Symbol('K', 0)) != current.end())
        {
            Pose3 trueExt = groundTruth.at<gtsam::Pose3>(Symbol('K', 0));
            Pose3 currentExt = current.at<gtsam::Pose3>(Symbol('K', 0));
            error.extrinsicError = (trueExt.translation() - currentExt.translation()).norm();
        }

        return error;
    }

    void ArmSlamCalib::GetVisibleLandmarks(const gtsam::Vector& q, std::vector<Landmark>& lms, const gtsam::Pose3& extrinsic)
    {
        Timer::Tick("FrustumCull");
        gtsam::Pose3 cameraPose = GetCameraPose(q, extrinsic);
        gtsam::PinholeCamera<Cal3_S2> camera(cameraPose, *calib);

        for (auto it = landmarksObserved.begin(); it != landmarksObserved.end(); it++)
        {
            const Landmark& landmark = it->second;

            try
            {
                gtsam::Point3 pt;
                if (currentEstimate.find(Symbol('l', landmark.id)) == currentEstimate.end())
                {
                    pt = landmark.position;
                }
                else
                {
                    pt = currentEstimate.at<gtsam::Point3>(Symbol('l', landmark.id));
                }
                std::pair<gtsam::Point2, bool> proj = camera.projectSafe(pt);
                const gtsam::Point2& uv = proj.first;
                if(proj.second && uv.x() > 0 && uv.y() > 0 && uv.x() < params.cx * 2 && uv.y() < params.cy * 2)
                {
                    lms.push_back(landmark);
                }
            }
            catch (gtsam::CheiralityException& e)
            {

            }

        }
        Timer::Tock("FrustumCull");
    }

    bool ArmSlamCalib::RealFrontEndStep()
    {
        size_t timeIndex = encoders.size();

        try
        {
            ros::Time stamp = frontEnd->GetLastImageStamp();
            gtsam::Vector q = GetRecordedJointAngles(stamp);
            std::cout << q.transpose() << std::endl;
            gtsam::Pose3 cameraPose = GetCameraPose(q);

            ROS_INFO("Time %lu\n", timeIndex);
            std::vector<Landmark> newLandmarks;
            std::vector<Landmark> visibleLandmarks;
            GetVisibleLandmarks(q, visibleLandmarks, GetCurrentExtrinsic());
            encoders.push_back(q);

            if (!params.simulated)
            {
                trajectory.push_back(q);
            }
            else
            {
                gtsam::Vector q_gt = GetSimRecordedJointAngles(stamp);
                trajectory.push_back(q_gt);

                if (groundTruth.find(Symbol('q', timeIndex)) != groundTruth.end())
                    groundTruth.erase(Symbol('q', timeIndex));
                groundTruth.insert(Symbol('q', timeIndex), RobotConfig(q_gt, arm));
            }

            AddConfig(q, timeIndex);
            if (frontEnd->GetLandmarkDisplay().get())
            {
                displayImgPublisher.publish(frontEnd->GetLandmarkDisplay()->toImageMsg());
            }

            if (params.generateStitchedPointClouds || params.generateCurrentPointCloud)
            {
                if (params.generateStitchedPointClouds)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
                    frontEnd->GeneratePointCloud(cloud);
                    pointClouds.push_back(cloud);
                    lastPointCloud = cloud;
                }
                else if (params.generateCurrentPointCloud)
                {
                    if (!lastPointCloud.get())
                    {
                        lastPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
                    }
                    frontEnd->GeneratePointCloud(lastPointCloud);
                }
            }

            if (!frontEnd->GetNewLandmarks(cameraPose, stamp, visibleLandmarks, newLandmarks))
                return false;


            std::vector<gtsam::Point3> worldLandmarks;

            for (size_t i = 0; i < newLandmarks.size(); i++)
            {
                gtsam::Point3 worldPoint = cameraPose.transform_from(newLandmarks.at(i).position);
                worldLandmarks.push_back(worldPoint);
            }

            if (params.saveImages)
                images.push_back(frontEnd->GetLastImage());

            for (size_t w = 0; w < worldLandmarks.size(); w++)
            {
                Point3 landmark = worldLandmarks.at(w);
                newLandmarks.at(w).position = landmark;

                if (landmarksObserved.find(newLandmarks.at(w).id) == landmarksObserved.end())
                {
                    if (!ShouldCreateNewLandmark(newLandmarks.at(w)))
                    {
                        continue;
                    }

                    if (frontEnd->GetMode() == FrontEnd::Mode_Features)
                    {
                        newLandmarks.at(w).id = maxID;
                        maxID++;
                    }
                }

                size_t i = newLandmarks.at(w).id;
                bool previouslyTriangulated = false;
                if (landmarksObserved.find(i) == landmarksObserved.end())
                {
                    landmarksObserved[i] = Landmark();
                    landmarksObserved[i].id = i;
                    landmarksObserved[i].color = newLandmarks.at(w).color;
                    landmarksObserved[i].position = worldLandmarks.at(w);
                    landmarksObserved[i].descriptor = newLandmarks.at(w).descriptor;
                }
                else
                {
                    previouslyTriangulated = landmarksObserved[i].isTriangulated;
                }

                Landmark& matchedLandmark = landmarksObserved[i];
                matchedLandmark.cameraPoses.push_back(cameraPose);
                matchedLandmark.isTriangulated = newLandmarks.at(w).isTriangulated;
                matchedLandmark.configs.push_back(timeIndex);
                matchedLandmark.observations.push_back(newLandmarks.at(w).observations.at(0));

                if (!matchedLandmark.isInGraph && matchedLandmark.observations.size() > 1 && (!params.runRansac || matchedLandmark.isTriangulated))
                {
                    AddLandmark(matchedLandmark);
                }
                else if (matchedLandmark.isInGraph)
                {
                    AddFactor(boost::make_shared<RobotProjectionFactor<Cal3_S2> >(
                            newLandmarks.at(w).observations.at(0), measurementNoise,
                            Symbol('q', timeIndex), Symbol('l', i), Symbol('K', 0),
                            arm,  cameraBody,  robotMutex.get(), calib, timeIndex, i, false, true));
                }
                if (matchedLandmark.observations.size() > 1)
                    numObservationsThisIter++;
            }
            ROS_INFO("%lu observations", numObservationsThisIter);


        }
        catch(std::invalid_argument& e)
        {
            return false;
        }



        Timer::Tick("Ransac/Triangulate");
        if (params.runRansac && frontEnd->GetMode() == FrontEnd::Mode_Features && encoders.size() > 1
                && currentEstimate.find(Symbol('q', timeIndex)) != currentEstimate.end())
        {
            for (int b = 0; b < (int)(timeIndex) - 1; b++)
            {
                if (currentEstimate.find(Symbol('q', b)) != currentEstimate.end())
                    RunRansac(timeIndex - 1, b);
            }
        }
        Timer::Tock("Ransac/Triangulate");

        return true;
    }

    void ArmSlamCalib::AddLandmark(Landmark& landmark)
    {
        size_t i = landmark.id;
        landmark.isNew = false;
        landmark.isInGraph = true;
        if (currentEstimate.find(Symbol('l', i)) != currentEstimate.end())
            currentEstimate.erase(Symbol('l', i));
        if (initialEstimate.find(Symbol('l', i)) != initialEstimate.end())
            initialEstimate.erase(Symbol('l', i));
        AddValue(Symbol('l', i), landmark.position);
        if (groundTruth.find(Symbol('l', i)) != groundTruth.end())
            groundTruth.erase(Symbol('l', i));
        groundTruth.insert(Symbol('l', i), landmark.position);
        AddFactor(boost::make_shared<gtsam::PriorFactor<Point3> >(Symbol('l', i), landmark.position, landmarkPrior));
        for (size_t k = 0; k < landmark.observations.size(); k++)
        {
            AddFactor(boost::make_shared<RobotProjectionFactor<Cal3_S2> >(
                    landmark.observations.at(k), measurementNoise,
                    Symbol('q', landmark.configs.at(k)), Symbol('l', i), Symbol('K', 0),
                    arm,  cameraBody,  robotMutex.get(), calib, landmark.configs.at(k), i, false, true));
        }
    }

    void ArmSlamCalib::AddConfig(const gtsam::Vector& encQ, size_t i)
    {
        if (initialEstimate.find(Symbol('q', i)) == initialEstimate.end())
        {
            AddValue(Symbol('q', i), RobotConfig(encQ, arm));
            AddFactor(boost::make_shared<EncoderFactor>(Symbol('q', i), encQ, encoderNoise, params.useDeadBand, params.deadBandSize));

            if (params.addDriftNoise && i > 0)
            {
                AddFactor(boost::make_shared<DriftFactor>(Symbol('q', i - 1), Symbol('q', i), encoders.at(i - 1), encoders.at(i), driftNoise));
            }
        }
    }

    gtsam::Pose3 ArmSlamCalib::GetCameraPose(const gtsam::Vector& jointAngles, const gtsam::Pose3& extr)
    {
        gtsam::Vector initQ = arm->getPositions();
        arm->setPositions(jointAngles);
        Eigen::Isometry3d linkPose = cameraBody->getWorldTransform();
        Eigen::Matrix4d mat = linkPose.matrix() * extr.matrix();
        arm->setPositions(initQ);
        return gtsam::Pose3(mat);
    }

    gtsam::Pose3 ArmSlamCalib::GetCameraPose(const gtsam::Vector& jointAngles)
    {
        return GetCameraPose(jointAngles, GetCurrentExtrinsic());
    }

    gtsam::Pose3 ArmSlamCalib::GetCurrentExtrinsic()
    {
        return currentEstimate.at<gtsam::Pose3>(Symbol('K', 0));
    }


    void ArmSlamCalib::SimulateObservations(size_t t)
    {
        arm->setPositions(trajectory[t]);
        PinholeCamera<Cal3_S2> camera(Pose3(cameraBody->getWorldTransform().matrix()), *calib);
        PinholeCamera<Cal3_S2> estCamera(GetCameraPose(encoders[t]), *calib);

        for (size_t l = 0; l < simLandmarks.size(); l++)
        {
            const Point3& landmark = simLandmarks.at(l);
            try
            {
                std::pair<Point2, bool> uv = camera.projectSafe(landmark);

                if (uv.second && uv.first.x() > 0 && uv.first.y() > 0 && uv.first.x() < params.cx * 2 && uv.first.y() < params.cy * 2)
                {
                    if (landmarksObserved.find(l) == landmarksObserved.end())
                    {
                        landmarksObserved[l]  = Landmark();
                        landmarksObserved[l].id = l;
                        gtsam::Point3 ptCorrupt = estCamera.backproject(uv.first, 1.0f);
                        AddValue(Symbol('l', l), ptCorrupt);
                        AddFactor(boost::make_shared<gtsam::PriorFactor<Point3> >(Symbol('l', l), ptCorrupt, landmarkPrior));
                        groundTruth.insert(Symbol('l', l), simLandmarks.at(l));
                    }

                    Landmark& observation = landmarksObserved[l];
                    observation.configs.push_back(t);
                    observation.observations.push_back(uv.first);

                    AddFactor(boost::make_shared<RobotProjectionFactor<Cal3_S2> >(uv.first, measurementNoise,
                            Symbol('q', t), Symbol('l', l), Symbol('K', 0),
                            arm, cameraBody,  robotMutex.get(), calib, t, l, false, true));

                    if (observation.observations.size()  >= 2)
                    {
                        numObservationsThisIter++;
                    }
                }
            }
            catch(CheiralityException& e)
            {
                continue;
            }
        }
    }

    bool ArmSlamCalib::GetNewLandmarksSimulated(size_t t, const std::vector<Landmark>& visibleLandmarks, std::vector<Landmark>& newLandmarks)
    {
        arm->setPositions(trajectory[t]);
        PinholeCamera<Cal3_S2> camera(GetCameraPose(trajectory[t], simExtrinsic), *calib);
        PinholeCamera<Cal3_S2> estCamera(GetCameraPose(encoders[t]), *calib);

        for (size_t l = 0; l < simLandmarks.size(); l++)
        {
            const Point3& landmark = simLandmarks.at(l);
            try
            {
                std::pair<Point2, bool> uv = camera.projectSafe(landmark);

                if (uv.second && uv.first.x() > 0 && uv.first.y() > 0 && uv.first.x() < params.cx * 2 && uv.first.y() < params.cy * 2)
                {

                    if (landmarksObserved.find(l) == landmarksObserved.end())
                    {
                        groundTruth.insert(Symbol('l', l), simLandmarks.at(l));
                    }

                    Landmark newLandmark;
                    newLandmark.id = l;
                    Point2 xy = calib->calibrate(uv.first);
                    newLandmark.position = Point3(xy.x(), xy.y(), 1.0);
                    newLandmark.isTriangulated = false;
                    newLandmark.configs.push_back(t);
                    newLandmark.observations.push_back(uv.first);
                    newLandmarks.push_back(newLandmark);
                }
            }
            catch(CheiralityException& e)
            {
                continue;
            }
        }
        return newLandmarks.size() > 0;
    }

    bool ArmSlamCalib::SimulateObservationsTriangulate(size_t timeIndex)
    {
        gtsam::Vector q = encoders.at(timeIndex);
        gtsam::Vector q_gt = trajectory.at(timeIndex);

        gtsam::Pose3 cameraPose = GetCameraPose(q);

        std::vector<Landmark> newLandmarks;
        std::vector<Landmark> visibleLandmarks;
        GetVisibleLandmarks(q_gt, visibleLandmarks, simExtrinsic);

        if (!GetNewLandmarksSimulated(timeIndex, visibleLandmarks, newLandmarks))
            return false;

        std::vector<gtsam::Point3> worldLandmarks;
        for (size_t i = 0; i < newLandmarks.size(); i++)
        {
            gtsam::Point3 worldPoint = cameraPose.transform_from(newLandmarks.at(i).position);
            worldLandmarks.push_back(worldPoint);
        }


        for (size_t w = 0; w < worldLandmarks.size(); w++)
        {
            Point3 landmark = worldLandmarks.at(w);
            size_t i = newLandmarks.at(w).id;
            bool previouslyTriangulated = false;
            if (landmarksObserved.find(i) == landmarksObserved.end())
            {
                landmarksObserved[i] = Landmark();
                landmarksObserved[i].id = i;
                landmarksObserved[i].color = newLandmarks.at(w).color;
                landmarksObserved[i].position = worldLandmarks.at(w);
            }
            else
            {
                previouslyTriangulated = landmarksObserved[i].isTriangulated;
            }

            Landmark& matchedLandmark = landmarksObserved[i];
            matchedLandmark.cameraPoses.push_back(cameraPose);
            matchedLandmark.isTriangulated = newLandmarks.at(w).isTriangulated;
            matchedLandmark.configs.push_back(timeIndex);
            matchedLandmark.observations.push_back(newLandmarks.at(w).observations.at(0));
            matchedLandmark.descriptor = newLandmarks.at(w).descriptor;


            if (!matchedLandmark.isInGraph && matchedLandmark.observations.size() > 3)
            {
                size_t i = matchedLandmark.id;
                matchedLandmark.isNew = false;
                matchedLandmark.isInGraph = true;
                if (currentEstimate.find(Symbol('l', i)) != currentEstimate.end())
                    currentEstimate.erase(Symbol('l', i));
                if (initialEstimate.find(Symbol('l', i)) != initialEstimate.end())
                    initialEstimate.erase(Symbol('l', i));
                AddValue(Symbol('l', i), matchedLandmark.position);

                AddFactor(boost::make_shared<gtsam::PriorFactor<Point3> >(Symbol('l', i), matchedLandmark.position, landmarkPrior));
                for (size_t k = 0; k < matchedLandmark.observations.size(); k++)
                {
                    AddFactor(boost::make_shared<RobotProjectionFactor<Cal3_S2> >(
                            matchedLandmark.observations.at(k), measurementNoise,
                            Symbol('q', matchedLandmark.configs.at(k)), Symbol('l', i), Symbol('K', 0),
                            arm,  cameraBody,  robotMutex.get(), calib, matchedLandmark.configs.at(k), i, false, true));
                }
            }
            else if (matchedLandmark.isInGraph)
            {
                AddFactor(boost::make_shared<RobotProjectionFactor<Cal3_S2> >(
                        newLandmarks.at(w).observations.at(0), measurementNoise,
                        Symbol('q', timeIndex), Symbol('l', i), Symbol('K', 0),
                        arm,  cameraBody,  robotMutex.get(), calib, timeIndex, i, false, true));
            }

            if (matchedLandmark.observations.size() > 3)
                numObservationsThisIter++;
        }
        ROS_INFO("%lu observations", worldLandmarks.size());
        ROS_INFO("Trajectory size is now %lu", encoders.size());
        return true;
    }

    void ArmSlamCalib::ShowReprojectionErrors()
    {
        for (auto it = graph->begin(); it != graph->end(); it++)
        {
            gtsam::RobotProjectionFactor<Cal3_S2>* projection = dynamic_cast<gtsam::RobotProjectionFactor<Cal3_S2>*>(it->get());

            if (projection)
            {
                size_t trajIndex = projection->getTrajIndex();
                size_t landmarkIndex = projection->getLandmarkIndex();
                gtsam::Vector q = currentEstimate.at<gtsam::RobotConfig>(Symbol('q', trajIndex)).getQ();
                gtsam::Point3 l = currentEstimate.at<gtsam::Point3>(Symbol('l', landmarkIndex));
                gtsam::Pose3 ext = currentEstimate.at<gtsam::Pose3>(Symbol('K', 0));
                ShowReprojectionError(*projection,  q, l, ext,landmarkIndex,  images[trajIndex]);

            }
        }
    }

    void ArmSlamCalib::SaveGraph()
    {
        // save factor graph as graphviz dot file
        // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
        std::ofstream os("graph.dot");
        graph->saveGraph(os, currentEstimate);
    }

    void ArmSlamCalib::AddFactor(const gtsam::NonlinearFactor::shared_ptr& factor)
    {
        switch(params.optimizationMode)
        {
            case BatchLM:
            case BatchDogleg:
                graph->add(factor);
                break;
            case ISAM:
                graph->add(factor);
                newGraph->add(factor);
                break;
        }
    }

    void ArmSlamCalib::AddValue(const gtsam::Key& key, const gtsam::Value& value)
    {
        if (initialEstimate.find(key) == initialEstimate.end())
        {
            initialEstimate.insert(key, value);
        }
        if (currentEstimate.find(key) == currentEstimate.end())
        {
            currentEstimate.insert(key, value);
        }
    }

    void ArmSlamCalib::RunRansac(size_t timeA, size_t timeB)
    {
        gtsam::Vector qA = currentEstimate.at<gtsam::RobotConfig>(Symbol('q', timeA)).getQ();
        gtsam::Vector qB = currentEstimate.at<gtsam::RobotConfig>(Symbol('q', timeB)).getQ();
        gtsam::Pose3 poseA = GetCameraPose(qA);
        gtsam::Pose3 poseB = GetCameraPose(qB);
        std::vector<Landmark> landmarks;
        std::vector<gtsam::Point2> obsA;
        std::vector<gtsam::Point2> obsB;
        double motionEstimation = (poseB.translation() - poseA.translation()).norm();

        if (motionEstimation < 0.05)
        {
            return;
        }

        for (auto it = landmarksObserved.begin(); it != landmarksObserved.end(); it++)
        {
            const Landmark& landmark = it->second;

            if (!landmark.isTriangulated)
            {
                bool observedInA = false;
                bool observedInB = false;
                size_t idxA = 0;
                size_t idxB = 0;
                for (size_t k = 0; k < landmark.configs.size(); k++)
                {
                    if (landmark.configs.at(k) == timeA)
                    {
                        observedInA = true;
                        idxA = k;
                    }
                    else if (landmark.configs.at(k) == timeB)
                    {
                        observedInB = true;
                        idxB = k;
                    }
                }

                if (observedInA && observedInB)
                {
                    landmarks.push_back(landmark);
                    obsA.push_back(landmark.observations[idxA]);
                    obsB.push_back(landmark.observations[idxB]);
                }
            }
        }

        if (landmarks.size() >= 8)
        {
            ROS_INFO("Running ransac %lu %lu", timeA, timeB);
            ROS_INFO("%lu landmarks co-observed.", landmarks.size());
        }
        if (landmarks.size() >= 8)
        {
            opengv::relative_pose::RelativePoseAdapter adapter(poseA, poseB, *calib, landmarks, obsA, obsB);
            typedef opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem CentralRelativePoseSacProblem;
            opengv::sac::Ransac<CentralRelativePoseSacProblem> ransac;
            boost::shared_ptr<CentralRelativePoseSacProblem> problem(new CentralRelativePoseSacProblem(adapter, CentralRelativePoseSacProblem::STEWENIUS));
            ransac.sac_model_ = problem;
            ransac.threshold_ = 9;     //(1.0 - cos(0.5/600));
            ransac.max_iterations_ = 50;

            if (ransac.computeModel(1))
            {
                // assess success
                size_t rel_pose_inliers = ransac.inliers_.size();
                float rel_pose_ratio = float(rel_pose_inliers) / float(landmarks.size());

                if (rel_pose_inliers > 5 && rel_pose_ratio > 0.5f)
                {
                    ROS_WARN("GOT %lu ransac inliers!", rel_pose_inliers);
                    Eigen::Matrix4d relTransform = Eigen::Matrix4d::Identity();
                    relTransform.topLeftCorner<3, 4>() = ransac.model_coefficients_;
                    Eigen::Vector3d trans = Eigen::Vector3d(relTransform.topRightCorner<3, 1>()).normalized() * motionEstimation;
                    opengv::rotation_t rotation2 = poseA.rotation().matrix() * relTransform.topLeftCorner<3, 3>();
                    opengv::translation_t position2 = poseA.rotation().matrix() * (trans) + poseA.translation().vector();

                    gtsam::Pose3 newPoseB = gtsam::Pose3(gtsam::Rot3(rotation2), gtsam::Point3(position2));
                    std::vector<Landmark> inlierLandmarks;
                    std::vector<gtsam::Point2> inlinerPointsA;
                    std::vector<gtsam::Point2> inlierPointsB;
                    for (size_t k = 0; k < ransac.inliers_.size(); k++)
                    {
                        size_t inlier = (size_t)ransac.inliers_.at(k);
                        inlierLandmarks.push_back(landmarks.at(inlier));
                        inlinerPointsA.push_back(obsA.at(inlier));
                        inlierPointsB.push_back(obsB.at(inlier));
                    }
                    opengv::relative_pose::RelativePoseAdapter tringulationAdapter(poseA, newPoseB, *calib, inlierLandmarks, inlinerPointsA, inlierPointsB);

                    for (size_t j = 0; j < inlierLandmarks.size(); j++)
                    {
                        const Landmark& landmark = inlierLandmarks.at(j);
                        opengv::point_t pt = opengv::triangulation::triangulate(tringulationAdapter, j);

                        if (pt.norm() < 0.1) continue;

                        Landmark& existingLandmark = landmarksObserved[landmark.id];
                        existingLandmark.isTriangulated = true;
                        existingLandmark.position = poseA.transform_from(gtsam::Point3(pt));
                        ROS_INFO("New landmark position is %f %f %f", pt.x(), pt.y(), pt.z());
                        UpdateLandmarkPos(existingLandmark.id, existingLandmark.position);
                    }
                    //exit(-1);
                }
                else
                {
                    ROS_ERROR("There were only %lu/%lu inliers.", rel_pose_inliers, landmarks.size());
                }
            }
            else
            {
                ROS_ERROR("Ransac failed.");
            }

        }
    }

    void ArmSlamCalib::CalculateCurrentErrors(std::vector<double>& errors)
    {
        size_t numFactors = 0;
        for (auto it = graph->begin(); it != graph->end(); it++)
        {
            gtsam::RobotProjectionFactor<Cal3_S2>* projFactor = dynamic_cast<gtsam::RobotProjectionFactor<Cal3_S2>*>(it->get());

            if (projFactor)
            {
                double err = projFactor->evaluateError(currentEstimate.at<RobotConfig>(Symbol('q', projFactor->getTrajIndex())),
                                                       currentEstimate.at<gtsam::Point3>(Symbol('l', projFactor->getLandmarkIndex())),
                                                       currentEstimate.at<gtsam::Pose3>(Symbol('K', 0))).norm();
                errors.push_back(err);
            }
        }
    }


    void ArmSlamCalib::UpdateViewer()
    {
        if (params.drawEstimateRobot && encoders.size() > 0)
        {
            size_t idx = encoders.size() - 1;
            if (currentEstimate.find(Symbol('q', idx)) != currentEstimate.end())
                estimateArm->setPositions(currentEstimate.at<gtsam::RobotConfig>(Symbol('q', idx)).getQ());
        }

        viewer->update();
    }

    // Space seperated matrix of joint angle values.
    void ArmSlamCalib::LoadSimTrajectory(const std::string& fileName)
    {
        std::ifstream infile(fileName);

        std::default_random_engine generator(nh.param("seed", 0));
        std::normal_distribution<double> distribution(0, params.encoderNoiseLevel);
        bool firstIter = true;
        std::cout << "Reading trajectory for arm with " << arm->getNumDofs() << " dofs." << std::endl;
        while (infile)
        {
            std::string s;
            // For each line...
            if (!std::getline(infile, s))
                break;

            // Read all the joint values.
            std::istringstream ss(s);
            gtsam::Vector q = gtsam::Vector::Zero(arm->getNumDofs());
            int i = 0;
            while (ss)
            {
                std::string s;
                // Read the line until the end.
                if (!std::getline(ss, s, ' '))
                    break;

                std::stringstream convert(s);
                double jointAngle;
                convert >> jointAngle;
                q(i) = jointAngle;
                i++;
            }

            simTrajectory.push_back(q);

            if (!params.addSimPerlinNoise)
            {
                simEncoders.push_back(q);
            }
            else
            {
                if (firstIter)
                {
                    noiseOffset = GetPerlinNoise(q, params.simPerlinFrequency, params.simPerlinMagnitude);
                    firstIter = false;
                }
                gtsam::Vector noisy = q;
                for (size_t j = 0; j < arm->getNumDofs(); j++)
                {
                    noisy(j) += distribution(generator);
                }
                simEncoders.push_back(Wrap(noisy));
                //simEncoders.push_back(Wrap(q + GetPerlinNoise(q, params.simPerlinFrequency, params.simPerlinMagnitude) - noiseOffset));
                //simEncoders.push_back(Wrap(q + gtsam::Vector::Ones(arm->getNumDofs()) * 0.025));
            }
        }
        groundTruth.insert(Symbol('K', 0), simExtrinsic);
        params.trajectorySize = simTrajectory.size();
        simJointPublisher = nh.advertise<sensor_msgs::JointState>("/sim_joints", 10);
        simJointPublisher_groundtruth = nh.advertise<sensor_msgs::JointState>("/sim_joints_groundtruth", 10);
        simEEPublisher = nh.advertise<geometry_msgs::PoseStamped>("/in/pose", 10);

    }

    void ArmSlamCalib::UpdateLandmarkPos(size_t id, const gtsam::Point3& worldPoint)
    {
        const Landmark& l = landmarksObserved[id];

        if (!l.isInGraph)
        {
            return;
        }

        Key key = Symbol('l', id);

        for(auto it = graph->begin(); it != graph->end(); it++)
        {
            PriorFactor<gtsam::Point3>* prior = dynamic_cast<PriorFactor<gtsam::Point3>*>(it->get());

            if (prior && key == prior->key())
            {
                (*prior) = PriorFactor<gtsam::Point3>(key, worldPoint, landmarkPrior);

            }
        }

        switch (params.optimizationMode)
        {
            case BatchLM:
            case BatchDogleg:
            {
                if (currentEstimate.find(key) != currentEstimate.end())
                {
                    currentEstimate.erase(key);
                }
                if (initialEstimate.find(key) != initialEstimate.end())
                {
                    initialEstimate.erase(key);
                }
                currentEstimate.insert(key, worldPoint);
                initialEstimate.insert(key, worldPoint);
                break;
            }
            case ISAM:
            {
                if (currentEstimate.find(key) != currentEstimate.end())
                {
                    currentEstimate.erase(key);
                }
                if (initialEstimate.find(key) != initialEstimate.end())
                {
                    initialEstimate.erase(key);
                }
                currentEstimate.insert(key, worldPoint);
                initialEstimate.insert(key, worldPoint);

                isam->clear();
                newGraph->resize(0);

                *newGraph = graph->clone();

                break;
            }
        }
    }

    void ArmSlamCalib::PrintSimErrors(const std::string& dir, const std::string& postfix)
    {
        std::ofstream outFileJoints(dir + std::string("/joint_error") + postfix + std::string(".txt"), std::ios::out | std::ios::trunc);
        std::ofstream outFileExtrinsic(dir + std::string("/extrinsic_error") + postfix + std::string(".txt"), std::ios::out | std::ios::trunc);
        std::ofstream outFilePosition(dir + std::string("/position_error") + postfix + std::string(".txt"), std::ios::out | std::ios::trunc);
        std::ofstream outFileKinematics(dir + std::string("/kinematic_error") + postfix + std::string(".txt"), std::ios::out | std::ios::trunc);

        gtsam::Pose3 extCur = currentEstimate.at<gtsam::Pose3>(Symbol('K', 0));
        gtsam::Quaternion extCurQ = extCur.rotation().toQuaternion();
        gtsam::Quaternion simExtQ = extCur.rotation().toQuaternion();
        outFileExtrinsic << simExtrinsic.translation().x() << " "
                         << simExtrinsic.translation().y() << " "
                         << simExtrinsic.translation().z() << " "
                         << simExtQ.x() << " "
                         << simExtQ.y() << " "
                         << simExtQ.z() << " "
                         << simExtQ.w() << " ";

        outFileExtrinsic << extCur.translation().x() << " "
                         << extCur.translation().y() << " "
                         << extCur.translation().z() << " "
                         << extCurQ.x() << " "
                         << extCurQ.y() << " "
                         << extCurQ.z() << " "
                         << extCurQ.w() << " ";

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            gtsam::Vector gt = trajectory.at(i);
            gtsam::Vector enc = encoders.at(i);

            if (currentEstimate.find(Symbol('q', i)) != currentEstimate.end())
            {
                RobotConfig qi = currentEstimate.at<RobotConfig>(Symbol('q', i));
                gtsam::Vector qi_q = qi.getQ();

                for (size_t j = 0; j < arm->getNumDofs(); j++)
                {
                    outFileJoints << gt(j) << " ";
                }

                for (size_t j = 0; j < arm->getNumDofs(); j++)
                {
                    outFileJoints << qi_q(j);
                    outFileJoints << " ";
                }

                for (size_t j = 0; j < arm->getNumDofs(); j++)
                {
                    outFileJoints << enc(j);

                    if (j < arm->getNumDofs() - 1)
                    {
                        outFileJoints << " ";
                    }
                }

                outFileJoints << std::endl;


                gtsam::Pose3 poseGT = GetCameraPose(gt, simExtrinsic);
                gtsam::Pose3 poseI = GetCameraPose(qi_q);
                gtsam::Pose3 pose_kinematics_gt = GetCameraPose(gt, simExtrinsic);
                gtsam::Pose3 pose_kinematics_i = GetCameraPose(qi_q, simExtrinsic);
                gtsam::Pose3 pose_kinematics_enc = GetCameraPose(enc, simExtrinsic);

                outFilePosition << poseGT.translation().x() << " " << poseGT.translation().y() << " " << poseGT.translation().z() << " " <<
                                   poseI.translation().x() << " " << poseI.translation().y() << " " << poseI.translation().z() << std::endl;

                outFileKinematics << pose_kinematics_gt.translation().x() << " " << pose_kinematics_gt.translation().y() << " " << pose_kinematics_gt.translation().z() << " " <<
                                     pose_kinematics_i.translation().x() << " " << pose_kinematics_i.translation().y() << " " << pose_kinematics_i.translation().z() << " " <<
                                     pose_kinematics_enc.translation().x() << " " << pose_kinematics_enc.translation().y() << " " << pose_kinematics_enc.translation().z() <<std::endl;

            }
            else continue;

        }

    }

    gtsam::Vector ArmSlamCalib::GetPerlinNoise(const gtsam::Vector& pos, const double& freq, const double& mag)
    {
        gtsam::Vector toReturn = pos;
        toReturn.setZero();

        for (size_t j = 0; j < pos.size(); j++)
        {
            toReturn(j) = (perlin.noise((double)(j + 1) * 100, -(double)(j + 1) * 999, (double)pos(j) * freq) - 0.5) * mag;
        }
        return toReturn;
    }

    void ArmSlamCalib::SaveStitchedPointClouds(const std::string& file)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr compositeCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
        pcl_conversions::toPCL(ros::Time::now(), compositeCloud->header.stamp);
        compositeCloud->header.frame_id = "/map";
        compositeCloud->is_dense = false;
        compositeCloud->width = 1;
        for (size_t t = 0; t < trajectory.size(); t++)
        {
            if (currentEstimate.find(Symbol('q', t)) == currentEstimate.end())
            {
                continue;
            }
           RobotConfig estQ = currentEstimate.at<RobotConfig>(Symbol('q', t));
           gtsam::Pose3 estPose = GetCameraPose(estQ.getQ());
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pointClouds.at(t);
           pcl::PointCloud<pcl::PointXYZRGB> transformed;
           pcl::transformPointCloud(*cloud, transformed, estPose.matrix());
           compositeCloud->points.insert(compositeCloud->points.end(), transformed.begin(), transformed.end());
        }
        compositeCloud->height = compositeCloud->points.size();
        pcl::io::savePCDFileASCII(file.c_str(), *compositeCloud);
    }

} /* namespace gtsam */
