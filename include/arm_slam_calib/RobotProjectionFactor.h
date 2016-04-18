/*
 * RobotProjectionFactor.h
 *
 *  Created on: Jan 22, 2016
 *      Author: mklingen
 */

#ifndef ROBOTPROJECTIONFACTOR_H_
#define ROBOTPROJECTIONFACTOR_H_

#include <dart/dart.h>
#include <dart/dynamics/Group.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <thread>
#include <arm_slam_calib/RobotProjectionFactor.h>
#include <arm_slam_calib/RobotConfig.h>
#include <arm_slam_calib/RobotModifier.h>

//#define SANITY_CHECK_JACOBIANS
namespace gtsam
{
    // Correlates together the robot's joint angles, a landmark, the extrinsics of the camera,
    // and the kinematic model of the robot.
    template <class CameraCalibration>
    class RobotProjectionFactor : public NoiseModelFactor4<RobotConfig, Point3, Pose3, RobotCalibration>
    {
        protected:
            Point2 measurement;
            boost::shared_ptr<CameraCalibration> K;
            dart::dynamics::MetaSkeletonPtr robot;
            dart::dynamics::BodyNode* linkBody;
            std::mutex* robotMutex;
            size_t trajIndex;
            size_t landmarkIndex;

            bool throwCheirality;
            bool verboseCheirality;

        public:

            typedef NoiseModelFactor4<RobotConfig, Point3, Pose3, RobotCalibration> Base;
            typedef RobotProjectionFactor<CameraCalibration> This;
            typedef boost::shared_ptr<This> shared_ptr;

            inline const Point2 getMeasurement() { return measurement; }
            inline size_t getTrajIndex() { return trajIndex; }
            inline size_t getLandmarkIndex() { return landmarkIndex; }

            RobotProjectionFactor() : linkBody(0x0), robotMutex(0x0), throwCheirality(false), verboseCheirality(false), landmarkIndex(0), trajIndex(0) {};

            RobotProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
                    Key robotKey, Key landmarkKey, Key extrinsicKey, Key calibKey,
                    const dart::dynamics::MetaSkeletonPtr& robot_,
                    dart::dynamics::BodyNode* body_,
                    std::mutex* robotMutex_,
                    const boost::shared_ptr<CameraCalibration>& calibration,
                    size_t traj,
                    size_t landmark,
                    bool throwCheirality_ = false,
                    bool verboseCheirality_ = false) :
                        Base(model, robotKey, landmarkKey, extrinsicKey, calibKey),
                        measurement(measured),
                        K(calibration),
                        robot(robot_),
                        linkBody(body_),
                        robotMutex(robotMutex_),
                        throwCheirality(throwCheirality_),
                        verboseCheirality(verboseCheirality_),
                        trajIndex(traj),
                        landmarkIndex(landmark)
            {

            }

            virtual ~RobotProjectionFactor() {}

            virtual gtsam::NonlinearFactor::shared_ptr clone() const
            {
                return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
            }

            void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter)
            {
                std::cout << s << "RobotProjectionFactor, z =";
                measurement.print();
                Base::print("", keyFormatter);
            }

            virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const
            {
                const This* e = dynamic_cast<const This*>(&p);
                return e
                        && this->measurement.equals(e->measurement, tol)
                        && this->K->equals(*(e->K), tol);
            }


            Vector evaluateError(const X1& config,
                                 const X2& landmark,
                                 const X3& extrinsic,
                                 const X4& kinematicCalibration,
                                 boost::optional<Matrix&> J1 = boost::none,
                                 boost::optional<Matrix&> J2 = boost::none,
                                 boost::optional<Matrix&> J3 = boost::none,
                                 boost::optional<Matrix&> J4 = boost::none) const
            {
                try
                {
                    Point2 projection = project(config.getQ(), landmark, extrinsic, kinematicCalibration, J1, J2, J3, J4);
                    Point2 err(projection - measurement);
#ifdef SANITY_CHECK_JACOBIANS
                    if (J1 || J2 || J3 || J4)
                    {
                        Matrix j1Diff = zeros(2, config.dim());
                        Matrix j2Diff = zeros(2, 3);
                        Matrix j3Diff = zeros(2, 6);
                        Matrix j4Diff = zeros(2, config.dim() * 6);

                        double diffQSize = 0.01;
                        for (size_t j = 0; j < config.dim(); j++)
                        {
                            Vector dQ = zeros(config.dim(), 1);
                            dQ(j) += diffQSize;

                            Point2 ptPlus = project(dQ + config.getQ(), landmark, extrinsic, kinematicCalibration);
                            Point2 ptMinus = project(config.getQ() - dQ, landmark, extrinsic, kinematicCalibration);

                            Point2 centralDifference =  (ptPlus - ptMinus) / (2 * diffQSize);
                            j1Diff.col(j) = centralDifference.vector();
                        }

                        float diffLandmarkSize = 0.001;
                        for (size_t k = 0; k < 3; k++)
                        {

                            Point3 dLandmark(k == 0 ? diffLandmarkSize : 0, k == 1 ? diffLandmarkSize : 0 , k == 2 ? diffLandmarkSize : 0);

                            Point2 ptPlus = project(config.getQ(), landmark + dLandmark, extrinsic, kinematicCalibration);
                            Point2 ptMinus = project(config.getQ(), landmark - dLandmark, extrinsic, kinematicCalibration);

                            Point2 centralDifference =  (ptPlus - ptMinus) / (2 * diffLandmarkSize);
                            j2Diff.col(k) = centralDifference.vector();
                        }

                        float diffExtrinsic = 0.001;
                        for (size_t k = 0; k < 6; k++)
                        {
                            gtsam::Vector6 dPose = zeros(6, 1);
                            dPose(k) = diffExtrinsic;
                            Point2 ptPlus = project(config.getQ(), landmark, extrinsic.retract(dPose), kinematicCalibration);
                            Point2 ptMinus = project(config.getQ(), landmark, extrinsic.retract(-1.0 * dPose), kinematicCalibration);

                            Point2 centralDifference =  (ptPlus - ptMinus) / (2 * diffExtrinsic);
                            j3Diff.col(k) = centralDifference.vector();
                        }

                        for (size_t j = 0; j < config.dim(); j++)
                        {
                            for (size_t k = 0; k < 6; k++)
                            {
                                gtsam::Vector dJoint = Vector::Zero(6 * config.dim());
                                dJoint(k + j * 6) = diffExtrinsic;
                                Point2 ptPlus = project(config.getQ(), landmark, extrinsic, kinematicCalibration.retract(dJoint));
                                Point2 ptMinus = project(config.getQ(), landmark, extrinsic, kinematicCalibration.retract(-1.0 * dJoint));

                                Point2 centralDifference =  (ptPlus - ptMinus) / (2 * diffExtrinsic);
                                j4Diff.col(k + j * 6) = centralDifference.vector();
                            }
                        }

                        robotMutex->lock();
                        std::flush(std::cerr);
                        std::cerr << "J1: " << std::endl << *J1 << std::endl;
                        std::cerr << "J1(sanity): " << std::endl << j1Diff << std::endl;
                        std::cerr << "J2: " << std::endl << *J2 << std::endl;
                        std::cerr << "J2(sanity): " << std::endl << j2Diff << std::endl;
                        std::cerr << "J3: " << std::endl << *J3 << std::endl;
                        std::cerr << "J3(sanity): " << std::endl << j3Diff << std::endl;
                        std::cerr << "J4: " << std::endl << (*J4).transpose() << std::endl;
                        std::cerr << "J4(sanity): " << std::endl << j4Diff.transpose() << std::endl;
                        robotMutex->unlock();
                        while(true)
                        {
                            sleep(1);
                        }
                    }
#endif
                    return err.vector();
                }
                catch(CheiralityException& e)
                {
                    if (J1) *J1 = zeros(2, robot->getNumDofs());
                    if (J2) *J2 = zeros(2, 3);
                    if (J3) *J3 = zeros(2, 6);
                    if (J4) *J4 = zeros(2, robot->getNumDofs() * 6);

                    if (verboseCheirality)
                    {
                        std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
                          " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
                    }
                    if (throwCheirality)
                    {
                        throw e;
                    }
                    return  ones(2) * 2.0 * K->fx();
                }
            }


            // worldpoint -- the position of the landmark in the world.
            // extrinsic -- the extrinsic calibration of the camera w.r.t robot.
            // J1 -- jacobian w.r.t configuration of robot (2 x N)
            // J2 -- jacobian w.r.t landmark (2 x 3)
            // J3 -- jacobian w.r.t extrinsic pose (2 x 6?)
            gtsam::Point2 project(const Vector& q,
                                  const gtsam::Point3& worldPoint,
                                  const gtsam::Pose3& extrinsic,
                                  const RobotCalibration& calibration,
                                  boost::optional<Matrix&> J1 = boost::none,
                                  boost::optional<Matrix&> J2 = boost::none,
                                  boost::optional<Matrix&> J3 = boost::none,
                                  boost::optional<Matrix&> J4 = boost::none) const
            {
                std::lock_guard<std::mutex> lock(*robotMutex);
                RobotModifier modifier(calibration, q);
                Eigen::Isometry3d linkPose = linkBody->getWorldTransform();
                Eigen::Matrix4d cameraPose = linkPose.matrix() * extrinsic.matrix();
                PinholeCamera<CameraCalibration> cameraCopy1(Pose3(cameraPose), *K);

                gtsam::Point3 cameraPoint = cameraCopy1.pose().transform_to(worldPoint);

                if (cameraPoint.z() < 0.1)
                {
                    if (J1) *J1 = zeros(2, robot->getNumDofs());
                    if (J2) *J2 = zeros(2, 3);
                    if (J3) *J3 = zeros(2, 6);
                    if (J4) *J4 = zeros(2, 6 * robot->getNumDofs());
                    return  gtsam::Point2(1, 1) * 2.0 * K->fx();
                }

                gtsam::Matrix duvDLandmark1;
                gtsam::Matrix duvDLandmark2;
                gtsam::Matrix duvDExtrinsic;
                gtsam::Point2 uv = cameraCopy1.project(worldPoint, duvDExtrinsic, duvDLandmark1);

                if (J1 || J2 || J3 || J4)
                {
                    // TODO: Does this make any sense?
                    *J2 = duvDLandmark1;

                    // TODO: Does this make any sense?

                    // Set the robot's config to the variable in question so
                    // we can compute forward kinematics.
                    robot->setPositions(q);
                    // Compute forward kinematics.
                    // Compute linear jacobian of the robot w.r.t the point in question,
                    // in the world coordinate frame.
                    Eigen::Vector3d relativePoint = linkPose.inverse() * worldPoint.vector();
                    dart::math::LinearJacobian Jrobot = robot->getLinearJacobian(linkBody, relativePoint);

                    // Now, I think I can just multiply the landmark's jacobian by
                    // the robot's jacobian. This is an application of the chain rule?
                    // The sizes check out: (2 x 3) x (3 x N) = (2 x N)
                    *J1 = -(duvDLandmark1 * Jrobot);

                    // TODO: What is jacobian wrt extrinsic?
                    // Make it so that everything is in the link's frame of reference...
                    PinholeCamera<CameraCalibration> cameraCopy2(extrinsic, *K);

                    // Project onto the camera in the link's frame of reference. Now we
                    // have the jacobian of the extrinsic!
                    cameraCopy2.project(gtsam::Point3(relativePoint), duvDExtrinsic, duvDLandmark2);
                    *J3 = duvDExtrinsic;


                    gtsam::Matrix j4 = zeros(2, robot->getNumDofs() * 6);
                    // Now to compute J4 (d measurement d kinematic model)
                    // Foreach joint..
                    for (size_t i = 0; i < robot->getNumDofs(); i++)
                    {
                        dart::dynamics::Joint* joint = robot->getJoint(i);

                        // Get the transformation of the joint
                        Eigen::Isometry3d jointTransform =  joint->getParentBodyNode()->getWorldTransform() * joint->getTransformFromParentBodyNode();
                        gtsam::Pose3 jointPose(jointTransform.matrix());
                        gtsam::Matrix dLandmarkdPose;
                        // Get the landmark relative to the joint's frame of reference
                        Point3 jointPoint = jointPose.transform_to(worldPoint);
                        Point3 worldPoint2 = jointPose.transform_from(jointPoint, dLandmarkdPose);

                        // TODO:?
                        // Now we have the jacobian of the landmark wrt to the joint pose? I think?
                        // The next step is to premultiply by the jacobian of the measurement with respect
                        // to the landmark jacobian.
                        j4.block(0, i * 6, 2, 6) = duvDLandmark1 * dLandmarkdPose;
                    }
                    *J4 = -j4;
                }
                return uv;
            }
    };

} /* namespace gtsam */

#endif /* ROBOTPROJECTIONFACTOR_H_ */
