/*
 * RobotModifer.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: mklingen
 */

#include <arm_slam_calib/RobotModifier.h>

namespace gtsam
{

    RobotCalibration::RobotCalibration()
    {

    }

    RobotCalibration::RobotCalibration(const dart::dynamics::GroupPtr& arm_,
                     const Eigen::aligned_vector<gtsam::Pose3>& jointPoseOffsets_,
                     const std::vector<bool>& modifyPoses_) :
                             arm(arm_), jointPoseOffsets(jointPoseOffsets_), modifyPoses(modifyPoses_)
    {
        for (size_t i = 0 ; i < jointPoseOffsets.size(); i++)
        {
            dart::dynamics::Joint* joint = arm->getJoint(i);
            originalJointPoses.push_back(joint->getTransformFromParentBodyNode());
        }
    }

    RobotModifier::RobotModifier() :
            mutex(0x0),
            modified(false)

    {

    }

    RobotModifier::RobotModifier(std::mutex* mutex_,
                 const RobotCalibration& calibration_,
                 const gtsam::Vector& q_) :
                         calibration(calibration_),
                         q(q_),
                         modified(false)
    {
        originalQ = calibration.GetArm()->getPositions();
        Apply();
    }

    RobotModifier::~RobotModifier()
    {
        Unapply();
    }

    void RobotModifier::Apply()
    {
        if (modified) return;

        std::lock_guard<std::mutex>(*mutex);

        calibration.GetArm()->setPositions(q);

        for (size_t i = 0 ; i < calibration.GetPoseOffsets().size(); i++)
        {
            if (!calibration.GetModifyPoses().at(i)) continue;

            dart::dynamics::Joint* joint = calibration.GetArm()->getJoint(i);

            joint->setTransformFromParentBodyNode(
                    Eigen::Isometry3d(joint->getTransformFromParentBodyNode().matrix() * calibration.GetPoseOffsets().at(i).matrix()));
        }

        modified = true;
    }

    void RobotModifier::Unapply()
    {
        if (!modified) return;

        std::lock_guard<std::mutex>(*mutex);

        calibration.GetArm()->setPositions(originalQ);

        for (size_t i = 0 ; i < calibration.GetPoseOffsets().size(); i++)
        {
            if (!calibration.GetModifyPoses().at(i)) continue;

            calibration.GetArm()->getJoint(i)->setTransformFromParentBodyNode(calibration.GetOriginalPoses().at(i));
        }
    }

} /* namespace gtsam */
