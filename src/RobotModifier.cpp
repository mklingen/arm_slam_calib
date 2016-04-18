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
                             arm(arm_), jointPoseOffsets(jointPoseOffsets_), modifyPoses(modifyPoses_), originalJointPoses(jointPoseOffsets_.size())
    {

        for (size_t i = 0 ; i < jointPoseOffsets.size(); i++)
        {
            dart::dynamics::Joint* joint = arm->getJoint(i);
            originalJointPoses.at(i) = joint->getTransformFromParentBodyNode();
        }
    }



    RobotModifier::RobotModifier(const RobotCalibration& calibration_,
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

    RobotCalibration RobotCalibration::Identity(const dart::dynamics::GroupPtr& arm_)
    {
        Eigen::aligned_vector<gtsam::Pose3> jointPoseOffsets;
        std::vector<bool> modifyPoses;

        for (size_t i = 0; i < arm_->getNumDofs(); i++)
        {
            jointPoseOffsets.push_back(gtsam::Pose3::identity());
            modifyPoses.push_back(true);
        }

        return RobotCalibration(arm_, jointPoseOffsets, modifyPoses);
    }

    void RobotModifier::Apply()
    {
        if (modified) return;

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

        calibration.GetArm()->setPositions(originalQ);

        for (size_t i = 0 ; i < calibration.GetPoseOffsets().size(); i++)
        {
            if (!calibration.GetModifyPoses().at(i)) continue;

            calibration.GetArm()->getJoint(i)->setTransformFromParentBodyNode(calibration.GetOriginalPoses().at(i));
        }
    }

} /* namespace gtsam */
