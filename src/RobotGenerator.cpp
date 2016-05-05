/*
 * RobotGenerator.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: mklingen
 */
#include <arm_slam_calib/Utils.h>
#include <arm_slam_calib/RobotGenerator.h>
using namespace dart::dynamics;
namespace dart
{

    RobotGenerator::RobotGenerator()
    {
    }

    RobotGenerator::~RobotGenerator()
    {

    }

    SkeletonPtr RobotGenerator::GenerateRobot(size_t numDof,
                                              float scale,
                                              float growth,
                                              float randomLength,
                                              float randomAngle,
                                              bool movableBase,
                                              size_t baseDofs)
    {
        Skeleton::Properties properties;
        properties.mEnabledAdjacentBodyCheck = true;
        properties.mEnabledSelfCollisionCheck = true;
        properties.mGravity = Eigen::Vector3d::Zero();
        properties.mIsMobile = false;
        properties.mName = "Random robot";
        properties.mVersion = 0;

        SkeletonPtr robot = Skeleton::create(properties);


        BodyNode* parentLink = 0x0;

        if (movableBase)
        {
            BodyNode::Properties linkProperties;
            linkProperties.mName = "base_link";

            switch (baseDofs)
            {
                case 3:
                {
                    PlanarJoint::Properties jointProperties;
                    jointProperties.mName = "base";
                    jointProperties.mPlaneType = PlanarJoint::PlaneType::XY;
                    jointProperties.mIsPositionLimited = false;
                    jointProperties.mRotAxis = Eigen::Vector3d::UnitZ();
                    jointProperties.mTransAxis1 = Eigen::Vector3d::UnitX();
                    jointProperties.mTransAxis2 = Eigen::Vector3d::UnitY();

                    std::pair<PlanarJoint*, BodyNode*> jointAndLink =
                            robot->createJointAndBodyNodePair<PlanarJoint, BodyNode>(parentLink, jointProperties, linkProperties);
                    parentLink = jointAndLink.second;

                    break;
                }
                case 6:
                {
                    FreeJoint::Properties jointProperties;
                    jointProperties.mName = "base";

                    std::pair<FreeJoint*, BodyNode*> jointAndLink = robot->createJointAndBodyNodePair<FreeJoint, BodyNode>(parentLink, jointProperties, linkProperties);
                    parentLink = jointAndLink.second;

                    break;
                }
            }
            auto shapeNode
                = parentLink->createShapeNodeWith<VisualAddon>(std::make_shared<BoxShape>(Eigen::Vector3d(scale * 0.5, scale * 0.5, scale * 0.1)));
            shapeNode->getVisualAddon()->setColor(Eigen::Vector3d(0, 0, 1));
        }


        float lastLinklength = 0;
        float linkScale = scale;
        for (size_t i = 0; i < numDof; i++)
        {
            double parity = i % 2 == 0 ? 1.0 : -1.0;
            std::stringstream dofName;
            std::stringstream linkName;
            dofName << "j" << i;
            linkName << "l" << i;
            RevoluteJoint::Properties jointProperties;
            jointProperties.mDofName = dofName.str();
            jointProperties.mName = dofName.str();
            jointProperties.mInitialPosition = 0.0;
            jointProperties.mInitialVelocity = 0;
            jointProperties.mPositionUpperLimit = 3.0;
            jointProperties.mPositionLowerLimit = -3.0;
            jointProperties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(lastLinklength * 0.5 + linkScale * 0.125, 0, 0);

            if (i < numDof - 1)
            {
                jointProperties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(-lastLinklength * 0.25, 0, 0);
            }
            else
            {
                jointProperties.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();
                jointProperties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(-lastLinklength * 0.25, 0, 0);
                jointProperties.mT_ChildBodyToJoint.linear() =  Eigen::AngleAxisd(1.57, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
            }
            if (i > 0)
            {
                if (parity < 0)
                    jointProperties.mT_ParentBodyToJoint.linear() = Eigen::AngleAxisd(parity * 1.57 + ::utils::Rand(-randomAngle, randomAngle), Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
                else
                    jointProperties.mT_ParentBodyToJoint.linear() = Eigen::AngleAxisd(parity * 1.57 + ::utils::Rand(-randomAngle, randomAngle), Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
            }
            else
            {
                jointProperties.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
                jointProperties.mT_ParentBodyToJoint.linear() = Eigen::AngleAxisd(-1.57, Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
                jointProperties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(-scale * 0.5, 0, 0);
            }

            jointProperties.mAxis =  Eigen::Vector3d(0, 0, 1);
            lastLinklength = linkScale;
            linkScale *= growth;
            linkScale += ::utils::Rand(-randomLength, randomLength);
            linkScale = fmax(linkScale, 0.01f);
            BodyNode::Properties linkProperties;
            linkProperties.mName = linkName.str();

            std::pair<RevoluteJoint*, BodyNode*> jointAndLink = robot->createJointAndBodyNodePair<RevoluteJoint, BodyNode>(parentLink, jointProperties, linkProperties);
            parentLink = jointAndLink.second;

            auto shapeNode
                = parentLink->createShapeNodeWith<VisualAddon>(std::make_shared<BoxShape>(Eigen::Vector3d(lastLinklength, lastLinklength * 0.25, lastLinklength * 0.15)));

        }

        //BodyNode* ee = robot->getBodyNode(numDof - 1);
        //Eigen::Isometry3d pose = ee->getWorldTransform();

        //Joint* joint0 = robot->getJoint(0);
        //joint0->setTransformFromParentBodyNode(pose.inverse());
        return robot;
    }

} /* namespace dart */
