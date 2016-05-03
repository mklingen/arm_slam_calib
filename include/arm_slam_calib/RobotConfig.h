/*
 * RobotConfig.h
 *
 *  Created on: Apr 12, 2016
 *      Author: mklingen
 */

#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_

#include <dart/dart.h>
#include <dart/dynamics/Group.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam
{
    class GTSAM_EXPORT RobotConfig: public DerivedValue<RobotConfig>
    {
        protected:
            Vector q;
            dart::dynamics::GroupPtr arm;

        public:

            inline const Vector& getQ() const
            {
                return q;
            }

            RobotConfig()
            {

            }

            bool IsFree(size_t index) const
            {
                const dart::dynamics::DegreeOfFreedom* dof = arm->getDof(index);
                const dart::dynamics::FreeJoint* freeJoint = dynamic_cast<const dart::dynamics::FreeJoint*>(dof->getJoint());
                if (freeJoint)
                    return true;
                else return false;
            }


            bool IsContinuous(size_t index) const
            {
                const double& tol = 1e-5;
                const dart::dynamics::DegreeOfFreedom* dof = arm->getDof(index);
                return fabs(dof->getPositionLowerLimit() - dof->getPositionUpperLimit()) < tol;
            }


            // Constructor, there is never a need to call the Value base class constructor.
            RobotConfig(const Vector& q_, const dart::dynamics::GroupPtr& arm_) :
                q(q_), arm(arm_)
            {

            }

            // Print for unit tests and debugging (virtual, implements Value::print())
            virtual void print(const std::string& str = "") const
            {
                std::cout << str << q.transpose() << std::endl;
            }

            // Equals working directly with Rot3 objects (non-virtual, non-overriding!)
            bool equals(const RobotConfig& other, double tol = 1e-9) const
            {
                return (other.q - q).norm() < tol;
            }

            // Tangent space dimensionality (virtual, implements Value::dim())
            virtual size_t dim() const
            {
                return q.rows();
            }

            // retract working directly with RobotConfig objects (non-virtual, non-overriding!)
            inline RobotConfig retract(const Vector& delta) const
            {
                Vector sum = q + delta;
                bool hasFreeBase = false;
                for (size_t k = 0; k < dim(); k++)
                {
                    if (IsFree(k))
                    {
                        hasFreeBase = true;
                        break;
                    }
                }

                if (hasFreeBase)
                {
                    Eigen::Isometry3d pose = dart::dynamics::FreeJoint::convertToTransform(getQ().head(6));
                    gtsam::Pose3 gt_pose(pose.matrix());
                    gtsam::Pose3 retracted = gt_pose.retract(delta.head(6));
                    //Eigen::Isometry3d newPose =  dart::math::expMap(delta.head(6)) * pose;
                    sum.head(6) = dart::dynamics::FreeJoint::convertToPositions(Eigen::Isometry3d(retracted.matrix()));
                }

                return RobotConfig(sum, arm);
            }

            // localCoordinates working directly with RobotConfig objects (non-virtual, non-overriding!)
            inline Vector localCoordinates(const RobotConfig& r2) const
            {
                Vector sum = r2.q - q;
                bool hasFreeBase = false;
                for (size_t k = 0; k < dim(); k++)
                {
                  if (IsFree(k))
                  {
                      hasFreeBase = true;
                      break;
                  }
                }

                if (hasFreeBase)
                {
                  Eigen::Isometry3d pose = dart::dynamics::FreeJoint::convertToTransform(getQ().head(6));
                  Eigen::Isometry3d pose2 = dart::dynamics::FreeJoint::convertToTransform(r2.getQ().head(6));
                  gtsam::Pose3 gt_pose(pose.matrix());
                  gtsam::Pose3 gt_pose2(pose2.matrix());
                  sum.head(6) = gt_pose.localCoordinates(gt_pose2);
                }

                return sum;
            }
    };
}

#endif /* ROBOTCONFIG_H_ */
