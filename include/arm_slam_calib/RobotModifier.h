/*
 * RobotModifer.h
 *
 *  Created on: Apr 12, 2016
 *      Author: mklingen
 */

#ifndef ROBOTMODIFER_H_
#define ROBOTMODIFER_H_

#include <mutex>
#include <gtsam/geometry/Pose3.h>
#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/Group.h>
#include <dart/dart.h>
#include <vector>

namespace gtsam
{
    class RobotCalibration : public gtsam::DerivedValue<RobotCalibration>
    {
        public:
            RobotCalibration();
            RobotCalibration(const dart::dynamics::GroupPtr& arm_,
                             const Eigen::aligned_vector<gtsam::Pose3>& jointPoseOffsets_,
                             const std::vector<bool>& modifyPoses_);

            inline dart::dynamics::GroupPtr GetArm() const { return arm; }
            inline const Eigen::aligned_vector<gtsam::Pose3>& GetPoseOffsets() const { return jointPoseOffsets; }
            inline const Eigen::aligned_vector<Eigen::Isometry3d>& GetOriginalPoses() const { return originalJointPoses; }
            inline const std::vector<bool>& GetModifyPoses() const { return modifyPoses; }

            // Print for unit tests and debugging (virtual, implements Value::print())
            virtual void print(const std::string& str = "") const
            {
                for (size_t i = 0; i < jointPoseOffsets.size(); i++)
                {
                    std::cout << jointPoseOffsets.at(i).matrix() << std::endl;
                }
            }

            // Equals working directly with Rot3 objects (non-virtual, non-overriding!)
            bool equals(const RobotCalibration& other, double tol = 1e-9) const
            {
                for (size_t i = 0; i < jointPoseOffsets.size(); i++)
                {
                    if (!other.GetPoseOffsets().at(i).equals(jointPoseOffsets.at(i), tol))
                    {
                        return false;
                    }
                }
                return true;
            }

            // Tangent space dimensionality (virtual, implements Value::dim())
            virtual size_t dim() const
            {
                return jointPoseOffsets.size() * 6;
            }

            // retract working directly with RobotConfig objects (non-virtual, non-overriding!)
            inline RobotCalibration retract(const Vector& delta) const
            {
                Eigen::aligned_vector<gtsam::Pose3> newPoses = jointPoseOffsets;
                for (size_t i = 0; i < jointPoseOffsets.size(); i++)
                {
                    gtsam::Vector subretraction = delta.block(0, i * 6, 1, 6);
                    newPoses[i] = jointPoseOffsets.at(i).retract(subretraction);
                }
                return RobotCalibration(arm, newPoses, modifyPoses);
            }

            // localCoordinates working directly with RobotConfig objects (non-virtual, non-overriding!)
            inline Vector localCoordinates(const RobotCalibration& r2) const
            {
                gtsam::Vector output = gtsam::Vector::Zero(dim());
                for (size_t i = 0; i < jointPoseOffsets.size(); i++)
                {
                    output.block(0, i * 6, 1, 6) = jointPoseOffsets.at(i).localCoordinates(r2.GetPoseOffsets().at(i));
                }

                return output;
            }

        protected:
            dart::dynamics::GroupPtr arm;
            Eigen::aligned_vector<gtsam::Pose3> jointPoseOffsets;
            Eigen::aligned_vector<Eigen::Isometry3d> originalJointPoses;
            std::vector<bool> modifyPoses;
    };

    class RobotModifier
    {
        public:
            RobotModifier();
            RobotModifier(std::mutex* mutex_,
                          const RobotCalibration& calibration_,
                          const gtsam::Vector& q_);

            virtual ~RobotModifier();

            void Apply();
            void Unapply();

        protected:
            RobotCalibration calibration;
            gtsam::Vector q;
            gtsam::Vector originalQ;
            std::mutex* mutex;
            bool modified;
    };

} /* namespace gtsam */

#endif /* ROBOTMODIFER_H_ */
