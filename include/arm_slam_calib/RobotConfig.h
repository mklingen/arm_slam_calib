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
                return RobotConfig(q + delta, arm);
            }

            // localCoordinates working directly with RobotConfig objects (non-virtual, non-overriding!)
            inline Vector localCoordinates(const RobotConfig& r2) const
            {
                return Vector(r2.q - q);
            }
    };
}

#endif /* ROBOTCONFIG_H_ */
