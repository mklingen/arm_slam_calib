/*
 * EncoderFactor.h
 *
 *  Created on: Jan 22, 2016
 *      Author: mklingen
 */

#ifndef ENCODERFACTOR_H_
#define ENCODERFACTOR_H_

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <arm_slam_calib/RobotProjectionFactor.h>
#include <arm_slam_calib/RobotConfig.h>
#include <arm_slam_calib/Utils.h>
//#define SANITY_CHECK

namespace gtsam
{

    class EncoderFactor : public NoiseModelFactor1<RobotConfig>
    {
        protected:
            Vector encoders;
            double deadBand;
            bool hasDeadBand;
        public:
            typedef NoiseModelFactor1<RobotConfig> Base;
            typedef boost::shared_ptr<EncoderFactor> shared_ptr;

            EncoderFactor(Key robotConfig, const Vector& measurement,  const SharedNoiseModel& model, bool hasDeadBand_, double deadBand_) :
                        Base(model, robotConfig), encoders(measurement), deadBand(deadBand_), hasDeadBand(hasDeadBand_)
            {
            }

            virtual ~EncoderFactor()
            {

            }

            double deadBandCost(const double& diff) const
            {
                if (fabs(diff) < deadBand)
                {
                    return 0.0;
                }
                else
                {
                    if (diff < 0)
                    {
                        return diff + deadBand;
                    }
                    else
                    {
                        return diff - deadBand;
                    }
                }
            }


            Vector err(const RobotConfig& robot, const gtsam::Vector& q) const
            {
                gtsam::Vector diff = q - encoders;
                for (size_t i = 0; i < robot.dim(); i++)
                {
                    // Free joints do not have encoders.
                    if (robot.IsFree(i) || robot.IsPlanar(i))
                    {
                        diff(i) = 0;
                    }
                    // Non-continuous joints can just be treated as real numbers
                    else if (!robot.IsContinuous(i))
                    {
                        diff(i) = q(i) - encoders(i);
                    }
                    // Other encoders must be mapped to SO(2).
                    else
                    {
                        diff(i) = utils::AngleDiff(q(i), encoders(i));
                    }
                    if (hasDeadBand)
                    {
                        diff(i) = deadBandCost(diff(i));
                    }
                }
                return diff;
            }

            Vector evaluateError(const RobotConfig& q, boost::optional<Matrix&> J = boost::none) const
            {
                if (J)
                {
                    // Jacobian is the identity, because
                    // encoders are directly related to the robot's
                    // configuration. TODO: Verify?
                    (*J) = eye(encoders.rows());

                    for (size_t i = 0; i < q.dim(); i++)
                    {
                        // Free joints do not have encoders.
                        if (q.IsFree(i) || q.IsPlanar(i))
                        {
                           J->col(i) *= 0;
                        }
                    }
                }

#ifdef SANITY_CHECK
                size_t n = q.dim();
                double dQ = 0.001;
                gtsam::Vector delta = q.getQ();
                gtsam::Matrix jTest = zeros(n, n);

                for (size_t k = 0; k < n; k++)
                {
                    delta = q.getQ();
                    delta(k) += dQ;
                    Vector ePlus = err(q, delta);
                    delta(k) -= 2 * dQ;
                    Vector eMinus = err(q, delta);
                    Vector centralDifference =  (ePlus - eMinus) / (2 * dQ);
                    jTest.col(k) = centralDifference;
                }

                std::cout << "Jacobian: " <<std::endl << jTest << std::endl;
#endif

                return err(q, q.getQ());
            }

            virtual gtsam::NonlinearFactor::shared_ptr clone() const {
              return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                  gtsam::NonlinearFactor::shared_ptr(new EncoderFactor(*this))); }

    };


} /* namespace gtsam */

#endif /* ENCODERFACTOR_H_ */
