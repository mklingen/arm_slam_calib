/*
 * EncoderFactor.h
 *
 *  Created on: Jan 22, 2016
 *      Author: mklingen
 */

#ifndef DRIFT_FACTOR_H_
#define DRIFT_FACTOR_H_

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <arm_slam_calib/RobotProjectionFactor.h>
#include <arm_slam_calib/RobotConfig.h>

namespace gtsam
{

    class DriftFactor : public NoiseModelFactor2<RobotConfig, RobotConfig>
    {
        protected:
            Vector q0Encoders;
            Vector q1Encoders;

        public:
            typedef NoiseModelFactor2<RobotConfig, RobotConfig> Base;
            typedef boost::shared_ptr<DriftFactor> shared_ptr;

            DriftFactor(Key q0, Key q1, const Vector& q0Encoders_, const Vector& q1Encoders_,  const SharedNoiseModel& model) :
                        Base(model, q0, q1), q0Encoders(q0Encoders_), q1Encoders(q1Encoders_)
            {

            }

            virtual ~DriftFactor()
            {

            }

            double angleDiff(const double& a, const double& b) const
            {
                return (atan2(sin(a - b), cos(a - b)));
            }

            Vector evaluateError(const RobotConfig& q0, const RobotConfig& q1, boost::optional<Matrix&> J1 = boost::none, boost::optional<Matrix&> J2 = boost::none) const
            {
                if (J1 || J2)
                {
                    // Jacobian is the identity, because
                    // encoders are directly related to the robot's
                    // configuration. TODO: Verify?
                    (*J1) = -eye(q0Encoders.rows());
                    (*J2) = eye(q1Encoders.rows());
                }

                gtsam::Vector diff = q0.getQ();
                for (size_t i = 0; i < q0.dim(); i++)
                {
                    diff(i) = (q1.getQ()(i) - q0.getQ()(i)) - (q1Encoders(i) - q0Encoders(i));
                }
                return diff;
            }

            virtual gtsam::NonlinearFactor::shared_ptr clone() const {
              return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                  gtsam::NonlinearFactor::shared_ptr(new DriftFactor(*this))); }

    };


} /* namespace gtsam */

#endif /* DRIFT_FACTOR_H_ */
