/*
 * EncoderFactor.h
 *
 *  Created on: Jan 22, 2016
 *      Author: mklingen
 */

#ifndef VELOCITY_FACTOR_H_
#define VELOCITY_FACTOR_H_

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <arm_slam_calib/RobotProjectionFactor.h>
#include <arm_slam_calib/RobotConfig.h>

#define JACOBIAN_CHECK

namespace gtsam
{

    class VelocityFactor : public NoiseModelFactor2<RobotConfig, RobotConfig>
    {
        protected:
            Vector qDot;

        public:
            typedef NoiseModelFactor2<RobotConfig, RobotConfig> Base;
            typedef boost::shared_ptr<VelocityFactor> shared_ptr;

            VelocityFactor(Key q0, Key q1, const Vector& qDot_, const SharedNoiseModel& model) :
                        Base(model, q0, q1), qDot(qDot_)
            {

            }

            virtual ~VelocityFactor()
            {

            }

            Vector err          (const RobotConfig& q0,
                                 const RobotConfig& q1,
                                 boost::optional<Matrix&> J1 = boost::none,
                                 boost::optional<Matrix&> J2 = boost::none) const
            {
                if (J1 || J2)
                {
                    // Jacobian is the identity, because
                    // encoders are directly related to the robot's
                    // configuration. TODO: Verify?
                    (*J1) = -eye(q0.dim());
                    (*J2) = eye(q1.dim());
                }

                // TODO: THIS IS NOW WRONG BECAUSE FIRST 6 DOF ARE SE(3)!
                // YOU CAN'T SIMPLY SUBTRACT THEM :( :(
                gtsam::Vector diff = q0.localCoordinates(q1) - qDot;
                /*
                for (size_t i = 0; i < q0.dim(); i++)
                {
                    diff(i) = (q1.getQ()(i) - q0.getQ()(i)) - (qDot(i));
                }
                */
                return diff;
            }

            Vector evaluateError(const RobotConfig& q0,
                                 const RobotConfig& q1,
                                 boost::optional<Matrix&> J1 = boost::none,
                                 boost::optional<Matrix&> J2 = boost::none) const
            {
#ifdef JACOBIAN_CHECK
                double dQ = 0.001;

                *J1 = zeros(q0.dim(), q0.dim());
                *J2 = zeros(q1.dim(), q1.dim());
                for (size_t i = 0; i < q0.dim(); i++)
                {
                    Vector q0Diff = q0.getQ() * 0;
                    q0Diff(i) = dQ;

                    Vector errPlus0 = err(q0.retract(q0Diff), q1);
                    Vector errMinus0 = err(q0.retract(-q0Diff), q1);
                    Vector errPlus1 = err(q0, q1.retract(q0Diff));
                    Vector errMinus1 = err(q0, q1.retract(-q0Diff));
                    J1->col(i) = (errPlus0 - errMinus0) / (2 * dQ);
                    J2->col(i) = (errPlus1 - errMinus1) / (2 * dQ);
                }

                //std::cout << J1->matrix() << std::endl;
                //std::cout << J2->matrix() << std::endl;
                //exit(-1);
#endif
                return err(q0, q1);
            }

            virtual gtsam::NonlinearFactor::shared_ptr clone() const {
              return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                  gtsam::NonlinearFactor::shared_ptr(new VelocityFactor(*this))); }

    };


} /* namespace gtsam */

#endif /* VELOCITY_FACTOR_H_ */
