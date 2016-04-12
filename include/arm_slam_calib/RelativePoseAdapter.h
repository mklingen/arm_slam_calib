/*
 * RelativePoseAdapter.h
 *
 *  Created on: Feb 16, 2016
 *      Author: mklingen
 */

#ifndef RELATIVEPOSEADAPTER_H_
#define RELATIVEPOSEADAPTER_H_

#include <arm_slam_calib/Landmark.h>
#include <opengv/relative_pose/RelativeAdapterBase.hpp>

namespace opengv
{
    namespace relative_pose
    {

        class RelativePoseAdapter : public opengv::relative_pose::RelativeAdapterBase
        {
            public:
                RelativePoseAdapter(const gtsam::Pose3& poseA, const gtsam::Pose3& poseB,
                        const gtsam::Cal3_S2& cal,
                        const std::vector<gtsam::Landmark>& landmarks,
                        const std::vector<gtsam::Point2>& keypointsA,
                        const std::vector<gtsam::Point2>& keypointsB);
                virtual ~RelativePoseAdapter();

                /**
                 * \brief Retrieve the bearing vector of a correspondence in viewpoint 1.
                 * \param[in] index The serialized index of the correspondence.
                 * \return The corresponding bearing vector.
                 */
                virtual opengv::bearingVector_t getBearingVector1( size_t index) const;
                /**
                 * \brief Retrieve the bearing vector of a correspondence in viewpoint 2.
                 * \param[in] index The serialized index of the correspondence.
                 * \return The corresponding bearing vector.
                 */
                virtual opengv::bearingVector_t getBearingVector2( size_t index ) const;

                /**
                 * \brief Retrieve the weight of a correspondence. The weight is supposed to
                 *        reflect the quality of a correspondence, and typically is between
                 *        0 and 1.
                 * \param[in] index The serialized index of the correspondence.
                 * \return The corresponding weight.
                 */
                virtual double getWeight( size_t index ) const;
                /**
                 * \brief Retrieve the position of a camera of a correspondence in viewpoint
                 *        1 seen from the origin of the viewpoint.
                 * \param[in] index The serialized index of the correspondence.
                 * \return The position of the corresponding camera seen from the viewpoint
                 *         origin.
                 */
                virtual opengv::translation_t getCamOffset1( size_t index ) const;
                /**
                 * \brief Retrieve the rotation from a camera of a correspondence in
                 *        viewpoint 1 to the viewpoint origin.
                 * \param[in] index The serialized index of the correspondence.
                 * \return The rotation from the corresponding camera back to the viewpoint
                 *         origin.
                 */
                virtual opengv::rotation_t getCamRotation1( size_t index ) const;
                /**
                 * \brief Retrieve the position of a camera of a correspondence in viewpoint
                 *        2 seen from the origin of the viewpoint.
                 * \param[in] index The serialized index of the correspondence.
                 * \return The position of the corresponding camera seen from the viewpoint
                 *         origin.
                 */
                virtual opengv::translation_t getCamOffset2( size_t index ) const;
                /**
                 * \brief Retrieve the rotation from a camera of a correspondence in
                 *        viewpoint 2 to the viewpoint origin.
                 * \param[in] index The serialized index of the correspondence.
                 * \return The rotation from the corresponding camera back to the viewpoint
                 *         origin.
                 */
                virtual opengv::rotation_t getCamRotation2( size_t index ) const;
                /**
                 * \brief Retrieve the number of correspondences.
                 * \return The number of correspondences.
                 */
                virtual size_t getNumberCorrespondences() const;

                opengv::bearingVectors_t _bearingVectors1;
                opengv::bearingVectors_t _bearingVectors2;
        };
    }

} /* namespace gtsam */

#endif /* RELATIVEPOSEADAPTER_H_ */
