/*
 * RelativePoseAdapter.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: mklingen
 */

#include <arm_slam_calib/RelativePoseAdapter.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

using namespace gtsam;

namespace opengv
{
    namespace relative_pose
    {
        RelativePoseAdapter::RelativePoseAdapter(
                const Pose3& poseA, const Pose3& poseB,
                const Cal3_S2& cal,
                const std::vector<Landmark>& landmarks,
                const std::vector<Point2>& keypointsA,
                const std::vector<Point2>& keypointsB)
        {
            PinholeCamera<Cal3_S2> camA(Pose3::identity(), cal);
            PinholeCamera<Cal3_S2> camB(Pose3::identity(), cal);

            for (size_t i = 0; i < keypointsA.size(); i++)
            {
                Point3 bearing = camA.backproject(keypointsA.at(i), 1.0f);
                bearing.normalize();
                _bearingVectors1.push_back(bearingVector_t(bearing.x(), bearing.y(), bearing.z()));
            }

            for (size_t i = 0; i < keypointsB.size(); i++)
            {
                Point3 bearing = camB.backproject(keypointsB.at(i), 1.0f);
                bearing.normalize();
                _bearingVectors2.push_back(bearingVector_t(bearing.x(), bearing.y(), bearing.z()));
            }

            rotation_t rotation1 = poseA.rotation().matrix();
            rotation_t rotation2 = poseB.rotation().matrix();
            translation_t position1 = poseA.translation().vector();
            translation_t position2 = poseB.translation().vector();

            _R12 = rotation1.transpose() * rotation2;
            _t12 = rotation1.transpose() * (position2 - position1);
        }

        /**
         * \brief Retrieve the bearing vector of a correspondence in viewpoint 1.
         * \param[in] index The serialized index of the correspondence.
         * \return The corresponding bearing vector.
         */
        opengv::bearingVector_t RelativePoseAdapter::getBearingVector1( size_t index) const
        {
            return _bearingVectors1[index];
        }

        /**
         * \brief Retrieve the bearing vector of a correspondence in viewpoint 2.
         * \param[in] index The serialized index of the correspondence.
         * \return The corresponding bearing vector.
         */
        opengv::bearingVector_t RelativePoseAdapter::getBearingVector2( size_t index ) const
        {
            return _bearingVectors2[index];
        }

        /**
         * \brief Retrieve the weight of a correspondence. The weight is supposed to
         *        reflect the quality of a correspondence, and typically is between
         *        0 and 1.
         * \param[in] index The serialized index of the correspondence.
         * \return The corresponding weight.
         */
        double RelativePoseAdapter::getWeight( size_t index ) const
        {
            return 1.0;
        }

        /**
         * \brief Retrieve the position of a camera of a correspondence in viewpoint
         *        1 seen from the origin of the viewpoint.
         * \param[in] index The serialized index of the correspondence.
         * \return The position of the corresponding camera seen from the viewpoint
         *         origin.
         */
        opengv::translation_t RelativePoseAdapter::getCamOffset1( size_t index ) const
        {
            return opengv::translation_t(0, 0, 0);
        }

        /**
         * \brief Retrieve the rotation from a camera of a correspondence in
         *        viewpoint 1 to the viewpoint origin.
         * \param[in] index The serialized index of the correspondence.
         * \return The rotation from the corresponding camera back to the viewpoint
         *         origin.
         */
        opengv::rotation_t RelativePoseAdapter::getCamRotation1( size_t index ) const
        {
            return opengv::rotation_t::Identity();
        }

        /**
         * \brief Retrieve the position of a camera of a correspondence in viewpoint
         *        2 seen from the origin of the viewpoint.
         * \param[in] index The serialized index of the correspondence.
         * \return The position of the corresponding camera seen from the viewpoint
         *         origin.
         */
        opengv::translation_t RelativePoseAdapter::getCamOffset2( size_t index ) const
        {
            return opengv::translation_t(0, 0, 0);
        }

        /**
         * \brief Retrieve the rotation from a camera of a correspondence in
         *        viewpoint 2 to the viewpoint origin.
         * \param[in] index The serialized index of the correspondence.
         * \return The rotation from the corresponding camera back to the viewpoint
         *         origin.
         */
        opengv::rotation_t RelativePoseAdapter::getCamRotation2( size_t index ) const
        {
            return opengv::rotation_t::Identity();
        }

        /**
         * \brief Retrieve the number of correspondences.
         * \return The number of correspondences.
         */
        size_t RelativePoseAdapter::getNumberCorrespondences() const
        {
            return _bearingVectors1.size();
        }

        RelativePoseAdapter::~RelativePoseAdapter()
        {
            // TODO Auto-generated destructor stub
        }
    }

} /* namespace  */
