/*
 * Utils.h
 *
 *  Created on: Apr 1, 2016
 *      Author: mklingen
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <Eigen/Dense>

namespace utils
{
    double Rand();
    double Rand(double magnitude);
    double Rand(double min, double max);
    double WrapCircle(double x);
    double AngleDiff(const double& a, const double& b);

    gtsam::Point3 TriangulateRealDumb(const gtsam::Point2& keypoint1,
            const gtsam::Point2& keypoint2,
            const gtsam::Pose3& poseA,
            const gtsam::Pose3& poseB, bool& valid, const gtsam::Cal3_S2& calib);

    // Gets the position of the landmark expressed in poseA, from two keypoints.
    gtsam::Point3 TriangulateDumb(const gtsam::Point2& keypoint1,
                                const gtsam::Point2& keypoint2,
                                const gtsam::Pose3& poseA,
                                const gtsam::Pose3& poseB, bool& valid, const gtsam::Cal3_S2& calib);


    // Gets the position of the landmark expressed in poseA, from two keypoints.
    gtsam::Point3 Triangulate(const gtsam::Point2& keypoint1,
                                const gtsam::Point2& keypoint2,
                                const gtsam::Pose3& poseA,
                                const gtsam::Pose3& poseB, bool& valid, const gtsam::Cal3_S2& calib);

    // Triangulate the intersection of two rays.
    Eigen::Vector4d Triangulate(const Eigen::Vector3d& p1, // origin of frame A expressed in A
            const Eigen::Vector3d& e1, // ray in frame A expressed in A
            const Eigen::Vector3d& p2, // origin of frame B expressed in A
            const Eigen::Vector3d& e2, // ray in frame B expressed in A
            double sigma,
            bool& isValid, bool& isParallel);
}

#endif /* UTILS_H_ */
