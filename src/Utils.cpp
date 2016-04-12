
#include <arm_slam_calib/Utils.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <math.h>
#include <ros/ros.h>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>


using namespace gtsam;
namespace utils
{

    double Rand()
    {
        return (double)rand() / (double)RAND_MAX;
    }

    double Rand(double magnitude)
    {
        return Rand() * magnitude - magnitude * 0.5;
    }

    double Rand(double min, double max)
    {
        return Rand() * (max - min) + min;
    }

    double WrapCircle(double x)
    {
        return atan2(sin(x), cos(x));
    }

    double AngleDiff(const double& a, const double& b)
    {
        return (atan2(sin(a - b), cos(a - b)));
    }

    gtsam::Point3 TriangulateRealDumb(const gtsam::Point2& keypoint1,
            const gtsam::Point2& keypoint2,
            const gtsam::Pose3& poseA,
            const gtsam::Pose3& poseB, bool& valid, const Cal3_S2& calib)
    {
        PinholeCamera<Cal3_S2> cameraA(poseA, calib);
        PinholeCamera<Cal3_S2> cameraB(poseB, calib);

        gtsam::Point2 p1C = calib.calibrate(keypoint1);
        gtsam::Point2 p2C = calib.calibrate(keypoint2);

        float minError = std::numeric_limits<float>::max();
        gtsam::Point3 minGuessWorld;
        valid = false;
        for (float depth1 = 0.01; depth1 < 3; depth1 += 0.01)
        for (float depth2 = 0.01; depth2 < 3; depth2 += 0.01)
        {
            try
            {
                gtsam::Point3 guess_camera1 = gtsam::Point3(p1C.x() * depth1, p1C.y() * depth1, depth1);
                gtsam::Point3 guess_camera2 = gtsam::Point3(p2C.x() * depth2, p2C.y() * depth2, depth2);
                gtsam::Point3 guess_world1 = poseA.transform_from(guess_camera1);
                gtsam::Point3 guess_world2 = poseB.transform_from(guess_camera2);
                gtsam::Point3 guessWorld = (guess_world2 + guess_world1) * 0.5f;
                float err = (cameraA.project(guessWorld) - keypoint1).norm() + (cameraB.project(guessWorld) - keypoint2).norm();

                if (err < minError)
                {
                    minGuessWorld = (guess_world2 + guess_world1) * 0.5f;
                    minError = err;
                }
            }
            catch (gtsam::CheiralityException& e)
            {

            }
        }

        valid = minError < 20.0f;

        if (!valid)
        {
            ROS_WARN("Could only reduce reprojection error to %f pixels", minError);
        }
        return poseA.transform_to(minGuessWorld);
    }

    // Gets the position of the landmark expressed in poseA, from two keypoints.
    gtsam::Point3 TriangulateDumb(const gtsam::Point2& keypoint1,
                                const gtsam::Point2& keypoint2,
                                const gtsam::Pose3& poseA,
                                const gtsam::Pose3& poseB, bool& valid, const Cal3_S2& calib)
    {
        PinholeCamera<Cal3_S2> cameraA(poseA, calib);
        PinholeCamera<Cal3_S2> cameraB(poseB, calib);

        gtsam::Point2 p1C = calib.calibrate(keypoint1);
        gtsam::Point2 p2C = calib.calibrate(keypoint2);

        float minError = std::numeric_limits<float>::max();
        float minDepth = -1;
        gtsam::Point3 minGuess;
        valid = false;
        for (float depth = 0.01; depth < 3; depth += 0.01)
        {
            gtsam::Point3 guess_camera = gtsam::Point3(p1C.x() * depth, p1C.y() * depth, depth);
            gtsam::Point3 guess_world = poseA.transform_from(guess_camera);
            try
            {
                std::pair<gtsam::Point2, bool> projection = cameraB.projectSafe(guess_world);

                if (projection.second)
                {
                    float err = (projection.first - keypoint2).norm();

                    if (err < minError)
                    {
                        minError = err;
                        minDepth = depth;
                        minGuess = guess_camera;
                    }

                    if (err < 20.0)
                    {
                        valid = true;
                    }
                }
            }
            catch(gtsam::CheiralityException& e)
            {

            }
        }

        ROS_INFO("Best depth is %f with an error of %f", minDepth, minError);
        if (!valid)
        {
            ROS_WARN("Could only reduce reprojection error to %f pixels", minError);
        }
        return minGuess;
    }


    // Gets the position of the landmark expressed in poseA, from two keypoints.
    gtsam::Point3 Triangulate(const gtsam::Point2& keypoint1,
                                const gtsam::Point2& keypoint2,
                                const gtsam::Pose3& poseA,
                                const gtsam::Pose3& poseB, bool& valid, const Cal3_S2& calib)
    {
        PinholeCamera<Cal3_S2> cameraA(poseA, calib);
        PinholeCamera<Cal3_S2> cameraB(poseB, calib);
        gtsam::Pose3 T_AB = poseB.transform_to(poseA);
        gtsam::Point2 p1C = calib.calibrate(keypoint1);
        gtsam::Point2 p2C = calib.calibrate(keypoint2);
        gtsam::Point3 p1A = gtsam::Point3(p1C.x(), p1C.y(), 1.0);
        gtsam::Point3 p2B = gtsam::Point3(p2C.x(), p2C.y(), 1.0);
        gtsam::Point3 p2A = T_AB.rotation() * p2B;

        bool isParallel = false;
        Eigen::Vector4d hPA =
                Triangulate(Eigen::Vector3d::Zero(),
                p1A.vector().normalized(),
                T_AB.translation().vector(),
                p2A.vector().normalized(),
                2.0,
                valid,
                isParallel);

        std::cout << "Valid? " << valid << std::endl;
        std::cout << "Parallel? " << isParallel << std::endl;
        std::cout << "Kp1 " << keypoint1 << std::endl;
        std::cout << "Kp2 " << keypoint2 << std::endl;
        std::cout << "TA " << poseA << std::endl;
        std::cout << "TB " << poseB << std::endl;
        std::cout << "TAB" << T_AB << std::endl;
        std::cout << "P1A " << p1A << std::endl;
        std::cout << "P2A " << p2A << std::endl;
        std::cout << "hPA " << hPA.transpose() << std::endl;
        //exit(-1);
        gtsam::Point3 result =  gtsam::Point3(hPA.x() / hPA.w(), hPA.y() / hPA.w(), hPA.z() / hPA.w());
        gtsam::Point3 resultWorld = poseA.transform_from(result);

        try
        {
            gtsam::Point2 uvResult1 = cameraA.project(resultWorld);
            gtsam::Point2 uvResult2 = cameraB.project(resultWorld);

            float error1 = (keypoint1 - uvResult1).norm();
            float error2 = (keypoint2 - uvResult2).norm();

            std::cout << "Err 1: " << error1 << std::endl;
            std::cout << "Err 2: " << error2 << std::endl;
            if (error1 > 2.0 || error2 > 2.0)
            {
                valid = false;
            }
        }
        catch(gtsam::CheiralityException& e)
        {
            ROS_ERROR("Cheirality test failed.");
            valid = false;
        }
        return result;
    }

    // Triangulate the intersection of two rays.
    Eigen::Vector4d Triangulate(const Eigen::Vector3d& p1, // origin of frame A expressed in A
            const Eigen::Vector3d& e1, // ray in frame A expressed in A
            const Eigen::Vector3d& p2, // origin of frame B expressed in A
            const Eigen::Vector3d& e2, // ray in frame B expressed in A
            double sigma,
            bool& isValid, bool& isParallel)
    {
        isParallel = false; // This should be the default.
        // But parallel and invalid is not the same. Points at infinity are valid and parallel.
        isValid = false; // hopefully this will be reset to true.

        // stolen and adapted from the Kneip toolchain geometric_vision/include/geometric_vision/triangulation/impl/triangulation.hpp
        Eigen::Vector3d t12 = p2 - p1;

        Eigen::Vector2d b;
        b[0] = t12.dot(e1);
        b[1] = t12.dot(e2);
        Eigen::Matrix2d A;
        A(0, 0) = e1.dot(e1);
        A(1, 0) = e1.dot(e2);
        A(0, 1) = -A(1, 0);
        A(1, 1) = -e2.dot(e2);

        if (A(1, 0) < 0.0)
        {
            A(1, 0) = -A(1, 0);
            A(0, 1) = -A(0, 1);
            // wrong viewing direction
        };

        bool invertible;
        Eigen::Matrix2d A_inverse;
        A.computeInverseWithCheck(A_inverse, invertible, 1.0e-6);
        Eigen::Vector2d lambda = A_inverse * b;
        if (!invertible)
        {
            isParallel = true; // let's note this.
            // parallel. that's fine. but A is not invertible. so handle it separately.
            if ((e1.cross(e2)).norm() < 6 * sigma)
            {
                isValid = true;  // check parallel
            }
            return (Eigen::Vector4d((e1[0] + e2[0]) / 2.0, (e1[1] + e2[1]) / 2.0,
                    (e1[2] + e2[2]) / 2.0, 1e-3).normalized());
        }

        Eigen::Vector3d xm = lambda[0] * e1 + p1;
        Eigen::Vector3d xn = lambda[1] * e2 + p2;
        Eigen::Vector3d midpoint = (xm + xn) / 2.0;


        // check it
        Eigen::Vector3d error = midpoint - xm;
        Eigen::Vector3d diff = midpoint - (p1 + 0.5 * t12);
        const double diff_sq = diff.dot(diff);
        const double chi2 = error.dot(error) * (1.0 / (diff_sq * sigma * sigma));

        isValid = true;
        if (chi2 > 9)
        {
            isValid = false;  // reject large chi2-errors
        }


        // flip if necessary
        if (diff.dot(e1) < 0)
        {
            midpoint = (p1 + 0.5 * t12) - diff;
        }

        return Eigen::Vector4d(midpoint[0], midpoint[1], midpoint[2], 1.0).normalized();
    }
}
