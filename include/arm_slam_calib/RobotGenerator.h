/*
 * RobotGenerator.h
 *
 *  Created on: Apr 19, 2016
 *      Author: mklingen
 */

#ifndef INCLUDE_ARM_SLAM_CALIB_ROBOTGENERATOR_H_
#define INCLUDE_ARM_SLAM_CALIB_ROBOTGENERATOR_H_

#include <dart/dart.h>

namespace dart
{
    class RobotGenerator
    {
        public:
            RobotGenerator();
            virtual ~RobotGenerator();
            static dart::dynamics::SkeletonPtr GenerateRobot(size_t numDof, float scale = 1.0f, float growth = 0.6f, float randomLength = 0.01f, float randomAngle = 0.01f);
    };

} /* namespace dart */

#endif /* INCLUDE_ARM_SLAM_CALIB_ROBOTGENERATOR_H_ */
