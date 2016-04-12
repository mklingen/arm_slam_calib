/*
 * Landmark.h
 *
 *  Created on: Feb 4, 2016
 *      Author: mklingen
 */

#ifndef LANDMARK_H_
#define LANDMARK_H_

#include <vector>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>

#include <opencv/cv.hpp>

namespace gtsam
{
    class Landmark
    {
        public:
            Landmark() : id(0), isTriangulated(false), isNew(true), isInGraph(false)
            {

            }

            Landmark& operator=(Landmark arg)
            {
                id = arg.id;
                configs = arg.configs;
                observations = arg.observations;
                position = arg.position;
                color = arg.color;
                arg.descriptor.copyTo(descriptor);
                cameraPoses = arg.cameraPoses;
                return *this;
            }

            size_t id;
            std::vector<size_t> configs;
            std::vector<gtsam::Point2> observations;
            gtsam::Point3 position;
            gtsam::Point3 color;
            cv::Mat descriptor;
            std::vector<gtsam::Pose3> cameraPoses;
            bool isTriangulated;
            bool isNew;
            bool isInGraph;
    };
}


#endif /* LANDMARK_H_ */
