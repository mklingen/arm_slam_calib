/*
 * Timer.h
 *
 *  Created on: Mar 7, 2016
 *      Author: mklingen
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <ros/time.h>
#include <map>

namespace timer
{
    class Timer
    {
        public:
            Timer();
            Timer(const std::string& name);
            virtual ~Timer();

            void Tick();
            void Tock();

            static void Tick(const std::string& timer);
            static void Tock(const std::string& timer);

            static void PrintStats();

        protected:
            static std::map<std::string, Timer> timers;
            std::vector<ros::Duration> times;
            ros::Time lastTick;
            ros::Time lastTock;
            std::string name;
    };
}


#endif /* TIMER_H_ */
