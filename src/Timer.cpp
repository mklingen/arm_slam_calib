/*
 * Timer.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: mklingen
 */

#include <arm_slam_calib/Timer.h>


namespace timer
{
    std::map<std::string, Timer> Timer::timers = std::map<std::string, Timer>();
    Timer::Timer()
    {

    }

    Timer::Timer(const std::string& name_) :
            name(name_)
    {

    }

    Timer::~Timer()
    {

    }

    void Timer::Tick()
    {
        lastTick = ros::Time::now();
    }

    void Timer::Tock()
    {
        lastTock = ros::Time::now();
        times.push_back(lastTock - lastTick);
    }

    void Timer::Tick(const std::string& timer)
    {
        if (timers.find(timer) == timers.end())
        {
            timers[timer] = Timer(timer);
        }
        timers[timer].Tick();
    }

    void Timer::Tock(const std::string& timer)
    {
        if (timers.find(timer) == timers.end())
        {
            timers[timer] = Timer(timer);
        }
        timers[timer].Tock();
    }

   void Timer::PrintStats()
   {
       std::cout << "###########Timing#########" << std::endl;
       for (auto timerIt : timers)
       {
           const Timer& timer = timerIt.second;

           if (timer.times.size() == 0) continue;

           double minTime = std::numeric_limits<double>::max();
           double maxTime = -std::numeric_limits<double>::max();
           double avgTime = 0;
           size_t numNonZero = 0;
           for (size_t i = 0; i < timer.times.size(); i++)
           {
               const ros::Duration& dur = timer.times.at(i);
               if (dur.toSec() > 1e-12)
               {
                   minTime = fmin(dur.toSec(), minTime);
                   maxTime = fmax(dur.toSec(), maxTime);
                   avgTime += dur.toSec();
                   numNonZero++;
               }
          }
           if (numNonZero == 0) continue;

           std::cout << timer.name << "\t";
           avgTime /= numNonZero;
           std::cout << "Min: " << minTime * 1000 << " Max: " << maxTime * 1000 << " Avg: " << avgTime * 1000 << std::endl;
       }
   }
}


