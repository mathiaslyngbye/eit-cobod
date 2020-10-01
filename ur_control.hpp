#ifndef UR_CONTROL_HPP
#define UR_CONTROL_HPP

// Include robwork
#include <rw/rw.hpp>
#include <rwhw/universalrobots/UniversalRobotsData.hpp>
#include <rwhw/universalrobots_rtde/URRTDE.hpp>

// Include standards
#include <thread>
#include <iostream>
#include <chrono>

// Include boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>

// Extra defines for robot gripper
#define OPEN true
#define CLOSE false

// Variables
extern std::atomic_bool stopped;

// Functions
void RunControl(rwhw::URRTDE *robot);
void RunPrintJoints(rwhw::URRTDE *robot);
void RunStop(rwhw::URRTDE *robot);


#endif // UR_CONTROL_HPP
