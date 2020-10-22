// RobWork includes
#include <rw/rw.hpp>

// RobWorkStudio includes
#include <RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

// RobWorkHardware includes
#include <rwhw/universalrobots/UniversalRobotsData.hpp>
#include <rwhw/universalrobots_rtde/URRTDE.hpp>
#include <string>
#include <thread>

int main(int argc, char* argv[])
{
    // Move robot
    //const static std::string robot_ip = "192.168.0.212";
    //rwhw::URRTDE robot(robot_ip);

    //std::thread controlth(RunControl, &robot);
    //std::thread printth(RunPrintJoints, &robot);
    //std::thread stopth(RunStop, &robot);

    //controlth.join();
    //printth.join();
    //stopth.join();

    return 0;
}
