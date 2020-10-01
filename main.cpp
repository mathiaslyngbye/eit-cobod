// Include files
#include "ur_control.hpp"

int main(int argc, char* argv[])
{
    // Move robot
    const static std::string robot_ip = "192.168.0.212";
    rwhw::URRTDE robot(robot_ip);

    boost::thread controlth(RunControl, &robot);
    boost::thread printth(RunPrintJoints, &robot);
    boost::thread stopth(RunStop, &robot);

    controlth.join();
    printth.join();
    stopth.join();

    return 0;
}
