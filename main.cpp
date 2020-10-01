//#include <ur_rtde/rtde_control_interface.h>
//#include <ur_rtde/rtde_io_interface.h>
//#include <ur_rtde/rtde_receive_interface.h>
#include <rw/rw.hpp>
#include <rwhw/universalrobots/UniversalRobotsData.hpp>
#include <rwhw/universalrobots_rtde/URRTDE.hpp>

#include <thread>
#include <iostream>
#include <chrono>

// Defines for robot gripper
#define OPEN true
#define CLOSE false

void control(std::string robot_ip)
{
    rwhw::URRTDE robot(robot_ip);

    const rw::math::Transform3D<> pose_up = rw::math::Transform3D<>(
            rw::math::Vector3D<>(-0.2, -0.5, 0.5),
            rw::math::RPY<>(0, 0,/*3.12*/ 0.04)
            );

    const rw::math::Transform3D<> pose_down = rw::math::Transform3D<>(
            rw::math::Vector3D<>(-0.2, -0.5, 0.2),
            rw::math::RPY<>(0, 3.12, 0.04)
            );

    rw::trajectory::Transform3DPath lpath;

    robot.setStandardDigitalOut(0,OPEN);
    lpath.push_back(pose_up);
    lpath.push_back(pose_down);
    robot.moveL(lpath);
    robot.setStandardDigitalOut(0,CLOSE);
    lpath.clear();
    lpath.push_back(pose_up);
    robot.moveL(lpath);
    robot.setStandardDigitalOut(0,OPEN);

    robot.stopRobot();

    rw::math::Q joint_positions = robot.getActualQ();
}

int main(int argc, char* argv[])
{
    // Test print function
    const rw::math::Q test_q = rw::math::Q(6, 0.999, -1.911, -1.757, -2.615, -2.137, 0);
    std::cout << test_q << std::endl;

    // Move robot
    const static std::string robot_ip = "192.168.0.212";
    control(robot_ip);

    return 0;
}
