#include "ur_control.hpp"

std::atomic_bool stopped(false);

void RunControl(rwhw::URRTDE *robot)
{
    const rw::math::Transform3D<> pose_up = rw::math::Transform3D<>(
            rw::math::Vector3D<>(-0.2, -0.5, 0.5),
            rw::math::RPY<>(0, 3.12, 0)
            );

    const rw::math::Transform3D<> pose_down = rw::math::Transform3D<>(
            rw::math::Vector3D<>(-0.2, -0.5, 0.2),
            rw::math::RPY<>(0, 3.12, 0)
            );

    rw::trajectory::Transform3DPath lpath;

    while(!stopped)
    {
        // Open and move down
        lpath.clear();
        robot->setStandardDigitalOut(0,OPEN);
        lpath.push_back(pose_down);
        robot->moveL(lpath);

        // Close and move up
        robot->setStandardDigitalOut(0,CLOSE);
        lpath.clear();
        lpath.push_back(pose_up);
        robot->moveL(lpath);
    }

    robot->stopRobot();
}

void RunPrintJoints(rwhw::URRTDE *robot)
{
    while(true)
    {
        std::cout << robot->getActualQ() << std::endl;
        std::cout << "\x1b[A";
    }
}

void RunStop(rwhw::URRTDE *robot)
{
    while(true)
    {
        char in;
        std::cin >> in;
        stopped = true;
    }
}
