#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <iostream>
#include <chrono>

void demo1(std::string robot_ip)
{
    ur_rtde::RTDEControlInterface rtde_control(robot_ip);
    double velocity = 0.2;
    double acceleration = 0.5;
    double blend_1 = 0.0;
    double blend_2 = 0.02;
    double blend_3 = 0.0;
    std::vector<double> path_pose1 = {-0.2, -0.5, 0.2, -0.001, 3.12, 0.04, velocity, acceleration, blend_1};
    std::vector<double> path_pose2 = {-0.2, -0.5, 0.5, -0.001, 3.12, 0.04, velocity, acceleration, blend_2};
    std::vector<double> path_pose3 = {-0.2, -0.5, 0.2, -0.001, 3.12, 0.04, velocity, acceleration, blend_3};

    std::vector<std::vector<double>> path;
    path.push_back(path_pose1);
    path.push_back(path_pose2);
    path.push_back(path_pose3);

    // Send a linear path with blending in between - (currently uses separate script)
    rtde_control.moveL(path);
    rtde_control.stopScript();
}


void demo2(std::string robot_ip)
{
  ur_rtde::RTDEControlInterface rtde_control(robot_ip);

  // Parameters
  std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
  std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
  std::vector<double> wrench_down = {0, 0, -10, 0, 0, 0};
  std::vector<double> wrench_up = {0, 0, 10, 0, 0, 0};
  int force_type = 2;
  double dt = 1.0/500; // 2ms
  std::vector<double> limits = {2, 2, 1.5, 1, 1, 1};
  std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

  // Move to initial joint position with a regular moveJ
  rtde_control.moveJ(joint_q);

  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<2000; i++)
  {
    auto t_start = std::chrono::high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 1000)
        rtde_control.forceModeStart(task_frame, selection_vector, wrench_up, force_type, limits);
    else
        rtde_control.forceModeStart(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop = std::chrono::high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }

  rtde_control.forceModeStop();
  rtde_control.stopScript();

}

void demo3(std::string robot_ip)
{
    ur_rtde::RTDEIOInterface rtde_io(robot_ip);
    ur_rtde::RTDEReceiveInterface rtde_receive(robot_ip);

    /** How-to set and get standard and tool digital outputs. Notice that we need the
      * RTDEIOInterface for setting an output and RTDEReceiveInterface for getting the state
      * of an output.
      */

    if (rtde_receive.getDigitalOutState(7))
      std::cout << "Standard digital out (7) is HIGH" << std::endl;
    else
      std::cout << "Standard digital out (7) is LOW" << std::endl;

    if (rtde_receive.getDigitalOutState(16))
      std::cout << "Tool digital out (16) is HIGH" << std::endl;
    else
      std::cout << "Tool digital out (16) is LOW" << std::endl;

    rtde_io.setStandardDigitalOut(7, true);
    rtde_io.setToolDigitalOut(0, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (rtde_receive.getDigitalOutState(7))
      std::cout << "Standard digital out (7) is HIGH" << std::endl;
    else
      std::cout << "Standard digital out (7) is LOW" << std::endl;

    if (rtde_receive.getDigitalOutState(16))
      std::cout << "Tool digital out (16) is HIGH" << std::endl;
    else
      std::cout << "Tool digital out (16) is LOW" << std::endl;

    // How to set a analog output with a specified current ratio
    rtde_io.setAnalogOutputCurrent(1, 0.25);
}

void jointPrint(std::vector<double> vec)
{
    std::cout << "{ ";
    for(int i = 0; i < vec.size(); i++)
        std::cout << vec[i] << ", ";
    std::cout << "\b\b }" << std::endl;
}

void control(std::string robot_ip)
{
    ur_rtde::RTDEControlInterface rtde_control(robot_ip);
    ur_rtde::RTDEReceiveInterface rtde_receive(robot_ip);
    ur_rtde::RTDEIOInterface rtde_io(robot_ip);

    double velocity = 0.4;
    double acceleration = 0.4;
    double blend_2 = 0.02;

    std::vector<double> path_pose1 = {-0.2, -0.5, 0.2, -0.001, 3.12, 0.04, velocity, acceleration, blend_2};
    std::vector<double> path_pose2 = {-0.2, -0.5, 0.5, -0.001, 3.12, 0.04, velocity, acceleration, blend_2};

    std::vector<std::vector<double>> path;
    for(int i = 0; i < 1; i++)
    {
        path.push_back(path_pose2);
        //path.push_back(path_pose2);
    }

    // Send a linear path with blending in between - (currently uses separate script)
    rtde_control.moveL(path);
    rtde_io.setStandardDigitalOut(0, true);
    path.clear();
    path.push_back(path_pose1);
    rtde_control.moveL(path);
    rtde_io.setStandardDigitalOut(0, false);
    path.clear();
    path.push_back(path_pose2);
    rtde_control.moveL(path);
    rtde_io.setStandardDigitalOut(0, true);
    rtde_control.stopScript();

    std::vector<double> joint_positions = rtde_receive.getActualQ();
    jointPrint(joint_positions);
}

int main(int argc, char* argv[])
{
    std::string robot_ip = "192.168.0.212";

    control(robot_ip);

    return 0;
}
