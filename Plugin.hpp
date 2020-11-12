#ifndef PLUGIN_HPP
#define PLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>

// RobWorkStudio includes
#include <RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

// UR RTDE includes
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

// RobWork library includes
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

// Boost includes
#include <boost/bind.hpp>

// Qt includes
#include <QPushButton>
#include <QGridLayout>

// Standard includes
#include <iostream>
#include <thread>
#include <utility>

// Extra defines for robot gripper
#define OPEN true
#define CLOSE false

class QPushButton;

class Plugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "Plugin.json")
public:
    Plugin();
    virtual ~Plugin();

    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

private slots:
    // Plugin default
    void clickEvent();
    void stateChangedListener(const rw::kinematics::State& state);

    // Functions
    void RunRobotMimic();
    void RunRobotControl();
    void RunHomeRobot();
    void GripRebarFast();
    void GripRebarSlow();
    void PlaceRebarFast();
    void PlaceRebarSlow();
    void PrintQ();

    // Start thread
    void startRobotMimic();
    void startRobotControl();
    void startHomeRobot();

    // Manage
    void connectRobot();
    void stopRobot();
    void teachModeToggle();

    // Assistive
    void printArray(std::vector<double>);
    std::vector<double> addMove(std::vector<double>, double, double);

    // Planning
    void createPathRRTConnect(std::vector<double>, std::vector<double>, double, std::vector<std::vector<double>>&, rw::kinematics::State);

private:
    // Qt buttons
    QPushButton *_btn0,*_btn1,*_btn2,*_btn3,*_btn4, *_btn5, *_btn6, *_btn7, *_btn8, *_btn9, *_btn10;

    // RobWorkStudio interface
    rw::proximity::CollisionDetector::Ptr collisionDetector;
    rw::models::WorkCell::Ptr rws_wc;
    rw::kinematics::State rws_state;
    rw::models::Device::Ptr rws_robot;

    // UR interface
    std::string ur_robot_ip = "192.168.0.212";
    ur_rtde::RTDEControlInterface   *ur_robot;
    ur_rtde::RTDEIOInterface        *ur_robot_io;
    ur_rtde::RTDEReceiveInterface   *ur_robot_receive;

    // Positions                       j0        j1        j2        j3        j4        j5
    std::vector<double> gripQ =     {  1.73867, -1.28944, 1.38227, -1.67159, -1.57051, -0.980185 };
    std::vector<double> gripTCP =   { -0.15573, -0.52874,  0.17813,  1.77626, -2.57197,  0.04202 };
    std::vector<double> homeQ =     {  1.17810, -1.57080,  1.57080, -1.57080, -1.57080, -1.57080 };
    std::vector<double> homeTCP =   { -0.06489, -0.50552,  0.48784, -1.74588,  2.61176,  0.00493 };

    std::vector<double> gripFast =  {   0.762,   -1.362, 1.599, -1.806, -1.541, -1.985 };
    std::vector<double> gripSlow =  { 0.762658, -0.972519, 1.91634, -2.51089, -1.5408, -1.98513 };
    std::vector<double> PlaceFast =   { 1.817, -1.386, 1.643, -1.764, -1.536, -0.883 };
    std::vector<double> PlaceSlow =   { 1.81889, -1.15219, 1.91062, -2.26603, -1.53601, -0.886415 };


// q location for rebars, with 3 reabrs in it Q[6]{0.762658, -0.972519, 1.91634, -2.51089, -1.5408, -1.98513}
// q for place    Q[6]{1.81889, -1.15219, 1.91062, -2.26603, -1.53601, -0.886415}

    // Flags
    std::atomic_bool ur_robot_exists;
    std::atomic_bool ur_robot_stopped;
    std::atomic_bool ur_robot_teach_mode;

    // Threads
    std::thread control_thread;
    std::thread update_thread;
    std::thread home_thread;
};

#endif /*PLUGIN_HPP*/
