#ifndef PLUGIN_HPP
#define PLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>

// RobWorkStudio includes
#include <RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

// RobWorkHardware includes
#include <rwhw/universalrobots/UniversalRobotsData.hpp>
#include <rwhw/universalrobots_rtde/URRTDE.hpp>

// RobWork library includes
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

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
    void clickEvent();
    void stateChangedListener(const rw::kinematics::State& state);

    // Threads
    void RunRobotMimic();
    void RunRobotControl();

    // Button functions
    void startRobotMimic();
    void startRobotControl();
    void teachMode(bool);
    void homeRobot();
    void connectRobot();
    void stopRobot();
    void updateState();

private:
    rw::proximity::CollisionDetector::Ptr collisionDetector;
    rw::models::WorkCell::Ptr rws_wc;
    rw::kinematics::State rws_state;
    rw::models::Device::Ptr rws_robot;
    QPushButton *_btn0,*_btn1,*_btn2,*_btn3,*_btn4, *_btn5, *_btn6;
    rw::math::Q home = rw::math::Q(6, 0, 0, 0, 0, 0, 0);

    std::string ur_robot_ip = "192.168.0.212";
    rwhw::URRTDE *ur_robot;
    bool ur_robot_exists = false;

    // Flags
    std::atomic_bool ur_robot_stopped;

    // Threads
    std::thread control_thread;
    std::thread update_thread;
};

#endif /*PLUGIN_HPP*/
