#ifndef PLUGIN_HPP
#define PLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>

// RobWorkStudio includes
#include <rws/RobWorkStudioPlugin.hpp>

// Include robworkhardware
#include <rwhw/universalrobots/UniversalRobotsData.hpp>
#include <rwhw/universalrobots_rtde/URRTDE.hpp>

// RobWork library includes
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

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
    void mainEvent();

private slots:
    void clickEvent();
    void stateChangedListener(const rw::kinematics::State& state);
    void buttonDemoEvent(std::string);
    void homeRobotEvent();
    void RunUpdateRobot(rwhw::URRTDE *robot);

private:
    rw::proximity::CollisionDetector::Ptr collisionDetector;
    rw::models::WorkCell::Ptr rws_wc;
    rw::kinematics::State rws_state;
    rw::models::Device::Ptr robot;
    QPushButton *_btn0,*_btn1,*_btn2,*_btn3;
    rw::math::Q home = rw::math::Q(6, 0, 0, 0, 0, 0, 0);
    //std::string robot_ip = "192.168.0.212";
};

#endif /*PLUGIN_HPP*/
