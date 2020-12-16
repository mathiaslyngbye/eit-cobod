#ifndef PLUGIN_HPP
#define PLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

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


#undef foreach
//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Boost includes
#include <boost/bind.hpp>

// Qt includes
#include <QPushButton>
#include <QGridLayout>
#include <QLabel>

// Standard includes
#include <iostream>
#include <thread>
#include <utility>
#include <chrono>
#include <cmath>

// Extra defines for robot gripper
#define OPEN false
#define CLOSE true


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
    void moveToForce(int mode);
    void moveToJ(std::vector<double>, double, double);
    void printLocation();

    // Start thread
    void startRobotMimic();
    void startRobotControl();
    void startHomeRobot();

    // Manage
    std::vector<double> invKin(std::vector<double>);
    void attachObject();
    void resetObject();
    void connectRobot();
    void zeroSensor();
    void stopRobot();
    void teachModeToggle();

    // Assistive
    void printArray(std::vector<double>);
    std::vector<double> addMove(std::vector<double>, double, double);

    // Vision
    void get25DImage();
    //Helper functions
    void nearest_feature(const pcl::Histogram<153>& query, const pcl::PointCloud<pcl::Histogram<153>>& target, int &idx, float &distsq);
    float dist_sq(const pcl::Histogram<153>& query, const pcl::Histogram<153>& target);

    // Planning
    void createPathRRTConnect(std::vector<double>, std::vector<double>, double, double, double, std::vector<std::vector<double>>&, rw::kinematics::State);

private:
    // Qt buttons
    QPushButton *_btn_connect,*_btn_sync,*_btn_control,*_btn_stop,*_btn_teach, *_btn_print, *_btn_home, *_btn_zero, *_btn_image, *_btn_attach;

    // RobWorkStudio interface
    rw::proximity::CollisionDetector::Ptr collisionDetector;
    rw::models::WorkCell::Ptr rws_wc;
    rw::kinematics::State rws_state;
    rw::models::SerialDevice::Ptr rws_robot;
    rw::kinematics::MovableFrame::Ptr rws_rebar;
    rw::kinematics::Frame::Ptr rws_robot_tcp;
    rw::kinematics::Frame::Ptr rws_table;


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

    std::vector<double> pickApproachQ = { 0.773998, -1.3719, 1.55647, -1.7573, -1.56807, -1.97355 };
    std::vector<double> pickApproachL = { -0.313509, -0.493764, 0.407642, 1.74502, -2.61216, 0.000178952 };
    std::vector<double> pickQ =  { 0.762658, -0.972519, 1.91634, -2.51089, -1.5408, -1.98513 };
    std::vector<double> placeApproachQ =   { 1.74693, -1.08593, 1.66241, -2.14615, -1.56947, -0.999986 };
    std::vector<double> placeApproachL =   { 0.241712, -0.593151, 0.224805, -1.7459, 2.61178, 0.00491445 };
    std::vector<double> placeQ =   { 1.81889, -1.15219, 1.91062, -2.26603, -1.53601, -0.886415 };

    std::vector<double> rebarL =  { 0.26965, -0.05, 0.13, 0, 0, 0 };

    //Vision
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;
    std::vector<std::string> _cameras25D;

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
