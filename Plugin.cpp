#include "Plugin.hpp"

Plugin::Plugin():
    rws::RobWorkStudioPlugin("Plugin", QIcon(":/plugin.png"))
{
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    int row = 0;

    _btn0 = new QPushButton("Connect");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn1 = new QPushButton("Home rws robot");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn2 = new QPushButton("Start robot mimic");
    pLayout->addWidget(_btn2, row++, 0);
    connect(_btn2, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn3 = new QPushButton("Start robot control");
    pLayout->addWidget(_btn3, row++, 0);
    connect(_btn3, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn4 = new QPushButton("Stop robot");
    pLayout->addWidget(_btn4, row++, 0);
    connect(_btn4, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn5 = new QPushButton("Start teach mode");
    pLayout->addWidget(_btn5, row++, 0);
    connect(_btn5, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn6 = new QPushButton("Stop teach mode");
    pLayout->addWidget(_btn6, row++, 0);
    connect(_btn6, SIGNAL(clicked()), this, SLOT(clickEvent()));

    pLayout->setRowStretch(row,1);
}

Plugin::~Plugin()
{
}

void Plugin::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&Plugin::stateChangedListener, this, boost::arg<1>()), this);

    std::cout << "End of initialize()" << std::endl;
}

void Plugin::open(rw::models::WorkCell* workcell)
{
    // If workcell exists
    if (workcell != NULL)
    {
        // Get rws info
        rws_wc = workcell;
        rws_state = rws_wc->getDefaultState();
        rws_robot = rws_wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");

        // Use rws collision checker
        collisionDetector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(rws_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    }
    std::cout << "End of open()" << std::endl;
}

void Plugin::close()
{
    std::cout << "End of close()" << std::endl;
}

void Plugin::clickEvent()
{
    // log().info() << "Button 0 pressed!\n";
    QObject *obj = sender();

    if(obj == _btn0)
        connectRobot();
    else if(obj == _btn1)
        homeRobot();
    else if(obj == _btn2)
        startRobotMimic();
    else if(obj == _btn3)
        startRobotControl();
    else if(obj == _btn4)
        stopRobot();
    else if(obj == _btn5)
        teachMode(true);
    else if(obj == _btn6)
        teachMode(false);
}

void Plugin::stateChangedListener(const rw::kinematics::State& state)
{
    rws_state = state;
    log().info() << "State changed!";
}

void Plugin::RunRobotControl()
{
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    const rw::math::Transform3D<> pose_up = rw::math::Transform3D<>(
            rw::math::Vector3D<>(-0.2, -0.5, 0.5),
            rw::math::RPY<>(0, 3.12, 0)
            );

    const rw::math::Transform3D<> pose_down = rw::math::Transform3D<>(
            rw::math::Vector3D<>(-0.2, -0.5, 0.2),
            rw::math::RPY<>(0, 3.12, 0)
            );

    rw::trajectory::Transform3DPath lpath;

    ur_robot_stopped = false;
    while(!ur_robot_stopped)
    {
        // Open and move down
        lpath.clear();
        ur_robot->setStandardDigitalOut(0,OPEN);
        lpath.push_back(pose_down);
        ur_robot->moveL(lpath);

        // Close and move up
        ur_robot->setStandardDigitalOut(0,CLOSE);
        lpath.clear();
        lpath.push_back(pose_up);
        ur_robot->moveL(lpath);
    }

    ur_robot->stopRobot();
}

void Plugin::teachMode(bool tm)
{
    if(tm)
        ur_robot->teachMode();
    else
        ur_robot->endTeachMode();
}

void Plugin::updateState()
{
    if(ur_robot_exists)
    {
        rw::math::Q currentQ = ur_robot->getActualQ();
        rw::kinematics::State s = rws_state;
        rws_robot->setQ(currentQ, s);
        getRobWorkStudio()->setState(s);
    }
}

void Plugin::RunRobotMimic()
{
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    while(true)
    {
        updateState();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Plugin::connectRobot()
{
    std::cout << "Connecting to " << ur_robot_ip << "..." << std::endl;
    if(!ur_robot_exists)
    {
        ur_robot = new rwhw::URRTDE(ur_robot_ip);
        ur_robot_exists = true;
        std::cout << "Connected!" << std::endl;
    }
    else
        std::cout << "Already connected..." << std::endl;
}

void Plugin::homeRobot()
{
    std::cout << "Setting RWS robot home Q:" << std::endl;
    std::cout << "> From:\t" << rws_robot->getQ(rws_state) << std::endl;
    std::cout << "> To:\t" << home << std::endl;
    rws_robot->setQ(home,rws_state);
    getRobWorkStudio()->setState(rws_state);
}

void Plugin::startRobotMimic()
{
    if(update_thread.joinable())
        update_thread.join();
    update_thread = std::thread(&Plugin::RunRobotMimic, this);
}

void Plugin::startRobotControl()
{
    if(control_thread.joinable())
        control_thread.join();
    control_thread = std::thread(&Plugin::RunRobotControl, this);
}

void Plugin::stopRobot()
{
    std::cout << "Stopping robot..." << std::endl;
    ur_robot_stopped = true;
}
