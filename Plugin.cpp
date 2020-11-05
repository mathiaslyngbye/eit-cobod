#include "Plugin.hpp"

Plugin::Plugin():
    rws::RobWorkStudioPlugin("Plugin", QIcon(":/plugin.png"))
{
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    // Define button layout
    int row = 0;

    _btn0 = new QPushButton("Connect");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn1 = new QPushButton("Start robot mimic");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn2 = new QPushButton("Start robot control");
    pLayout->addWidget(_btn2, row++, 0);
    connect(_btn2, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn3 = new QPushButton("Stop robot");
    pLayout->addWidget(_btn3, row++, 0);
    connect(_btn3, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn4 = new QPushButton("Toggle teach mode");
    pLayout->addWidget(_btn4, row++, 0);
    connect(_btn4, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn5 = new QPushButton("Home robot");
    pLayout->addWidget(_btn5, row++, 0);
    connect(_btn5, SIGNAL(clicked()), this, SLOT(clickEvent()));

    pLayout->setRowStretch(row,1);

    // Initialize flags
    ur_robot_exists = false;
    ur_robot_teach_mode = false;
    ur_robot_stopped = true;
}

Plugin::~Plugin()
{
}

void Plugin::initialize()
{
    std::cout << "Start of initialize()" << std::endl;
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&Plugin::stateChangedListener, this, boost::arg<1>()), this);
    std::cout << "End of initialize()" << std::endl;
}

void Plugin::open(rw::models::WorkCell* workcell)
{
    std::cout << "Start of open()" << std::endl;

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
    std::cout << "Start of close()" << std::endl;
    std::cout << "End of close()" << std::endl;
}

void Plugin::clickEvent()
{
    // log().info() << "Button 0 pressed!\n";
    QObject *obj = sender();

    if(obj == _btn0)
        connectRobot();
    else if(obj == _btn1)
        startRobotMimic();
    else if(obj == _btn2)
        startRobotControl();
    else if(obj == _btn3)
        stopRobot();
    else if(obj == _btn4)
        teachModeToggle();
    else if(obj == _btn5)
        startHomeRobot();
}

void Plugin::stateChangedListener(const rw::kinematics::State& state)
{
    rws_state = state;
    log().info() << "State changed!";
}

void Plugin::connectRobot()
{
    std::cout << "Connecting to " << ur_robot_ip << "..." << std::endl;
    if(!ur_robot_exists)
    {
        std::cout << "Control interface:\t";
        ur_robot = new ur_rtde::RTDEControlInterface(ur_robot_ip);
        std::cout << "Connected!" << std::endl;
        std::cout << "Receive interface:\t";
        ur_robot_receive = new ur_rtde::RTDEReceiveInterface(ur_robot_ip);
        std::cout << "Connected!" << std::endl;
        std::cout << "IO interface:\t";
        ur_robot_io = new ur_rtde::RTDEIOInterface(ur_robot_ip);
        std::cout << "Connected!" << std::endl;

        ur_robot_exists = true;
        std::cout << "Robot ready!" << std::endl;
    }
    else
        std::cout << "Already connected..." << std::endl;
}

std::vector<double> Plugin::addMove(std::vector<double> position, double acceleration = 0.5, double velocity = 0.5)
{
    std::vector<double> move = { acceleration, velocity };
    std::vector<double> position_and_move;
    position_and_move.reserve(position.size() + move.size());
    position_and_move.insert( position_and_move.end(), position.begin(), position.end() );
    position_and_move.insert( position_and_move.end(), move.begin(), move.end() );
    return position_and_move;
}

void Plugin::RunRobotControl()
{
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    if(ur_robot_teach_mode)
    {
        std::cout << "Teach mode enabled..." << std::endl;
        return;
    }

    std::vector<double> grip = addMove(gripQ, 0.2, 0.2);
    std::vector<double> home = addMove(homeQ);

    std::vector<std::vector<double>> path;

    ur_robot_stopped = false;
    while(!ur_robot_stopped)
    {
        // Open and move down
        path.clear();
        ur_robot_io->setStandardDigitalOut(0,OPEN);
        path.push_back(grip);
        ur_robot->moveJ(path);

        // Close and move up
        ur_robot_io->setStandardDigitalOut(0,CLOSE);
        path.clear();
        path.push_back(home);
        ur_robot->moveJ(path);
    }
    // ur_robot->stopScript(); // Stops further actions
}

void Plugin::teachModeToggle()
{
    if(ur_robot_exists)
    {
        if(ur_robot_teach_mode)
        {
            ur_robot->endTeachMode();
            std::cout << "Disabling teach mode!" << std::endl;
            ur_robot_teach_mode = false;
        }
        else
        {
            ur_robot->teachMode();
            std::cout << "Enabling teach mode!" << std::endl;
            ur_robot_teach_mode = true;
        }
    }
    else
        std::cout << "Robot not connected..." << std::endl;
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
        std::vector<double> currentQ = ur_robot_receive->getActualQ();
        rw::kinematics::State s = rws_state;
        rws_robot->setQ(currentQ, s);
        getRobWorkStudio()->setState(s);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
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

void Plugin::startHomeRobot()
{
    if(home_thread.joinable())
        home_thread.join();
    home_thread = std::thread(&Plugin::RunHomeRobot, this);
}

void Plugin::stopRobot()
{
    std::cout << "Stopping robot..." << std::endl;
    ur_robot_stopped = true;
}

void Plugin::RunHomeRobot()
{
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    if(ur_robot_teach_mode)
    {
        std::cout << "Teach mode enabled..." << std::endl;
        return;
    }

    std::cout << "Homing robot..." << std::endl;

    std::vector<std::vector<double>> path;
    std::vector<double> fromQ = ur_robot_receive->getActualQ();
    std::vector<double> toQ = homeQ;
    rw::kinematics::State tmp_state = rws_state;

    createPathRRTConnect(fromQ, toQ, 0.05, path, tmp_state);

    std::cout << "Moving robot..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ur_robot->moveJ(path);
}

void Plugin::printArray(std::vector<double> input)
{
    std::cout << "{ ";
    for(size_t i = 0; i < input.size()-1; i++)
        std::cout << input[i] << ", ";
    std::cout << input[input.size()-1] << " }" << std::endl;
}

void Plugin::createPathRRTConnect(std::vector<double> from, std::vector<double> to, double epsilon, std::vector<std::vector<double>> &path, rw::kinematics::State state)
{
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(collisionDetector.get(), rws_robot, state);
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(rws_robot), constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    rw::trajectory::QPath qpath;
    std::cout << "Generating path with NO max. time. Be patient or cancel manually... " << std::endl;
    planner->query(from, to, qpath); // DOES THIS JUST ACCEPT VECTORS OF DOUBLES???
    std::cout << "Found path of size " << qpath.size() << '!' << std::endl;

    path.clear();
    for(const auto &q : qpath)
    {
        path.push_back(addMove(q.toStdVector()));
    }
}



