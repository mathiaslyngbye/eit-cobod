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

    _btn6 = new QPushButton("Print Location");
    pLayout->addWidget(_btn6, row++, 0);
    connect(_btn6, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn7 = new QPushButton("Go to Pick Approach");
    pLayout->addWidget(_btn7, row++, 0);
    connect(_btn7, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn8 = new QPushButton("Pick Rebar");
    pLayout->addWidget(_btn8, row++, 0);
    connect(_btn8, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn9 = new QPushButton("Go to Place Approach");
    pLayout->addWidget(_btn9, row++, 0);
    connect(_btn9, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn10 = new QPushButton("Place Rebar");
    pLayout->addWidget(_btn10, row++, 0);
    connect(_btn10, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn11 = new QPushButton("Get 25DImage");
    pLayout->addWidget(_btn11, row++, 0);
    connect(_btn11, SIGNAL(clicked()), this, SLOT(clickEvent()));


    pLayout->setRowStretch(row,1);

    // Initialize flags
    ur_robot_exists = false;
    ur_robot_teach_mode = false;
    ur_robot_stopped = true;

    //Initialize camera vector
    _cameras25D = {"Scanner25D"};
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

    rw::kinematics::Frame* cameraFrame25D = rws_wc->findFrame(_cameras25D[0]);
    if (cameraFrame25D != NULL) {
        if (cameraFrame25D->getPropertyMap().has("Scanner25D")) {
            // Read the dimensions and field of view
            double fovy;
            int width,height;
            std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;
            // Create a frame grabber
            _framegrabber25D = new rwlibs::simulation::GLFrameGrabber25D(width,height,fovy);
            rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
            _framegrabber25D->init(gldrawer);
        }
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
    else if(obj == _btn6)
        printLocation();
    else if(obj == _btn7)
        moveToJ(pickApproachQ,0.8,0.8);
    else if(obj == _btn8)
        moveToForce(CLOSE);
    else if(obj == _btn9)
        moveToJ(placeApproachQ,0.8,0.8);
    else if(obj == _btn10)
        moveToForce(OPEN);
    else if(obj == _btn11)
        get25DImage();
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

    /*
    std::vector<double> grip = addMove(gripQ, 0.2, 0.2);
    std::vector<double> home = addMove(homeQ);

    std::vector<std::vector<double>> path;
    */

    std::vector<double> dynamicPlaceApproachL = placeApproachL;
    double theta = 22.5 * (M_PI / 180);
    double d = 0.05;
    double dy = d*cos(theta);
    double dx = d*sin(theta);
    std::cout << "c=" << 0.05 << "\tdy=" << dy << "\tdx=" << dx << std::endl;

    ur_robot_stopped = false;
    for(int i = 0; i<4; i++)
    {
        if(ur_robot_stopped)
            break;

        // Move home
        moveToJ(homeQ,0.8,0.8);
        ur_robot_io->setStandardDigitalOut(0,OPEN);

        // Move to pick approach
        moveToJ(pickApproachQ, 0.8, 0.8);

        // Grip
        moveToForce(CLOSE);
        ur_robot->moveL(pickApproachL,0.8,0.8);

        // Move home
        moveToJ(homeQ,0.8,0.8);

        // Move to place approach
        ur_robot->moveL(dynamicPlaceApproachL,0.8,0.8);

        // Ungrip
        moveToForce(OPEN);
        ur_robot->moveL(dynamicPlaceApproachL,0.8,0.8);

        // Move home
        moveToJ(homeQ,0.8,0.8);

        // Calculate new approach
        dynamicPlaceApproachL[0]+=dx;
        dynamicPlaceApproachL[1]+=dy;
        std::cout << "New location:" << std::endl;
        printArray(dynamicPlaceApproachL);

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
    ur_robot->stopL();
    ur_robot->stopJ();
    ur_robot_stopped = true;
}

void Plugin::moveToJ(std::vector<double> goal, double acceleration, double velocity)
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

    std::cout << "Moving to location" << std::endl;
    ur_robot->moveJ(goal, acceleration, velocity);
}

void Plugin::moveToForce(int mode)
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

    std::cout << "Moving to pick location..." << std::endl;
    //std::vector<double> pickL = { -0.313482, -0.493764, 0.14425, 1.7451, -2.61215, 0.000203642 };
    //std::vector<double> pickJ = { -0.313482, -0.493764, 0.14425, 1.7451, -2.61215, 0.000203642 };

    // Forcemode parameters
    std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
    std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
    std::vector<double> wrench_down = {0, 0, -50, 0, 0, 0};
    std::vector<double> limits = {2, 2, 10, 1, 1, 1};
    int force_type = 2;
    double dt = 1.0/2000; // 2ms

    for(int i = 0; i < 10000; i++)
    {
        if(ur_robot_receive->getActualTCPForce()[2]>2)
            break;

        auto t_start = std::chrono::high_resolution_clock::now();                           // Move time start
        ur_robot->forceMode(task_frame, selection_vector, wrench_down, force_type,limits);  // Move
        auto t_stop = std::chrono::high_resolution_clock::now();                            // Move time stop
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);                  // Move duration

        // If move was shorter than timestep dt, sleep until sync
        if (t_duration.count() < dt)
          std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
    ur_robot->forceModeStop();

    // Grip
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ur_robot_io->setStandardDigitalOut(0,mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

void Plugin::printLocation()
{
    std::cout << "JOINT POSE:" << std::endl;
    std::vector<double> actualQ=ur_robot_receive->getActualQ();
    printArray(actualQ);

    std::cout << "TCP POSE:" << std::endl;
    std::vector<double> actualL=ur_robot_receive->getActualTCPPose();
    printArray(actualL);
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
        std::vector<double> q_copy = q.toStdVector();
        path.push_back(addMove(q_copy, 0.5, 0.5));
    }
}

void Plugin::get25DImage()
{
    std::vector<double> q_vector = {rw::math::Pi/2,-rw::math::Pi/2,rw::math::Pi/2,-rw::math::Pi/2,-rw::math::Pi/2,0};
    rw::math::Q new_q(q_vector);
    //rw::math::Q old_q = rws_robot->getQ(rws_state);
    rws_robot->setQ(new_q, rws_state);
    getRobWorkStudio()->setState(rws_state);
    if (_framegrabber25D != NULL)
    {
        for( int i = 0; i < _cameras25D.size(); i ++)
        {
            // Get the image as a RW image
            rw::kinematics::Frame* cameraFrame25D = rws_wc->findFrame(_cameras25D[i]); // "Camera");
            _framegrabber25D->grab(cameraFrame25D, rws_state);

            //const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage());

            std::ofstream output(_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth() << "\n";
            output << "HEIGHT " << img->getHeight() << "\n";
            output << "POINTS " << img->getData().size() << "\n";
            output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData())
            {
                rw::math::Vector3D<float> p = p_tmp;
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();

        }
    }
}

