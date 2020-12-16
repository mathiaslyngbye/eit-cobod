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

    // Initiation
    QLabel *label_init = new QLabel(this);
    label_init->setText("Initiation");
    pLayout->addWidget(label_init,row++,0);

    _btn_connect = new QPushButton("Connect");
    pLayout->addWidget(_btn_connect, row++, 0);
    connect(_btn_connect, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_sync = new QPushButton("Synchronize movement");
    pLayout->addWidget(_btn_sync, row++, 0);
    connect(_btn_sync, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_stop_sync = new QPushButton("Stop synchronized movement");
    pLayout->addWidget(_btn_stop_sync, row++, 0);
    connect(_btn_stop_sync, SIGNAL(clicked()), this, SLOT(clickEvent()));

    // Movement/Control
    QLabel *label_control = new QLabel(this);
    label_control->setText("Control");
    pLayout->addWidget(label_control,row++,0);

    _btn_control = new QPushButton("Start robot control");
    pLayout->addWidget(_btn_control, row++, 0);
    connect(_btn_control, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_stop = new QPushButton("Stop robot");
    pLayout->addWidget(_btn_stop, row++, 0);
    connect(_btn_stop, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_home = new QPushButton("Home robot");
    pLayout->addWidget(_btn_home, row++, 0);
    connect(_btn_home, SIGNAL(clicked()), this, SLOT(clickEvent()));

    // Debug / Managing
    QLabel *label_debug = new QLabel(this);
    label_debug->setText("Debugging");
    pLayout->addWidget(label_debug,row++,0);

    _btn_print = new QPushButton("Print Location");
    pLayout->addWidget(_btn_print, row++, 0);
    connect(_btn_print, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_zero = new QPushButton("Zero force-torque sensor");
    pLayout->addWidget(_btn_zero, row++, 0);
    connect(_btn_zero, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_teach = new QPushButton("Enable teach mode");
    pLayout->addWidget(_btn_teach, row++, 0);
    connect(_btn_teach, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_attach = new QPushButton("Attach rebar");
    pLayout->addWidget(_btn_attach, row++, 0);
    connect(_btn_attach, SIGNAL(clicked()), this, SLOT(clickEvent()));

    // Vision
    QLabel *label_vision = new QLabel(this);
    label_vision->setText("Vision");
    pLayout->addWidget(label_vision,row++,0);

    _btn_image = new QPushButton("Get 25D image");
    pLayout->addWidget(_btn_image, row++, 0);
    connect(_btn_image, SIGNAL(clicked()), this, SLOT(clickEvent()));

    pLayout->setRowStretch(row,1);

    // Initialize flags
    ur_robot_exists = false;
    ur_robot_teach_mode = false;
    ur_robot_stopped = true;
    rws_robot_synced = false;

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
        rws_state       = rws_wc->getDefaultState();
        rws_robot       = rws_wc->findDevice<rw::models::SerialDevice>("UR5e_2018");
        rws_rebar       = rws_wc->findFrame<rw::kinematics::MovableFrame>("Rebar");
        rws_robot_tcp   = rws_wc->findFrame<rw::kinematics::Frame>("GraspTCP");
        rws_robot_base  = rws_wc->findFrame<rw::kinematics::Frame>("UR5e_2018.Base");
        rws_table       = rws_wc->findFrame<rw::kinematics::Frame>("Table");

        if(rws_robot == NULL)
        {
            std::cout << "Couldn't locate rws_robot!" << std::endl;
            return;
        }

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

    if(obj == _btn_connect)
        connectRobot();
    else if(obj == _btn_sync)
        startRobotMimic();
    else if(obj == _btn_control)
        startRobotControl();
    else if(obj == _btn_stop)
        stopRobot();
    else if(obj == _btn_stop_sync)
        stopSync();
    else if(obj == _btn_teach)
        teachModeToggle();
    else if(obj == _btn_home)
        startHomeRobot();
    else if(obj == _btn_print)
        printLocation();
    else if(obj == _btn_zero)
        zeroSensor();
    else if(obj == _btn_attach)
        attachObject();
    else if(obj == _btn_image)
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

std::vector<double> Plugin::addMove(std::vector<double> position, double acceleration = 0.5, double velocity = 0.5, double blend = 0.2)
{
    std::vector<double> move = { acceleration, velocity, blend };
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

    std::vector<double> dynamicPlaceApproachL = placeApproachL_RW;
    double theta = 22.5 * (M_PI / 180);
    double d = 0.05;
    double dy = d*cos(theta);
    double dx = d*sin(theta);
    std::cout << "c=" << 0.05 << "\tdy=" << dy << "\tdx=" << dx << std::endl;

    // Home robot and calibrate before start
    moveToJ(homeQ,0.8,0.8);
    ur_robot_io->setStandardDigitalOut(0,OPEN);
    zeroSensor();
    ur_robot->setPayload(0.2);

    ur_robot_stopped = false;
    for(int i = 0; i<4; i++)
    {
        if(ur_robot_stopped)
            break;

        // Move to pick approach
        moveToJ(pickApproachQ, 0.8, 0.8);

        // Grip
        moveToForce(CLOSE);
        attachObject();
        ur_robot->moveL(pickApproachL,0.8,0.8);


        // RRT Between points
        std::vector<std::vector<double>> path;
        std::vector<double> fromQ = ur_robot_receive->getActualQ();
        std::vector<double> toQ = invKin(fromQ,dynamicPlaceApproachL);
        rw::kinematics::State tmp_state = rws_state.clone();
        createPathRRTConnect(fromQ, toQ, 0.05, 0.4, 0.4, 0.02, path, tmp_state);
        path.push_back(addMove(toQ, 0.4, 0.4, 0));

        std::cout << "Moving robot..." << std::endl;
        ur_robot->moveJ(path);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Move home
        //moveToJ(homeQ,0.8,0.8);

        // Move to place approach
        //std::vector<double> fromQ = rws_robot->getQ(rws_state).toStdVector();
        //std::vector<double> toQ = invKin(fromQ, dynamicPlaceApproachL);
        //ur_robot->moveJ(toQ,0.4,0.4);

        // Place
        moveToForce(OPEN);
        resetObject();
        ur_robot->moveJ(toQ,0.4,0.4);
        //ur_robot->moveL(dynamicPlaceApproachL,0.8,0.8);

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

    rws_robot_synced = true;
    while(rws_robot_synced)
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

void Plugin::stopSync()
{
    std::cout << "Stopping robot sync..." << std::endl;
    rws_robot_synced = false;
}

void Plugin::zeroSensor()
{
    std::cout << "Calibrating..." << std::endl;
    ur_robot->zeroFtSensor();
    std::cout << "Done!" << std::endl;
}

void Plugin::attachObject()
{
     rw::kinematics::State tmp_state = rws_state.clone();

     // Attach rebar to TCP
     rws_rebar->setTransform(
                 rw::math::Transform3D<>(
                     rw::math::Vector3D<>(0, 0, 0),
                     rw::math::RPY<>(0, 0, 0)),
             tmp_state
             );
     rws_rebar->attachTo(rws_robot_tcp.get(), tmp_state);

     getRobWorkStudio()->setState(tmp_state);
}

void Plugin::resetObject()
{
     rw::kinematics::State tmp_state = rws_state.clone();

     // Attach rebar to Table
     rws_rebar->setTransform(
                 rw::math::Transform3D<>(
                     rw::math::Vector3D<>(0.297, -0.1, 0.126),
                     rw::math::RPY<>(0, 0, 0)),
             tmp_state
             );
     rws_rebar->attachTo(rws_table.get(), tmp_state);

     getRobWorkStudio()->setState(tmp_state);
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
    std::cout << "Homing robot..." << std::endl;
    rw::kinematics::State tmp_state = rws_state.clone();

    std::vector<std::vector<double>> path;
    //std::vector<double> fromQ = ur_robot_receive->getActualQ();
    std::vector<double> fromQ = rws_robot->getQ(tmp_state).toStdVector();
    std::vector<double> toQ = invKin(fromQ, placeApproachL_RW);


    rws_robot->setQ(toQ, tmp_state);
    getRobWorkStudio()->setState(tmp_state);

    /*
    printArray(fromQ);
    printArray(toQ);

    std::cout << "> Running RRT" << std::endl;
    createPathRRTConnect(fromQ, toQ, 0.05, 0.8, 0.8, path, tmp_state);

    std::cout << "Moving robot..." << std::endl;
    for(int i = 0; i < path.size(); i++)
    {
        printArray(path[i]);
    }

    path.clear();
    path.push_back(addMove(fromQ,0.5,0.5));
    path.push_back(addMove(homeQ,0.5,0.5));
    ur_robot->moveJ(path);
    */
}

void Plugin::printLocation()
{
    printArray(rws_robot->getQ(rws_state).toStdVector());
    /*
    std::cout << "JOINT POSE:" << std::endl;
    std::vector<double> actualQ=ur_robot_receive->getActualQ();
    printArray(actualQ);

    std::cout << "TCP POSE:" << std::endl;
    std::vector<double> actualL=ur_robot_receive->getActualTCPPose();
    printArray(actualL);
    */
}

void Plugin::printArray(std::vector<double> input)
{
    std::cout << "{ ";
    for(size_t i = 0; i < input.size()-1; i++)
        std::cout << input[i] << ", ";
    std::cout << input[input.size()-1] << " }" << std::endl;
}

void Plugin::createPathRRTConnect(std::vector<double> from, std::vector<double> to, double epsilon, double velocity, double acceleration, double blend, std::vector<std::vector<double>> &path, rw::kinematics::State state)
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
    int index = 0;
    for(const auto &q : qpath)
    {
        std::vector<double> q_copy = q.toStdVector();
        if(index < qpath.size()-2)
        {
            path.push_back(addMove(q_copy, velocity, acceleration, blend));
            std::cout << "This is not last point" << std::endl;
        }
        else
        {
            path.push_back(addMove(q_copy, velocity, acceleration, 0));
            std::cout << "This is last point" << std::endl;
            //printArray(q.toStdVector());
        }
        index++;
    }
}

std::vector<double> Plugin::invKin(std::vector<double> startQ, std::vector<double> goalL)
{
    // Duplicate state
    rw::kinematics::State tmp_state = rws_state.clone();

    // Create solver
    const rw::invkin::ClosedFormIKSolverUR solver(rws_robot, tmp_state);

    // Create goal transform object
    std::cout << "Goal (TCP):" << std::endl;
    printArray(goalL);

    const rw::math::Transform3D<> homeT = rws_robot_base->wTf(tmp_state);

    const rw::math::Transform3D<> Tdesired(
            rw::math::Vector3D<>(goalL[0], goalL[1], goalL[2]),
            rw::math::RPY<>(goalL[3], goalL[4], goalL[5]));

    const rw::math::Transform3D<> Tdesired_wtf = Tdesired*homeT;

    const std::vector<rw::math::Q> solutions = solver.solve(Tdesired_wtf, tmp_state);
    std::cout << "Found " << solutions.size() << " inverse kinematic solutions!" << std::endl;

    // Use first solution (SHOULD USE SHORTEST CONFIG DISTANCE)
    int best_solution_index = 0;
    double best_solution_distance = 99999;
    int index = 0;
    for(const auto &solution : solutions)
    {
        rws_robot->setQ(solution, tmp_state);
        //getRobWorkStudio()->setState(tmp_state);
        if( !collisionDetector->inCollision(tmp_state,NULL,true) )
        {
            double solution_distance = getConfDistance(startQ, solution.toStdVector());
            if(solution_distance < best_solution_distance)
            {
                best_solution_index = index;
                best_solution_distance = solution_distance;
            }
        }
        index++;
    }

    return solutions[best_solution_index].toStdVector();
}

double Plugin::getConfDistance(std::vector<double> a, std::vector<double> b)
{
    double sum = 0;
    for(size_t i = 0; i<a.size(); i++)
    {
        sum += pow((a[i] - b[i]),2);
    }
    sum = sqrt(sum);
    return sum;
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
        for(size_t i = 0; i < _cameras25D.size(); i ++)
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
                //std::cout << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();

            pcl::PCLPointCloud2::Ptr object_in (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr scene_in (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr scene_cropped (new pcl::PCLPointCloud2 ());

            pcl::io::loadPCDFile("wall_hr.pcd", *object_in);
            pcl::io::loadPCDFile(_cameras25D[i] + ".pcd", *scene_in);


            Eigen::Vector4f minPoint;
            minPoint[0]=-2;  // define minimum point x
            minPoint[1]=-2;  // define minimum point y
            minPoint[2]=-2;  // define minimum point z
            Eigen::Vector4f maxPoint;
            maxPoint[0]=1;  // define max point x
            maxPoint[1]=1;  // define max point y
            maxPoint[2]=0;  // define max point z

            pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
            cropFilter.setInputCloud (scene_in);
            cropFilter.setMin(minPoint);
            cropFilter.setMax(maxPoint);

            cropFilter.filter (*scene_cropped);

            //pcl::io::savePCDFile ("test_cropbox.pcd", *scene_out);

            pcl::PCLPointCloud2::Ptr object_temp (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr scene_temp (new pcl::PCLPointCloud2 ());

            pcl::VoxelGrid<pcl::PCLPointCloud2> voxobj;
            voxobj.setInputCloud (object_in);
            voxobj.setLeafSize (0.01f, 0.01f, 0.01f);
            voxobj.filter (*object_temp);

            pcl::VoxelGrid<pcl::PCLPointCloud2> voxscene;
            voxscene.setInputCloud (scene_cropped);
            voxscene.setLeafSize (0.01f, 0.01f, 0.01f);
            voxscene.filter (*scene_temp);

            pcl::PointCloud<pcl::PointNormal>::Ptr object_filtered(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointNormal>);

            pcl::fromPCLPointCloud2( *object_temp, *object_filtered);
            pcl::fromPCLPointCloud2( *scene_temp, *scene_filtered);

            // Show
            {
                pcl::visualization::PCLVisualizer v("Before global alignment");
                v.addPointCloud<pcl::PointNormal>(object_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_filtered, 0, 255, 0), "object");
                v.addPointCloud<pcl::PointNormal>(scene_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scene_filtered, 255, 0, 0),"scene");
                v.spin();
            }

            // Compute surface normals
            {
                pcl::ScopeTime t("Surface normals");
                pcl::NormalEstimation<pcl::PointNormal,pcl::PointNormal> ne;
                ne.setKSearch(10);

                ne.setInputCloud(object_filtered);
                ne.compute(*object_filtered);

                ne.setInputCloud(scene_filtered);
                ne.compute(*scene_filtered);
            }

            // Compute shape features
            pcl::PointCloud<pcl::Histogram<153>>::Ptr object_features(new pcl::PointCloud<pcl::Histogram<153>>);
            pcl::PointCloud<pcl::Histogram<153>>::Ptr scene_features(new pcl::PointCloud<pcl::Histogram<153>>);
            {
                pcl::ScopeTime t("Shape features");

                pcl::SpinImageEstimation<pcl::PointNormal,pcl::PointNormal,pcl::Histogram<153>> spin;
                spin.setRadiusSearch(0.05);

                spin.setInputCloud(object_filtered);
                spin.setInputNormals(object_filtered);
                spin.compute(*object_features);

                spin.setInputCloud(scene_filtered);
                spin.setInputNormals(scene_filtered);
                spin.compute(*scene_features);
            }

            // Find feature matches
            pcl::Correspondences corr(object_features->size());
            {
                pcl::ScopeTime t("Feature matches");
                for(size_t i = 0; i < object_features->size(); ++i) {
                    corr[i].index_query = i;
                    nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
                }
            }

            // Show matches
            {
                pcl::visualization::PCLVisualizer v("Matches");
                v.addPointCloud<pcl::PointNormal>(object_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_filtered, 0, 255, 0), "object");
                v.addPointCloud<pcl::PointNormal>(scene_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scene_filtered, 255, 0, 0),"scene");
                v.addCorrespondences<pcl::PointNormal>(object_filtered, scene_filtered, corr, 1);
                v.spin();
            }

            // Create a k-d tree for scene
            pcl::search::KdTree<pcl::PointNormal> tree;
            tree.setInputCloud(scene_filtered);

            // Set RANSAC parameters
            const size_t iter = 5000;
            const float thressq = 0.01 * 0.01;

            // Start RANSAC
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned(new pcl::PointCloud<pcl::PointNormal>);
            float penalty = FLT_MAX;
            {
                pcl::ScopeTime t("RANSAC");
                cout << "Starting RANSAC..." << endl;
                pcl::common::UniformGenerator<int> gen(0, corr.size() - 1);
                for(size_t i = 0; i < iter; ++i) {
                    if((i + 1) % 100 == 0)
                        cout << "\t" << i+1 << endl;
                    // Sample 3 random correspondences
                    std::vector<int> idxobj(3);
                    std::vector<int> idxscn(3);
                    for(int j = 0; j < 3; ++j) {
                        const int idx = gen.run();
                        idxobj[j] = corr[idx].index_query;
                        idxscn[j] = corr[idx].index_match;
                    }

                    // Estimate transformation
                    Eigen::Matrix4f T;
                    pcl::registration::TransformationEstimationSVD<pcl::PointNormal,pcl::PointNormal> est;
                    est.estimateRigidTransformation(*object_filtered, idxobj, *scene_filtered, idxscn, T);

                    // Apply pose
                    pcl::transformPointCloud(*object_filtered, *object_aligned, T);

                    // Validate
                    std::vector<std::vector<int> > idx;
                    std::vector<std::vector<float> > distsq;
                    tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

                    // Compute inliers and RMSE
                    size_t inliers = 0;
                    float rmse = 0;
                    for(size_t j = 0; j < distsq.size(); ++j)
                        if(distsq[j][0] <= thressq)
                            ++inliers, rmse += distsq[j][0];
                    rmse = sqrtf(rmse / inliers);

                    // Evaluate a penalty function
                    const float outlier_rate = 1.0f - float(inliers) / object_filtered->size();
                    //const float penaltyi = rmse;
                    const float penaltyi = outlier_rate;

                    // Update result
                    if(penaltyi < penalty) {
                        cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
                        penalty = penaltyi;
                        pose = T;
                    }
                }

                transformPointCloud(*object_filtered, *object_aligned, pose);

                // Compute inliers and RMSE
                std::vector<std::vector<int> > idx;
                std::vector<std::vector<float> > distsq;
                tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
                size_t inliers = 0;
                float rmse = 0;
                for(size_t i = 0; i < distsq.size(); ++i)
                    if(distsq[i][0] <= thressq)
                        ++inliers, rmse += distsq[i][0];
                rmse = sqrtf(rmse / inliers);

                // Print pose
                cout << "Got the following pose:" << endl << pose << endl;
                cout << "Inliers: " << inliers << "/" << object_filtered->size() << endl;
                cout << "RMSE: " << rmse << endl;
            } // End timing

            // Show result
            {
                pcl::visualization::PCLVisualizer v("After global alignment");
                v.addPointCloud<pcl::PointNormal>(object_aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
                v.addPointCloud<pcl::PointNormal>(scene_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scene_filtered, 255, 0, 0),"scene");
                v.spin();
            }


        }
    }
}

float Plugin::dist_sq(const pcl::Histogram<153>& query, const pcl::Histogram<153>& target) {
    float result = 0.0;
    for(int i = 0; i < pcl::Histogram<153>::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }

    return result;
}

void Plugin::nearest_feature(const pcl::Histogram<153>& query, const pcl::PointCloud<pcl::Histogram<153>>& target, int &idx, float &distsq) {

    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}

