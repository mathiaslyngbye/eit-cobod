#include "Plugin.hpp"

// Qt includes
#include <QPushButton>
#include <QGridLayout>

// RobWork includes
#include <RobWorkStudio.hpp>

// standard includes
#include <boost/bind.hpp>

Plugin::Plugin():
    rws::RobWorkStudioPlugin("Plugin", QIcon(":/plugin.png"))
{
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    int row = 0;

    _btn0 = new QPushButton("Button0");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn1 = new QPushButton("Button1");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn2 = new QPushButton("Home robot");
    pLayout->addWidget(_btn2, row++, 0);
    connect(_btn2, SIGNAL(clicked()), this, SLOT(clickEvent()));

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

        // Locate robot in workcell
        robot = rws_wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");

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
    QObject *obj = sender();
    if(obj == _btn0)
    {
        log().info() << "Button 0 pressed!\n";
        buttonDemoEvent("Button 0");
    }
    else if(obj == _btn1)
    {
        log().info() << "Button 1 pressed!\n";
        buttonDemoEvent("Button 1");
    }
    else if(obj == _btn2)
    {
        log().info() << "Button 2 pressed!\n";
        buttonDemoEvent("Home robot");
        homeRobotEvent();
    }
}

void Plugin::stateChangedListener(const rw::kinematics::State& state)
{
    log().info() << "State changed!";
}

void Plugin::buttonDemoEvent(std::string text)
{
    std::cout << "\"" << text << "\" pressed!" << std::endl;
}

void Plugin::homeRobotEvent()
{
    robot->setQ(home,rws_state);
    getRobWorkStudio()->setState(rws_state);
}
