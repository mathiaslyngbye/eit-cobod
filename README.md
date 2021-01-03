# EiT COBOD
Primary practical of the 2020 EiT course.

## Usage
After compiling, the plugin will be in ``./libs/``. 
Do not edit the ``RobWorkStudio.ini`` in the workcell directory.
Instead, either run ``./workcell/generate_ini.sh`` to generate the init file
with absolute path to plugin. Alternately, run ``./start.sh`` to automatically
generate the init file, run RobWorkStudio, and load selected Workcell and 
plugin. Make both scripts executable (``chmod +x``) as needed.

### QtCreator
In order for it to work smoothly with QtCreator, go to 
``Project > Build & Run > Run`` and specify the following:

| Setting | Argument |
|---|---|
| Exectuable | %{sourceDir}/start.sh |
| Working directory | %{sourceDir} |

### Various notes
Following are some notes about how the system works so far. Practical uses of below may be found in functions ```RunRobotControl```, ```RunRobotMimic```, and ```createPathRRTConnect```.
* On startup, run ```Connect``` before doing anything else. 
* If robot position is needed (it mostly is), run ```Start robot mimic``` to enable the digital twin to read values from the robot. 
* When RobWorkStudio state is updated, it calls the ```stateChangedListener(...)``` which in turn updates the global ```rws_state``` variable. Never alter ```rws_state```. If you need the current state, make a copy (i.e. ```rw::kinematics::State tmp_state = rws_state;```)
* All ur_rtde move-to-position commands require values for acceleration and velocity. I made a wrapper (```addMove(...)```) for this to avoid saving static version of these. In practice, this means that we save _only_ joint positions as ```std::vector<double> q = {q1, q2, ..., q5}```, and later add acceleration and velocity before pushing values to a path, e.g. ```std::vector<double> gripMove = addMove(gripQ, 0.2, 0.2);```.
* Function ```addMove()``` uses _optional_ arguments, i.e. if no velocity or acceleration is specified, it defaults to 0.5 for both.
* To make new buttons, be aware that they should be added 3 places; in the header (i.e. ```_btn0```), in the constructor, and in the ```clickEvent``` handler function. Just copy and modify what is already in these places for the millions of buttons already added.
* If you need to plan a route, you may add a button and try to mimic what I did in the ```RunHomeRobot``` function. Be aware that if it is not run as its own thread, it hijacks the main thread, and the display won't work for the duration. This is perfectly okay for testing, though. I will momentarily re-add the printQ function (it got butchered during the great debug session), so you can enable teach mode, got to a location, print Q values, and manually save them.

## Installation/prerequisites
### Qt
```bash
sudo apt install qtbase5-dev
```
### RobWork
Add sdurobotics/robwork to apt ppa repositories.
```bash
sudo add-apt-repository ppa:sdurobotics/robwork
sudo apt-get update
```
Install needed packages with the following command (Note: RobWorkHardware is no longer required. We bypass this by using ur_rtde directly).
```bash
sudo apt install libsdurw-all-dev
sudo apt install libsdurws-all-dev
```

Compiling projects with robwork may need the 'libassimp' package, install it with:
```bash
sudo apt install libassimp-dev
```

### UR RTDE
Add sdurobotics/ur-rtde to apt ppa repositories.
```bash
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
```

Install ur_rtde library.
```bash
sudo apt install librtde librtde-dev
```

### Point Cloud Library
Install pcl.
```bash
sudo apt install libpcl-dev
```

## Network configuration
1. Read robot IP and DNS from tablet interface. 
2. Configure the wired IPv4 connection:
   * IPv4 Method: Manual
   * Addresses: Address NOT the same subnet as robot (i.e. if robot is 192.168.0.212, your PC may be 192.168.0.250). Netmask matching the robot.
   * DNS: Matching robot; likely 0.0.0.0.
3. When done configuring restart your wired connection.
