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

## Prerequisites
### Qt
```bash
sudo apt install qtbase5-dev
```
### RobWork (And colliding packages)
Add sdurobotics/robwork to apt ppa repositories.
```bash
sudo add-apt-repository ppa:sdurobotics/robwork
sudo apt-get update
```
Install packages with 
```bash
sudo apt install libsdurw-all-dev
sudo apt install libsdurws-all-dev
sudo apt install libsdurwhw-all-dev
sudo apt install libsdurwsim-all-dev
```

If these packages collide with URRTDE, the installation fails.
Overwrite colliding packages by
``` bash
sudo dpkg -i --force-overwrite /var/cache/apt/archives/sdurwhw-cmake1.1_1.1.11-2_amd64.deb
sudo dpkg -i --force-overwrite /var/cache/apt/archives/libsdurwhw-universalrobots-rtde1.1_1.1.11-2_amd64.deb
```

Compiling projects with robwork may need the 'libassimp' package, install it with:
```bash
sudo apt install libassimp-dev
```

### Configure network
1. Read robot IP and DNS from tablet interface. 
2. Configure the wired IPv4 connection:
   * IPv4 Method: Manual
   * Addresses: Address NOT the same subnet as robot (i.e. if robot is 192.168.0.212, your PC may be 192.168.0.250). Netmask matching the robot.
   * DNS: Matching robot; likely 0.0.0.0.
3. When done configuring, if it doesn't work - restart your wired connection.
