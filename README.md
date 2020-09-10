# EiT COBOD
Primary practical of the 2020 EiT course.

## Prerequisites

### Install UR RTDE library
```bash
sudo add-apt-repository ppa:sdurobotics/ur-rtde
```
```bash
sudo apt-get update
```
```bash
sudo apt install librtde librtde-dev
```

### Configure network
1. Read robot IP and DNS from tablet interface. 
2. Configure the wired IPv4 connection:
   * IPv4 Method: Manual
   * Addresses: Address NOT the same subnet as robot (i.e. if robot is 192.168.0.212, your PC may be 192.168.0.250). Netmask matching the robot.
   * DNS: Matching robot; likely 0.0.0.0.
3. When done configuring, if it doesn't work - restart your wired connection.
