# `mini_auv_sim` : Mini Autonomous Underwater Vehicle Simulator

Welcome to the Mini AUV Simulator! This AUV simulator is under-development to be used by SDSU Mechatronics as a testing platform for the 2021 Robosub competition. This repo is under heavy development! The simulation environment is built in the open source, high-fidelity robot simulator Gazebo.

## Requirements
* [cmake](https://cmake.org/)
* [Gazebo](http://gazebosim.org/)
  * [Ignition](https://ignitionrobotics.org/) - Should be already included with Gazebo
  * [Protobuf](https://developers.google.com/protocol-buffers) - Should also be installed with Gazebo
  
* [socat](https://linux.die.net/man/1/socat) - Used for serial port emulation
* [Python3](https://www.python.org/downloads/) - Optional: Used for demo purposes
  * [pyserial](https://pypi.org/project/pyserial/)

## Installation

Clone this repository
```
git clone https://github.com/PierceATronics/mini_auv_sim.git
```

Make a build folder and build the project.
```
cd mini_auv_sim
mkdir build && cd build
cmake ..
make
```

## Run Simulator (for SDSU Mechatronics)

In terminal 1, from the repos home directory, launch Gazebo. The Pico mini auv robot will need to be manually spawned within the Gazebo GUI environment.
```
source setup.sh
gazebo worlds/pool_testing.world
```

In terminal 2, navigate to the `scripts` directory in the repo and run the following script to create the virutal serial ports (requires sudo privledges)
```
bash launch_virtual_ports
```

To interface with the simulated AUV, refer to the Mechatronics 2021 repo [MechatronicRobosub2021](https://github.com/DOCgould/MechatronicsRobosub2021)







