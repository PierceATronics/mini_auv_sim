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



