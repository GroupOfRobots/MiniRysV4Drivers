# RobotDrivers

This repo contains basic firmware drivers for robot pheripherials 

## Eclipse IDE setup for master branch
1. Download and install Eclipse C++ IDE.
2. Clone raspberryPi cross compilation tools.
```
git clone git://github.com/raspberrypi/tools.git
```
3. Import project from this repo: File->Import->General->Existing project into workspace, and selest this repo folder.
4. Right click on project in project list three and enter project properties. Properties->C/C++ Build->Settings. 
- In "Cross Settings" set compiler prefix type to "arm-linux-gnueabihf-".
- In the "Cross Settings" set path to the full path to the toolchain: "/home/'hostname'/tools/arm-bcm2708/gcc-linaro-arm-linux- gnueabihf-raspbian-x64/bin/".
- In "Cross GCC Compiler -> Includes" add path "/home/'hostname'/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/include/c++/4.8.3".
- In "Cross GCC Compiler -> Miscellanous" add flags "-c -fmessage-length=0 -pthread".
- In "Cross G++ Compiler -> Dialect" set language standard to "ISO C++11".
- In "Cross G++ Compiler -> Includes" add path "../__GXX_EXPERIMENTAL_CXX0X__". 
- In "Cross G++ Compiler -> Miscellanous" type "-c -fmessage-length=0 -std=c++11 -pthread".
- In "Cross G++ Linker -> Miscellanous" add linker flags "-lpthread".
5. Build project. 

## Setup and building on raspberry4 branch
1. Clone repository and enter raspberry4 branch
2. Perform 'make all' command in Debug/ folder.

## Setup and building on ros2_rasp4 branch
1. Download and setup 'ros2 dashing diademata' and 'colcon'
2. Source ros2:
```
source /opt/ros2/dashing/setup.bash
```
3. Create workspace folder e.g. 'ros2_ws' and enter it
4. Create 'src' folder and enter it
5. Clone repository and enter ros2_rasp4 branch
6. Change folder name from 'RobotDrivers' to 'minirys_drivers' with command:
```
mv RobotDrivers/ minirys_drivers
```
7. Go back to 'ros2_ws' and run following command to build:
```
colcon build --packages-select minirys_drivers
```
8. Source the new setup:
```
. install/setup.bash
```
9. Run selected program:
```
ros2 run minirys_drivers <executable>
```
10. (OPTIONAL) Some executables may require root authorization to run. You can't do 'sudo ros2...'!!! You will have to enter 'sudo su' command.


## Running program on Raspberry PI: 
1. Copy bin file from project Debug folder and scp to raspberryPi. 
2. Run bin file with sudo. 

## TODO
1. Reading parameters from motor drivers
2. Unify I2C library
3. Unify GPIO library 
4. Improve TMP102 class 
5. Add IMU data filtering 
