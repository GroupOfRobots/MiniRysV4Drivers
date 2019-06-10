# RobotDrivers

This repo contains basic firmware drivers for robot pheripherials 

## Eclipse IDE setup
1. Download and install Eclipse C++ IDE.
2. Clone raspberryPi cross compilation tools.
```
git clone git://github.com/raspberrypi/tools.git
```
3. Import project from this repo: File->Import->General->Existing project into workspace, and selest this repo folder.
4. Right click on project in project list three and enter project properties. Properties->C/C++ Build->Settings. In Cross compiler prefix type "arm-linux-gnueabihf-". In the "Cross compiler path" field type the full path to the toolchain: "/home/'hostname'/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/". In "Cross GCC Compiler -> Includes" add path "/home/'hostname'/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/include/c++/4.8.3". In "Cross G++ Compiler -> Includes" add path "../__GXX_EXPERIMENTAL_CXX0X__". In "Cross G++ Compiler -> Includes" type "-c -fmessage-length=0 -std=c++11".
5. Build project. 

## Running program on Raspberry PI: 
1. Copy bin file from project Debug folder and scp to raspberryPi. 
2. Run bin file with sudo. 

## TODO
1. Reading parameters from motor drivers
2. Unify I2C library
3. Unify GPIO library 
4. Improve TMP102 class 
5. Add IMU data filtering 
