# cs225a
Repository for class resources: CS225a experimental robotics

Installation Instructions:
1. Install Brew for Mac or have Ubuntu installed or any other distro with the following packages. Also install git. Windows will have to dualboot ubuntu or install a VM.

2. Install Cmake

   Linux: ```sudo apt-get install cmake```
   
   Mac: ```brew install cmake```
   
3. Install Eigen

   Linux:```sudo apt-get install libeigen3-dev```
    
   Mac:```brew install eigen```
    
4. Install TinyXML
 
   Linux:```sudo apt-get install libtinyxml2-dev```
    
   Mac:```brew install tinyxml2```

5. Install JsonCPP

   Linux:```sudo apt-get install libjsoncpp-dev```
    
   Mac:```brew install jsoncpp```
   
6. Install HiRedis Client

   Linux:```sudo apt-get install libhiredis-dev```
    
   Mac:```brew install hiredis```
   
7. Install GLFW + Other things Chai3D Needs

   Linux:
   ```
   sudo apt-get install libglfw3-dev
   sudo apt-get install xorg-dev
   sudo apt-get install freeglut3-dev
   sudo apt-get install libasound2-dev
   sudo apt-get install libusb-1.0-0-dev
   ```
    
   Mac:```brew install glfw3```
   
8. Install Redis-Server

   Linux:```sudo apt-get install redis-server```
    
   Mac:```brew install redis```
   
9. Install Yaml-CPP

   Linux:```sudo apt-get install libyaml-cpp-dev```
    
   Mac:```brew install yaml```
   
10. Clone, Build, Install RBDL
    Download a copy of the repository as a zip from https://bitbucket.org/rbdl/rbdl
    Extract into a folder (should be called rbdl)
    ```
    cd rbdl
    mkdir build 
    cd build
    cmake -DRBDL_BUILD_ADDON_URDFREADER=On -DRBDL_USE_ROS_URDF_LIBRARY=OFF ..
    make -j4
    sudo make install
    ```
   
11. Install Chai

    Download and extract the multiplatform release from : http://www.chai3d.org/download/releases
    Cd into the extracted folder
 
    ```
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
    
12. Extract the Zip of SAI2 Simulation Library given to you and cd into the extracted folder.
    ```
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
    
13. Clone This Repo
    ```
    cd HW0
    mkdir build
    cd build
    cmake ..
    make
    ```
