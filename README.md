# cs225a
Repository for class resources: CS225a experimental robotics

Installation Instructions:
1. Install Brew for Mac or have Ubuntu installed or any other distro with the following packages. Also install git and hg (mercurial). Windows will have to dualboot ubuntu or install a VM.

2. Install Cmake

   Linux: ```sudo apt-get install cmake```
   
   Mac: ```brew install cmake```
   
3. Install Eigen

   ```sudo apt-get install libeigen3-dev```
    
   ```brew install libeigen3-dev```
    
4. Install TinyXML
 
   ```sudo apt-get install libtinyxml2-dev```
    
   ```brew install libtinyxml2-dev```

5. Install JsonCPP

   ```sudo apt-get install libjsoncpp-dev```
    
   ```brew install libjsoncpp-dev```
   
6. Install HiRedis Client

   ```sudo apt-get install libhiredis-dev```
    
   ```brew install libhiredis-dev```
   
7. Install GLFW

   ```sudo apt-get install libglfw3-dev```
   + Xorg dev?
   + freeglut3-dev?
   + libasound2-dev?
   + sudo apt-get install libusb-1.0-0-dev?
    
   ```brew install libglfw3-dev```
   
8. Install Redis-Server

   ```sudo apt-get install redis-server```
    
   ```brew install redis-server```
   
9. Install Yaml-CPP

   ```sudo apt-get install libyaml-cpp-dev```
    
   ```brew install libyaml-cpp-dev```
   
10. Clone, Build, Install RBDL

    ```
    hg clone https://bitbucket.org/rbdl/rbdl
    cd rbdl
    mkdir build 
    cd build
    cmake ..
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
    make install
    ```
    
12. Extract the Zip of SAI2 Simulation Library given to you and cd into the extracted folder.
    ```
    mkdir build
    cd build
    cmake ..
    make -j4
    make install
    ```
    
13. Clone This Repo
    ```
    cd HW0
    mkdir build
    cd build
    cmake ..
    make
    ```
