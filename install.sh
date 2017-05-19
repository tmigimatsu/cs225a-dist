set -e

# ---------------------------------------
# Install precompiled 3rd party libraries
# ---------------------------------------

if [[ "$OSTYPE" == "linux-gnu" ]]; then
	sudo apt-get install curl cmake libeigen3-dev libtinyxml2-dev libjsoncpp-dev libhiredis-dev libglfw3-dev xorg-dev freeglut3-dev libasound2-dev libusb-1.0-0-dev redis-server
	# Install gcc 5 for Ubuntu 14.04:
	# sudo add-apt-repository ppa:ubuntu-toolchain-r/test
	# sudo apt-get update
	# sudo apt-get install gcc-5 g++-5
	# sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
elif [[ "$OSTYPE" == "darwin"* ]]; then
	brew install cmake eigen redis hiredis tinyxml2 jsoncpp glfw3
fi

mkdir -p external
cd external


# ------------
# Install RBDL
# ------------

curl -L https://bitbucket.org/rbdl/rbdl/get/default.zip -o rbdl-2.5.0.zip
unzip rbdl-2.5.0.zip
mv rbdl-rbdl-849d2aee8f4c rbdl
cd rbdl
if [ ! -f CMakeLists.txt.bkp ]; then
	cp CMakeLists.txt CMakeLists.txt.bkp
	cat <<EOF >> CMakeLists.txt

# ------------------------
# Custom packaging for SAI
#
# Exposes RBDL to find_package(RBDL)
# ------------------------

# Get library binaries
IF (RBDL_BUILD_STATIC)
	SET(EXPORT_TARGET_RBDL rbdl-static)
	SET(EXPORT_TARGET_RBDL_URDF rbdl_urdfreader-static)
ELSE (RBDL_BUILD_STATIC)
	SET(EXPORT_TARGET_RBDL rbdl)
	SET(EXPORT_TARGET_RBDL_URDF rbdl_urdfreader)
ENDIF ()
GET_PROPERTY(RBDL_LIB TARGET \${EXPORT_TARGET_RBDL} PROPERTY LOCATION)
IF (RBDL_BUILD_ADDON_URDFREADER)
	GET_PROPERTY(RBDL_URDF_LIB TARGET \${EXPORT_TARGET_RBDL_URDF} PROPERTY LOCATION)
ENDIF (RBDL_BUILD_ADDON_URDFREADER)

# Export package
EXPORT(TARGETS \${EXPORT_TARGET_RBDL} FILE \${PROJECT_BINARY_DIR}/RBDLTargets.cmake)
EXPORT(PACKAGE RBDL)

# Set config variables
SET(CONF_INCLUDE_DIRS
	\${CMAKE_CURRENT_SOURCE_DIR}/..
	\${CMAKE_CURRENT_SOURCE_DIR}/include
	\${CMAKE_CURRENT_BINARY_DIR}/include
)
SET(CONF_LIBRARY \${RBDL_LIB})
SET(CONF_URDF_LIBRARY \${RBDL_URDF_LIB})

# Write config file
CONFIGURE_FILE(RBDLConfig.cmake.in "\${PROJECT_BINARY_DIR}/RBDLConfig.cmake" @ONLY)
INCLUDE(CMakePackageConfigHelpers)
WRITE_BASIC_PACKAGE_VERSION_FILE(
	"\${CMAKE_CURRENT_BINARY_DIR}/RBDLConfigVersion.cmake"
	VERSION \${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)
EOF
	cat <<EOF > RBDLConfig.cmake.in
# ------------------------
# Custom packaging for SAI
#
# Exports RBDL_INCLUDE_DIRS, RBDL_LIB, and RBDL_URDF_LIB
# ------------------------

get_filename_component(RBDL_CMAKE_DIR "\${CMAKE_CURRENT_LIST_FILE}" PATH)
set(RBDL_INCLUDE_DIR "@CONF_INCLUDE_DIRS@")
set(RBDL_LIBRARIES "@CONF_LIBRARY@")
set(RBDL_URDFREADER_LIBRARIES "@CONF_URDF_LIBRARY@")
if(NOT TARGET rbdl AND NOT RBDL_BINARY_DIR)
  include("\${RBDL_CMAKE_DIR}/RBDLTargets.cmake")
endif()
EOF
fi

mkdir -p build_rel
cd build_rel
cmake -DCMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON -DRBDL_USE_ROS_URDF_LIBRARY=OFF ..
make -j4
cd ../..


# --------------
# Install Chai3d
# --------------

# TODO: Make this a git submodule
git clone https://github.com/chai3d/chai3d chai3d
# curl -L http://www.chai3d.org/download/chai3d-3.2.0-CMake.zip -o chai3d-3.2.0.zip
# unzip chai3d-3.2.0.zip
# mv chai3d-3.2.0 chai3d
cd chai3d
mkdir build_rel
cd build_rel
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
cd ../..

cd ..


# --------------------
# Build SAI Simulation
# --------------------

cd sai2-simulation
mkdir -p build_rel
cd build_rel
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
cd ../..


# ----------------
# Build SAI Common
# ----------------

cd sai2-common
git submodule update --init --recursive
mkdir -p build_rel
cd build_rel
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
cd ../..

