
cmake_minimum_required (VERSION 2.8.12)
project (CS225A)
SET(CMAKE_CXX_FLAGS "-std=c++11")

# set common source
set (CS225A_COMMON_SOURCE
	${PROJECT_SOURCE_DIR}/src/redis/RedisClient.h
	${PROJECT_SOURCE_DIR}/src/timer/LoopTimer.h)
include_directories(${PROJECT_SOURCE_DIR}/src)

# set common dependencies
# - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI2-SIMULATION. TODO: this should not be needed. something fishy.
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - glfw3
find_package(glfw3 REQUIRED)
find_library(GLFW_LIBRARY glfw)

set(CS225A_COMMON_LIBRARIES
	${CHAI3D_LIBARIES}
	${SAI2-COMMON_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	)
