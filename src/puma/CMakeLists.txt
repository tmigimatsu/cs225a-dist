################Initialize the Cmake Defaults#################

cmake_minimum_required (VERSION 3.1)

#Name the project
project (puma_driver)

#Make sure the generated makefile is not shortened
set (CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the executable#################

#Set the compilation flags
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

set (CS225A_COMMON_SOURCE
	${PROJECT_SOURCE_DIR}/../redis/RedisClient.cpp
	${PROJECT_SOURCE_DIR}/../timer/LoopTimer.cpp
)
include_directories (${PROJECT_SOURCE_DIR}/..)

#Set all the sources required for the library
set (PUMA_DRIVER_SOURCE
	RobotCom.cpp
	unitMsg.cpp
	RedisDriver.cpp
)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable (puma_driver ${CS225A_COMMON_SOURCE} ${PUMA_DRIVER_SOURCE})

target_link_libraries(puma_driver ${CS225A_COMMON_LIBRARIES})
