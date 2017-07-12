cmake_minimum_required (VERSION 3.1)

#Name the project
project (kuka_iiwa_driver)

#Make sure the generated makefile is not shortened
set (CMAKE_VERBOSE_MAKEFILE ON)

cmake_minimum_required(VERSION 3.1)


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
set (KUKA_DRIVER_SOURCE
	LBRRedisDriver.cpp
	LBRRedisClient.cpp
)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable (${PROJECT_NAME} ${CS225A_COMMON_SOURCE} ${KUKA_DRIVER_SOURCE})

target_link_libraries(${PROJECT_NAME} ${CS225A_COMMON_LIBRARIES})










# KUKA IIWA FRI SDK
message(${CMAKE_CURRENT_SOURCE_DIR})
set(FRI_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/FRI-Client-SDK)
set(FRI_INCLUDE_DIR ${FRI_BASE_DIR}/include)
find_library(FRI_LIBRARIES FRIClient PATHS ${FRI_BASE_DIR}/lib)
# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O2 -Wall -Wextra -pedantic -std=c++11 -fPIC -fopenmp")
# add_definitions(-DHAVE_SOCKLEN_T -DPB_SYSTEM_HEADER=\"pb_syshdr.h\" -DHAVE_STDINT_H -DHAVE_STDDEF_H -DHAVE_STDBOOL_H -DHAVE_STDLIB_H -DHAVE_STRING_H)

# KUKA LBR Dynamics
if (USE_KUKA_LBR_DYNAMICS)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/KukaLBRDynamics)
    option(USE_KUKA_RBDL    "USE_KUKA_RBDL"     OFF)
    find_package (KukaLBRDynamics REQUIRED)
    include_directories(${KukaLBRDynamics_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${KukaLBRDynamics_LIBRARIES})
    add_definitions(-DUSE_KUKA_LBR_DYNAMICS)

    FIND_LIBRARY(TINYXML_LIBRARIES tinyxml2)
    target_link_libraries(${PROJECT_NAME} ${TINYXML_LIBRARIES})
endif(USE_KUKA_LBR_DYNAMICS)

# # - hiredis
# find_library(HIREDIS_LIBRARY hiredis)

# # - jsoncpp
# if (CMAKE_SYSTEM_NAME MATCHES Darwin)
#     find_library(JSONCPP_LIBRARY jsoncpp)
# elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
#     find_package(PkgConfig REQUIRED)
#     pkg_check_modules(JSONCPP jsoncpp)
#     include_directories(${JSONCPP_INCLUDE_DIRS})
#     link_directories(${JSONCPP_LIBRARY_DIRS})
#     set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
# endif ()

# # - eigen3
# find_package(Eigen3 REQUIRED)

# # - RBDL
# FIND_LIBRARY(RBDL_LIBRARIES rbdl)
# FIND_LIBRARY(RBDL_URDFREADER_LIBRARIES rbdl_urdfreader)

# Filters
SET(FILTERS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/filters/)

# # create headers list
# set(kuka_iiwa_redis_driver_INCLUDE_DIRS ${FRI_INCLUDE_DIRS}
#                                         ${FILTERS_INCLUDE_DIR}
#                                         ${EIGEN3_INCLUDE_DIR}
#                                         ${JSONCPP_INCLUDE_DIRS}
# )

# # create libraries list
# set(kuka_iiwa_redis_driver_LIBRARIES    ${FRI_LIBRARIES}
#                                         ${HIREDIS_LIBRARY}
#                                         ${JSONCPP_LIBRARY}
#                                         ${RBDL_LIBRARIES} ${RBDL_URDFREADER_LIBRARIES}
#                                         -lrt
# )

include_directories(${FRI_INCLUDE_DIR} ${FILTERS_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${CS225A_COMMON_LIBRARIES} ${FRI_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
# SET(RESOURCE_DIR resources)
# FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
# FILE(COPY model/kuka_iiwa.urdf DESTINATION ${RESOURCE_DIR})

