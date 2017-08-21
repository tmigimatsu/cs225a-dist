cmake_minimum_required(VERSION 3.1)

#Name the project
project(kinova_jaco_driver)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

###(a) Jaco API
SET(KINOVA_API_DIR ${PROJECT_SOURCE_DIR}/KinovaAPI)

FIND_LIBRARY(KINOVA_API_COMM NAMES Kinova.API.CommLayerUbuntu.so PATHS ${KINOVA_API_DIR})
FIND_LIBRARY(KINOVA_API_USBCOMMAND NAMES Kinova.API.USBCommandLayerUbuntu.so PATHS ${KINOVA_API_DIR} NO_DEFAULT_PATH)
SET(KINOVA_API ${KINOVA_API_COMM} ${KINOVA_API_USBCOMMAND})
message(${KINOVA_API})
INCLUDE_DIRECTORIES(${KINOVA_API_DIR}) 

################Initialize the executable#################
#Set the include directories

#Set the compilation flags
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

set (CS225A_COMMON_SOURCE
	${PROJECT_SOURCE_DIR}/../redis/RedisClient.cpp
	${PROJECT_SOURCE_DIR}/../timer/LoopTimer.cpp
)
include_directories (${CS225A_COMMON_HEADERS})

# Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
ADD_EXECUTABLE(kinova_jaco_driver ${CS225A_COMMON_SOURCE} KinovaJacoRedisDriver.cpp)
TARGET_LINK_LIBRARIES(kinova_jaco_driver ${CS225A_COMMON_LIBRARIES} ${KINOVA_API} ${CMAKE_DL_LIBS})

# Set gravity compensation executable
ADD_EXECUTABLE(kinova_jaco_gravity_calibration ${CS225A_COMMON_SOURCE} gravity_calibration_main.cpp)
TARGET_LINK_LIBRARIES(kinova_jaco_gravity_calibration ${CS225A_COMMON_LIBRARIES} ${KINOVA_API} ${CMAKE_DL_LIBS})

# Copy resources
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/kinova_jaco_driver)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY gravity.xml DESTINATION ${APP_RESOURCE_DIR})
