################Initialize the executable#################

# Set all the sources required for the executables
set (PUMA_DRIVER_SOURCE
	RobotCom.cpp
	unitMsg.cpp
	RedisDriver.cpp
)
set (PUMA_CONTROLLER_SOURCE DemoController.cpp)

# Create the executables
add_executable (puma_driver ${CS225A_COMMON_SOURCE} ${PUMA_DRIVER_SOURCE})
add_executable (puma_demo ${CS225A_COMMON_SOURCE} ${PUMA_CONTROLLER_SOURCE})

# Link the required libraries
target_link_libraries (puma_driver ${HIREDIS_LIBRARY})
target_link_libraries (puma_demo ${CS225A_COMMON_LIBRARIES})

set (RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
file (MAKE_DIRECTORY ${RESOURCE_DIR})
set (APP_RESOURCE_DIR ${RESOURCE_DIR}/puma_demo)
file (MAKE_DIRECTORY ${APP_RESOURCE_DIR})
file (COPY puma.urdf world.urdf DESTINATION ${APP_RESOURCE_DIR})
