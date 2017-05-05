# create an executable
ADD_EXECUTABLE (kuka_hold_pos_controller controller_main.cpp)
ADD_EXECUTABLE (kuka_hold_pos_simulation simulation_main.cpp)
ADD_EXECUTABLE (kuka_hold_pos_visualization visualizer_main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (kuka_hold_pos_controller ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (kuka_hold_pos_simulation ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (kuka_hold_pos_visualization ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/kuka_hold_pos)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf world.urdf DESTINATION ${APP_RESOURCE_DIR})
