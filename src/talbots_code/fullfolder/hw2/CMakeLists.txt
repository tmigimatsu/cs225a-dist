# create an executable
ADD_EXECUTABLE (redis_test   test.cpp)
ADD_EXECUTABLE (haptic_sim	haptic_sim.cpp)
ADD_EXECUTABLE (force_sim	force_sim.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (redis_test  ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (haptic_sim  ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (force_sim  ${CS327A_COMMON_LIBRARIES})


# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS327A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/hw2)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf world_1_puma.urdf world_2_iiwa.urdf world_3_iiwa.urdf DESTINATION ${APP_RESOURCE_DIR})

