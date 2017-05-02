# create an executable
#ADD_EXECUTABLE (hw2-p1 p1-main.cpp)
#ADD_EXECUTABLE (hw2-p2 p2-main.cpp)
#ADD_EXECUTABLE (hw2-p3 p3-main.cpp)
ADD_EXECUTABLE (redis_test   test.cpp)
ADD_EXECUTABLE (haptic_sim	haptic_sim.cpp)
ADD_EXECUTABLE (force_sim	force_sim.cpp)

# and link the library against the executable
#TARGET_LINK_LIBRARIES (hw2-p1 ${CS327A_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (hw2-p2 ${CS327A_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (hw2-p3 ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (redis_test  ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (haptic_sim  ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (force_sim  ${CS225A_COMMON_LIBRARIES})


# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/talbots_code)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY Puma.urdf fullfolder/hw2/world_1_puma.urdf DESTINATION ${APP_RESOURCE_DIR})
