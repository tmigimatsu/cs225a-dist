# create an executable
ADD_EXECUTABLE (hw0 hw0.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw0
        ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/hw0)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY RRPbot.urdf world.urdf DESTINATION ${APP_RESOURCE_DIR})