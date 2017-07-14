# create an executable
SET(EXECUTABLE_NAME sawyer_hold_pos)
ADD_EXECUTABLE (${EXECUTABLE_NAME}
        ${CS225A_COMMON_SOURCE} 
        sawyer_hold_pos_main.cpp
)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXECUTABLE_NAME}
        ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/${EXECUTABLE_NAME})
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY sawyer.urdf world.urdf DESTINATION ${APP_RESOURCE_DIR})
