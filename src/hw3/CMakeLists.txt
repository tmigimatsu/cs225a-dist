# create an executable
ADD_EXECUTABLE (hw3
	${CS225A_COMMON_SOURCE}
	hw3.cpp
)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw3
        ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/hw3)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf world.urdf DESTINATION ${APP_RESOURCE_DIR})
