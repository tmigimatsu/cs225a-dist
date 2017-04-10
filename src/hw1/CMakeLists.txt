# create an executable
ADD_EXECUTABLE (hw1 hw1.cpp)
SET(CMAKE_CXX_FLAGS "-Wno-ignored-attributes -Wno-deprecated-declarations")

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw1
        ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/hw1)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf world.urdf meshes DESTINATION ${APP_RESOURCE_DIR})
