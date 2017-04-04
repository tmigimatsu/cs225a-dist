CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT (cs225a)

SET(CMAKE_CXX_FLAGS "-std=c++11")

# set project base directory location
SET(SAI2_BASEDIR ${PROJECT_SOURCE_DIR}/../../)
SET(SAI2_INCLUDE_DIR ${SAI2_BASEDIR}/src/)

# query build type. TODO: this is a bad pattern!
IF (CMAKE_BUILD_TYPE STREQUAL Release)
	SET(3RDPARTY_BUILDDIR "build_rel")
ELSE ()
	SET(3RDPARTY_BUILDDIR "build_dbg")
ENDIF () #CMAKE_BUILD_TYPE

# set 3rdParty location
SET(3RDPARTY_BASEDIR ${SAI2_BASEDIR}/3rdParty/)

# include eigen
SET(EIGEN_INCLUDE_DIR ${3RDPARTY_BASEDIR}/eigen/)

# include tinyxml2
SET(TINYXML_INCLUDE_DIR ${3RDPARTY_BASEDIR}/tinyxml/src/tinyxml2-156bc1b/)
SET(TINYXML_LIB_DIR ${3RDPARTY_BASEDIR}/tinyxml/${3RDPARTY_BUILDDIR}/)

# include hiredis
SET(HIREDIS_INCLUDE_DIR ${3RDPARTY_BASEDIR}/hiredis/)
SET(HIREDIS_LIB_DIR ${3RDPARTY_BASEDIR}/hiredis/)

# include json
SET(JSON_INCLUDE_DIR ${3RDPARTY_BASEDIR}/jsoncpp/include/)
SET(JSON_LIB_DIR ${3RDPARTY_BASEDIR}/jsoncpp/${3RDPARTY_BUILDDIR}/src/lib_json/)

# include openGL (required by Chai3d)
FIND_PACKAGE(OpenGL REQUIRED)

# include Chai3d (required by Dynamics3D)
# -- this is required by Chai3d to pick the correct path for OpenGL headers
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
  ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
SET(CHAI3D_INCLUDE_DIR ${3RDPARTY_BASEDIR}/chai/src/chai3d-3.1.1/src/)
SET(CHAI3D_LIB_DIR ${3RDPARTY_BASEDIR}/chai/${3RDPARTY_BUILDDIR}/)

# include Dynamics3d
SET(DYNAMICS3D_INCLUDE_DIR ${3RDPARTY_BASEDIR}/dyn3d/src/DYN3D/src/)
SET(DYNAMICS3D_LIB_DIR ${3RDPARTY_BASEDIR}/dyn3d/${3RDPARTY_BUILDDIR}/)

# include Parser
SET(PARSER_SOURCE ${SAI2_BASEDIR}/src/Parser/URDFToDynamics3d.cpp
	${SAI2_BASEDIR}/src/Parser/URDFToChaiGraphics.cpp)

# include RBDL
SET(RBDL_INCLUDE_DIR
	${3RDPARTY_BASEDIR}/rbdl/src/rbdl-rbdl-849d2aee8f4c/include/
	${3RDPARTY_BASEDIR}/rbdl/src/rbdl-rbdl-849d2aee8f4c/addons/urdfreader/ 
	${3RDPARTY_BASEDIR}/rbdl/${3RDPARTY_BUILDDIR}/include/
	)
#NOTE: For RBDL, apparently debug mode urdf reading does not work!
SET(RBDL_LIB_DIR ${3RDPARTY_BASEDIR}/rbdl/build_rel/)
SET(RBDL_URDF_LIB_DIR ${3RDPARTY_BASEDIR}/rbdl/build_rel/addons/urdfreader/)

# include glfw3
SET(GLFW_INCLUDE_DIR ${3RDPARTY_BASEDIR}/glfw/src/glfw-3.2.1/include/)
SET(GLFW_LIB_DIR ${3RDPARTY_BASEDIR}/glfw/${3RDPARTY_BUILDDIR}/src/)

# include urdf, compiled with application
SET(URDF_INCLUDE_DIR ${SAI2_BASEDIR}/src/Parser/)
SET(URDF_SOURCE ${SAI2_BASEDIR}/src/Parser/urdf/urdfdom/urdf_parser/src/pose.cpp
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom/urdf_parser/src/link.cpp
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom/urdf_parser/src/graphics.cpp
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom/urdf_parser/src/world.cpp
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom/urdf_parser/src/model.cpp
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom/urdf_parser/src/joint.cpp
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h	
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom_headers/urdf_world/include/urdf_world/world.h
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h
		${SAI2_BASEDIR}/src/Parser/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h
		${SAI2_BASEDIR}/src/Parser/urdf/boost_replacement/lexical_cast.h
		${SAI2_BASEDIR}/src/Parser/urdf/boost_replacement/shared_ptr.h
		${SAI2_BASEDIR}/src/Parser/urdf/boost_replacement/printf_console.cpp
		${SAI2_BASEDIR}/src/Parser/urdf/boost_replacement/printf_console.h
		${SAI2_BASEDIR}/src/Parser/urdf/boost_replacement/string_split.cpp
		${SAI2_BASEDIR}/src/Parser/urdf/boost_replacement/string_split.h)

# include Simulation
SET(SIMULATION_SOURCE ${SAI2_BASEDIR}/src/Simulation/SimulationInterface.cpp
	${SAI2_BASEDIR}/src/Simulation/Dynamics3dSimulation.cpp)

# include Graphics
SET(GRAPHICS_SOURCE ${SAI2_BASEDIR}/src/Graphics/GraphicsInterface.cpp
	${SAI2_BASEDIR}/src/Graphics/ChaiGraphics.cpp)

# include Model
SET(MODEL_SOURCE ${SAI2_BASEDIR}/src/Model/ModelInterface.cpp
	${SAI2_BASEDIR}/src/Model/RBDLModel.cpp)

# include Redis client
# SET(REDIS_CLI_SOURCE src/redis/CDatabaseRedisClient.cpp)

# include Timer
SET(TIMER_INCLUDE_DIR ./timer)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES (
	src/
	/usr/local/include
	${SAI2_INCLUDE_DIR}
	${RBDL_INCLUDE_DIR}
	${GLFW_INCLUDE_DIR}
	${URDF_INCLUDE_DIR}
	${TINYXML_INCLUDE_DIR}
	${EIGEN_INCLUDE_DIR}
	${HIREDIS_INCLUDE_DIR}
	${JSON_INCLUDE_DIR}
	${OPENGL_INCLUDE_DIR}
	${CHAI3D_INCLUDE_DIR}
	${DYNAMICS3D_INCLUDE_DIR}
	${TIMER_INCLUDE_DIR}
	)

# Find dependencies
if (CMAKE_SYSTEM_NAME MATCHES Darwin) #mac specific libraries
  FIND_LIBRARY(FOUNDATION_LIBRARY CoreFoundation)
  FIND_LIBRARY(COCOA_LIBRARY Cocoa)
  FIND_LIBRARY(COREVIDEO_LIBRARY CoreVideo)
  FIND_LIBRARY(COREAUDIO_LIBRARY CoreAudio)
  FIND_LIBRARY(AUDIOTOOLBOX_LIBRARY AudioToolbox)
  FIND_LIBRARY(AUDIOUNIT_LIBRARY AudioUnit)
  FIND_LIBRARY(IOKIT_LIBRARY Iokit)
  SET(COREAUDIO_LIBRARIES ${COREAUDIO_LIBRARY} ${AUDIOTOOLBOX_LIBRARY} ${AUDIOUNIT_LIBRARY})
  SET(EXTRA_LIBS ${FOUNDATION_LIBRARY} ${COCOA_LIBRARY} ${COREVIDEO_LIBRARY}
            ${COREAUDIO_LIBRARIES} ${IOKIT_LIBRARY})
else (CMAKE_SYSTEM_NAME MATCHES Linux) # linux specific packages
  SET(EXTRA_LIBS X11 Xrandr Xi Xinerama Xcursor Xxf86vm GL GLU GLEW usb pthread m dl)
endif ()
FIND_LIBRARY(TINYXML_LIB tinyxml2 PATHS ${TINYXML_LIB_DIR} NO_DEFAULT_PATH)
FIND_LIBRARY(HIREDIS_LIB NAMES libhiredis.a hiredis PATHS ${HIREDIS_LIB_DIR} NO_DEFAULT_PATH)
FIND_LIBRARY(JSON_LIB jsoncpp PATHS ${JSON_LIB_DIR} NO_DEFAULT_PATH)
FIND_LIBRARY(CHAI3D_LIB chai3d PATHS ${CHAI3D_LIB_DIR} NO_DEFAULT_PATH)
FIND_LIBRARY(DYNAMICS3D_LIB chai3d-DYN3D PATHS ${DYNAMICS3D_LIB_DIR} NO_DEFAULT_PATH)
FIND_LIBRARY(RBDL_LIB rbdl PATHS ${RBDL_LIB_DIR} NO_DEFAULT_PATH)
FIND_LIBRARY(RBDL_URDF_LIB rbdl_urdfreader PATHS ${RBDL_URDF_LIB_DIR} NO_DEFAULT_PATH)
FIND_LIBRARY(GLFW_LIB glfw3 PATHS ${GLFW_LIB_DIR} NO_DEFAULT_PATH)
SET(ALL_LIBS 
	${TINYXML_LIB}
	${HIREDIS_LIB}
	${JSON_LIB}
	${OPENGL_LIBRARIES}
	${EXTRA_LIBS}
	${CHAI3D_LIB}
	${RBDL_LIB} ${RBDL_URDF_LIB}
	${GLFW_LIB}
	${EXTRA_LIBS}
	${DYNAMICS3D_LIB}
	)

# Create targets
ADD_EXECUTABLE (simulation src/simulation_main.cpp
	${URDF_SOURCE}
	${PARSER_SOURCE}
	${SIMULATION_SOURCE}
	${GRAPHICS_SOURCE}
	${MODEL_SOURCE}
	)
ADD_EXECUTABLE (controller src/controller_main.cpp
	${URDF_SOURCE}
	${PARSER_SOURCE}
	${MODEL_SOURCE}
	)
ADD_EXECUTABLE (visualizer src/visualizer_main.cpp src/uiforce/UIForceWidget.cpp
	${URDF_SOURCE}
	${PARSER_SOURCE}
	${GRAPHICS_SOURCE}
	${MODEL_SOURCE}
	)
ADD_EXECUTABLE (cs225a_test src/test.cpp
	)
TARGET_LINK_LIBRARIES (simulation ${ALL_LIBS})
TARGET_LINK_LIBRARIES (controller ${ALL_LIBS})
TARGET_LINK_LIBRARIES (visualizer ${ALL_LIBS})
TARGET_LINK_LIBRARIES (cs225a_test ${ALL_LIBS})


# Export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${RESOURCE_DIR})
FILE(COPY kuka_iiwa DESTINATION ${RESOURCE_DIR})

