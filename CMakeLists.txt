cmake_minimum_required(VERSION 2.8.3)
project( roboy_plexus )

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)
if(NOT EXISTS "ENV{SOCEDS_DEST_ROOT}")
    message(STATUS "cross compiling for arm")
	# we are cross compiling for arm (make sure to be in embedded_command_shell.sh)
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR armv7l)

    set(compiler_path /usr/bin)
    set(CMAKE_C_COMPILER ${compiler_path}/arm-linux-gnueabihf-gcc-5)
    set(CMAKE_CXX_COMPILER ${compiler_path}/arm-linux-gnueabihf-g++-5)
#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rviz tf tf_conversions visualization_msgs roboy_communication_middleware common_utilities )

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# yaml-cpp
pkg_check_modules( yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities )

include_directories(
		include/
        ${catkin_INCLUDE_DIRS}
)
	#==============================================================================
	# Application
	#==============================================================================

else()
    message(STATUS "cannot cross compile, are you in an embedded_command_shell.sh?!")
endif()
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        darkroom
	    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS
		CommunicationTest
	    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
