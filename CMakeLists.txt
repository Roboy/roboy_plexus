cmake_minimum_required(VERSION 3.5)
project(roboy_plexus)

#SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")

SET(CMAKE_BUILD_TYPE Debug)
#if (DEFINED "ENV{SOCEDS_DEST_ROOT}")
#    message(STATUS "cross compiling for arm")
    # we are cross compiling for arm (make sure to be in embedded_command_shell.sh)
#    set(CMAKE_SYSTEM_NAME Linux)
#    set(CMAKE_SYSTEM_PROCESSOR armv7l)

#    set(compiler_path /usr/bin)
#    set(CMAKE_C_COMPILER ${compiler_path}/arm-linux-gnueabihf-gcc-5)
#    set(CMAKE_CXX_COMPILER ${compiler_path}/arm-linux-gnueabihf-g++-5)
    #==============================================================================
    # Dependencies
    #==============================================================================
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs  REQUIRED)
    find_package(sensor_msgs  REQUIRED)
    find_package(roboy_middleware_msgs REQUIRED)
    find_package(roboy_control_msgs REQUIRED)
    find_package(common_utilities REQUIRED COMPONENTS motor_cofig)


#    find_package(catkin REQUIRED COMPONENTS
#            roscpp
#            std_msgs
#            message_generation
##            rviz
##            tf
##            tf_conversions
#            visualization_msgs
#            roboy_middleware_msgs
#            roboy_control_msgs
#            common_utilities
#            )
#    include(findros)
    # Eigen
   # find_package(PkgConfig REQUIRED)
   #  pkg_check_modules(EIGEN3 REQUIRED eigen3)
   #  include_directories(${EIGEN3_INCLUDE_DIRS})
    find_package(eigen3_cmake_module)
    find_package(Eigen3)
    include_directories(${Eigen3_INCLUDE_DIRS})

    # yaml-cpp
    find_package(yaml-cpp REQUIRED yaml-cpp)
    include_directories(${yaml-cpp_INCLUDE_DIRS})

    find_package(tinyxml_vendor REQUIRED)
    find_package(TinyXML REQUIRED)
    include_directories(SYSTEM ${TinyXML_INCLUDE_DIRS})
#    catkin_package(CATKIN_DEPENDS
#            message_runtime
#            roboy_middleware_msgs
#            roboy_control_msgs
#            common_utilities
#            )

    SET(ALT_DEVICE_FAMILY soc_cv_av)
    add_definitions(-D${ALT_DEVICE_FAMILY})
    MESSAGE(STATUS "fpga include dir: /home/missxa/intel/embedded/ip/altera/hps/altera_hps/hwlib/${ALT_DEVICE_FAMILY}")

    include_directories(
            include/roboy_plexus
            ${common_utilities_INCLUDE_DIRS}
#            ${catkin_INCLUDE_DIRS}
            /home/missxa/intel/embedded/ip/altera/hps/altera_hps/hwlib/${ALT_DEVICE_FAMILY}
            /home/missxa/intel/embedded/ip/altera/hps/altera_hps/hwlib/include
            /home/missxa/intel/embedded/ip/altera/hps/altera_hps/hwlib/include/${ALT_DEVICE_FAMILY}
    )
    #==============================================================================
    # Application
    #==============================================================================
    add_executable( roboy_plexus
#              src/test.cpp
             src/main.cpp
             src/interfaces/icebusControl.cpp
             src/interfaces/myoControl.cpp
             src/interfaces/motorControl.cpp
             src/interfaces/i2c.cpp
             src/roboyPlexus.cpp
             src/sensors/A1335.cpp
#             src/control/controlActions.cpp
             src/control/fanControl.cpp
             src/sensors/tle493d_w2b6.cpp
 #            src/utility/UDPSocket.cpp
             src/sensors/BallJoint.cpp
#             ../common_utilities/src/MotorConfig.cpp
            include/roboy_plexus/interfaces/icebusControl.hpp
            # include/roboy_plexus/interfaces/myoControl.hpp
            # include/roboy_plexus/interfaces/motorControl.hpp
            include/roboy_plexus/interfaces/i2c.hpp
            include/roboy_plexus/roboyPlexus.hpp
            include/roboy_plexus/sensors/A1335.hpp
            # include/roboy_plexus/control/controlActions.hpp
            # include/roboy_plexus/control/fanControl.hpp
            include/roboy_plexus/sensors/tle493d_w2b6.hpp
#            common_utilities/include/common_utilities/MotorConfig.hpp
#            include/roboy_plexus/common_utilities/CommonDefinitions.h
            # include/roboy_plexus/interfaces/motorControl.hpp
            include/roboy_plexus/sensors/BallJoint.hpp
            )
    ament_target_dependencies(roboy_plexus rclcpp motor_config sensor_msgs roboy_middleware_msgs roboy_control_msgs yaml-cpp )
    target_link_libraries( roboy_plexus  ${common_utilities_LIBRARIES} ${TinyXML_LIBRARIES} yaml-cpp -lpthread -lrt  )

#    add_executable( automaticRoboyPlexusLauncher src/utility/ROS_MASTER_URI_receiver.cpp src/utility/UDPSocket.cpp)
#    target_link_libraries( automaticRoboyPlexusLauncher -lpthread -lrt ${ros_LIBRARIES} )

    #==============================================================================
    # Installation
    #==============================================================================
    install(TARGETS
            roboy_plexus
#            automaticRoboyPlexusLauncher
            RUNTIME DESTINATION bin
            )


ament_package()
