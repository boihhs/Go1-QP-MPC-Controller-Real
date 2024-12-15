cmake_minimum_required(VERSION 3.10.1)
project(Go1_cpp)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#check environmental variables
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        unitree_legged_msgs
        )

find_package(Eigen3 REQUIRED)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES rc_sim_oub
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

find_package(OsqpEigen REQUIRED)


# --- Unitree SDK Configuration ---
#   Important: Assumes the Unitree SDK is built in /root/support_files
set(UNITREE_SDK_INCLUDE_DIR /root/support_files/unitree_legged_sdk/include)
set(UNITREE_SDK_LIB_DIR /root/support_files/unitree_legged_sdk/lib)
include_directories(${UNITREE_SDK_INCLUDE_DIR})
link_directories(${UNITREE_SDK_LIB_DIR})

set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

# Declare cpp libraries
# common robot controller every type of robots need
add_library(go1_lib
        src/legKinematics/Go1Kinematics.h
        src/legKinematics/Go1Kinematics.cpp
        src/Go1Params.h
        src/Go1ControlStates.h
        src/utils/Utils.cpp
        src/utils/Utils.h
        src/Go1RobotControl.cpp
        src/Go1RobotControl.h
        src/Go1FilterEKF.cpp
        src/Go1FilterEKF.h
        src/ConvexMpc.cpp
        src/ConvexMpc.h
        )
target_link_libraries(go1_lib ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

# lib of different types of robots
add_library(gazebo_go1_lib
        src/GazeboGo1ROS.h
        src/GazeboGo1ROS.cpp
        )

# add_library(hardware_go1_lib
#       src/Go1HardwareROS.h
#       src/Go1HardwareROS.cpp
#       )

# all robots depends on the go1_lib
target_link_libraries(gazebo_go1_lib PUBLIC go1_lib)
# target_link_libraries(hardware_go1_lib PUBLIC go1_lib ${EXTRA_LIBS})

# Declare a cpp executable for gazebo robot
add_executable(gazebo_go1_ctrl src/MainGazebo.cpp)
target_link_libraries(gazebo_go1_ctrl
        gazebo_go1_lib
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        )

# Declare a cpp executable for hardware robot
# add_executable(hardware_go1_ctrl src/Go1MainHardware.cpp)
# target_link_libraries(hardware_go1_ctrl
#       hardware_go1_lib
#       ${catkin_LIBRARIES}
#       OsqpEigen::OsqpEigen
#       )
