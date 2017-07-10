PROJECT(roboy_dynamics C CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g ")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

#==============================================================================
# Dependencies
#==============================================================================
# ros
find_package(catkin REQUIRED COMPONENTS roboy_communication_middleware pcl_ros roscpp std_msgs common_utilities)
catkin_package( CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities INCLUDE_DIRS include)
include_directories(${catkin_INCLUDE_DIRS} include ../common_utilities/include)

# Eigen
find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

#==============================================================================
# Application
#==============================================================================

add_executable(PaBiRoboy_inverse_kinematics src/PaBiRoboy_inverse_kinematics.cpp)
add_dependencies(PaBiRoboy_inverse_kinematics roboy_communication_middleware_gencpp) 
target_link_libraries(PaBiRoboy_inverse_kinematics ${catkin_LIBRARIES} rviz_visualization)

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        PaBiRoboy_inverse_kinematics
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
