cmake_minimum_required(VERSION 2.8.3)
project(spherical_joint_plugin)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_plugins
        gazebo_ros
)
find_package(gazebo REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp gazebo_plugins gazebo_ros
        DEPENDS EIGEN3 GAZEBO
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(SRCS
        src/spherical_joint_plugin.cc)

add_library(spherical_joint_plugin SHARED ${SRCS})
target_link_libraries(spherical_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(spherical_joint_plugin ${catkin_EXPORTED_TARGETS})
