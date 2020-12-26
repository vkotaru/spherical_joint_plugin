cmake_minimum_required(VERSION 2.8.3)
project(spherical_joint_plugin)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_plugins
        )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES spherical_joint_plugin
        #  CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

set (SRCS
        src/spherical_joint_plugin.cc)
add_library(spherical_joint_plugin SHARED ${SRCS})
target_link_libraries(spherical_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(spherical_joint_plugin ${catkin_EXPORTED_TARGETS})
