cmake_minimum_required(VERSION 3.0.2)
project(softhands_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES softhands_plugin
   CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs
   DEPENDS roscpp gazebo_ros
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/softhands_plugin.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} 
   ${GAZEBO_LIBRARIES}
 )

## Declare a C++ library
add_library(softhands_synergy_plugin
   src/softhands_synergy_plugin.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(softhands_synergy_plugin
   ${catkin_LIBRARIES} 
   ${GAZEBO_LIBRARIES}
 )

## Declare a C++ library
add_library(softhands_mimic_plugin
   src/softhands_mimic_plugin.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(softhands_mimic_plugin
   ${catkin_LIBRARIES} 
   ${GAZEBO_LIBRARIES}
 )

## Declare a C++ library
add_library(softhands_mimic_two_plugin
   src/softhands_mimic_two_plugin.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(softhands_mimic_two_plugin
   ${catkin_LIBRARIES} 
   ${GAZEBO_LIBRARIES}
 )

## Declare a C++ library
add_library(softhands_mimic_two_plugin_qb
   src/softhands_mimic_two_plugin_qb.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(softhands_mimic_two_plugin_qb
   ${catkin_LIBRARIES} 
   ${GAZEBO_LIBRARIES}
 )