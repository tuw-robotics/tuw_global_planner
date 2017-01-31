cmake_minimum_required(VERSION 2.8.3)
project(tuw_route_msgs)

add_compile_options(-std=c++1y) ## for c++ 2011

SET(CMAKE_BUILD_TYPE Release)

MESSAGE( STATUS "CMAKE_BUILD_TYPE:              " ${CMAKE_BUILD_TYPE} )
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -fno-strength-reduce -pipe ")# -Og
set(CMAKE_CSS_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -pipe -Ofast")

if(CMAKE_BUILD_TYPE MATCHES Release)
MESSAGE( STATUS "CMAKE_CSS_FLAGS_RELEASE:       " ${CMAKE_CSS_FLAGS_RELEASE} )
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES Debug)
MESSAGE( STATUS "CMAKE_CXX_FLAGS_DEBUG:         " ${CMAKE_CXX_FLAGS_DEBUG} )
endif(CMAKE_BUILD_TYPE MATCHES Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    geometry_msgs
)


## Generate messages in the 'msg' folder
 add_message_files(
  DIRECTORY msg
  FILES
  Waypoint.msg
  Route.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  message_generation
)