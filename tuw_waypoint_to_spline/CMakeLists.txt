
cmake_minimum_required(VERSION 2.8.3)
project(tuw_waypoint_to_spline)

add_compile_options(-std=c++1y) ## for c++ 2011

SET(CMAKE_BUILD_TYPE Debug)

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
    dynamic_reconfigure 
    tuw_spline_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  tuw_path_to_spline
  tuw_path_to_spline_node
  
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  dynamic_reconfigure 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_library ( tuw_waypoint_to_spline 
  src/tuw_waypoint_to_spline.cpp 
)
target_link_libraries( tuw_waypoint_to_spline ${catkin_LIBRARIES} )

add_executable       ( tuw_waypoint_to_spline_node src/tuw_waypoint_to_spline_node.cpp )
target_link_libraries( tuw_waypoint_to_spline_node tuw_waypoint_to_spline ${catkin_LIBRARIES} ${Boost_LIBRARIES}  )


install(TARGETS 
  tuw_waypoint_to_spline 
  tuw_waypoint_to_spline_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
