
cmake_minimum_required(VERSION 2.8.3)
project(tuw_route)

add_compile_options(-std=c++1y) ## for c++ 2011

SET(CMAKE_BUILD_TYPE Release)

MESSAGE( STATUS "CMAKE_BUILD_TYPE:              " ${CMAKE_BUILD_TYPE} )
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -fno-strength-reduce -pipe ")# -Og
set(CMAKE_CSS_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -pipe -O3")

if(CMAKE_BUILD_TYPE MATCHES Release)
MESSAGE( STATUS "CMAKE_CSS_FLAGS_RELEASE:       " ${CMAKE_CSS_FLAGS_RELEASE} )
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES Debug)
MESSAGE( STATUS "CMAKE_CXX_FLAGS_DEBUG:         " ${CMAKE_CXX_FLAGS_DEBUG} )
endif(CMAKE_BUILD_TYPE MATCHES Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tuw_geometry
    tuw_route_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tuw_route
  
  CATKIN_DEPENDS roscpp rospy tuw_geometry
  DEPENDS Boost OpenCV
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library ( tuw_route
  src/waypoint/waypoint.cpp
  src/route.cpp
  src/route_manager.cpp
)

add_dependencies(tuw_route ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( tuw_route ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${OpenCV_LIBS})



install(TARGETS
  tuw_route
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
