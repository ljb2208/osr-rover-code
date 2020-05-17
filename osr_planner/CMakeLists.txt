cmake_minimum_required(VERSION 2.8.3)
project(osr_planner)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    )

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Node2D.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Node3D.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CollisionDetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Settings.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Visualization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Dubins.cpp #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/src/DynamicVoronoi.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BucketedQueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Node2D.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Node3D.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CollisionDetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Settings.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Path.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Vector2D.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Visualization.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Lookup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Gradient.h #Andrew Noske
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Dubins.h #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/include/DynamicVoronoi.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/BucketedQueue.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Point.h #Boris Lau, Christoph Sprunk, Wolfram Burgard

    )
#add_library(osr_planner_lib ${SOURCES} ${HEADERS})

## Declare a catkin package
catkin_package()

## OPEN MOTION PLANNING LIBRARY
find_package(OMPL REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include include)

#add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(osr_planner src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(osr_planner ${catkin_LIBRARIES})
target_link_libraries(osr_planner ${OMPL_LIBRARIES})

#install(TARGETS ${PROJECT_NAME} tf_broadcaster
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
