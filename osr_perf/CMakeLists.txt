cmake_minimum_required(VERSION 2.8.3)
project(osr_perf)
find_package(catkin REQUIRED COMPONENTS
  osr_msgs
  roscpp
  geometry_msgs
  sensor_msgs
  serial
)

#add_message_files(DIRECTORY msg)
#generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs osr_msgs serial)

# include boost
find_package(Boost REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_definitions("-std=c++0x -Wall")


add_executable(osr_mc_node src/osr_mc_node.cpp src/motorcontrollers.cpp src/roboclaw.cpp src/robot.cpp)
target_link_libraries(osr_mc_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME} src/osr_node.cpp src/motorcontrollers.cpp src/roboclaw.cpp src/robot.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)
