/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
*/

//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>

// #include "constants.h"
// #include "planner.h"
#include "Planner.h"


int main(int argc, char** argv) {

  ROS_INFO("Starting OSR Planning node");

  ros::init(argc, argv, "osr_planner");

  OsrPlanner::Planner planner;
  planner.plan();

  ros::spin();

  ROS_INFO("OSR Planning node exiting");
  return 0;
}
