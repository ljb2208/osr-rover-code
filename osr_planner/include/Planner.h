#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "Helper.h"
#include "Settings.h"
#include "DynamicVoronoi.h"
#include "Constants.h"
#include "Node2D.h"
#include "Node3D.h"
#include "Visualization.h"
#include "Path.h"
#include "Smoother.h"
#include "CollisionDetection.h"
#include "Algorithm.h"
#include "Lookup.h"

namespace OsrPlanner {
    class Planner {
        public:
            Planner();
            ~Planner();
            void initializeLookups();
            void setMap(const nav_msgs::OccupancyGrid::Ptr map);
            void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
            void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
            void plan();

        private:
            ros::NodeHandle n;  
            ros::Publisher pubStart;            
            ros::Publisher pubVoronoi;
            ros::Subscriber subMap;            
            ros::Subscriber subGoal;            
            ros::Subscriber subStart;            
            tf::TransformListener listener;            
            tf::StampedTransform transform;

            nav_msgs::OccupancyGrid::Ptr grid;  
            geometry_msgs::PoseWithCovarianceStamped start;  
            geometry_msgs::PoseStamped goal;      
            bool validStart = false;  
            bool validGoal = false;

            bool manualMode = true;

            Settings settings;
            DynamicVoronoi voronoiDiagram;
            CollisionDetection configurationSpace = CollisionDetection(&settings);
            Visualization visualization;
            Path path;
            Path smoothedPath = Path(true);
            Smoother smoother;

            Constants::config collisionLookup[Constants::headings * Constants::positions];
            float * dubinsLookup = NULL;
    };
}

#endif // PLANNER_H