#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/server.h>
#include <osr_planner/PlannerSettingsConfig.h>


#include "Helper.h"
#include "Settings.h"
#include "DynamicVoronoi.h"
#include "Constants.h"
#include "Node2D.h"
#include "Node3D.h"
#include "Visualization.h"
#include "Path.h"
#include "Smoother.h"
#include "CollisionMap.h"
#include "CollisionDetection.h"
#include "Algorithm.h"
#include "AlgorithmStats.h"
#include "Lookup.h"
#include "Heuristics.h"
#include "HASAlgorithm.h"
#include "VoronoiField.h"
#include "PostSmoothing.h"

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

            void reconfigureCallback(osr_planner::PlannerSettingsConfig &config, uint32_t level);


        private:
            void updateSettings();
            ros::NodeHandle n;  
            ros::Publisher pubStart;            
            ros::Publisher pubVoronoi;
            ros::Subscriber subMap;            
            ros::Subscriber subGoal;            
            ros::Subscriber subStart;            
            tf::TransformListener listener;            
            tf::StampedTransform transform;

            dynamic_reconfigure::Server<osr_planner::PlannerSettingsConfig> reconfigureServer;
            dynamic_reconfigure::Server<osr_planner::PlannerSettingsConfig>::CallbackType reconfigureCB;

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
            Path gradientPath = Path(true, true);
            Smoother smoother;
            Heuristics heuristics;
            HASAlgorithm hasAlgorithm;
            CollisionMap collisionMap;
            VoronoiField voronoiField;
            PostSmoothing postSmoother;

            Constants::config collisionLookup[Constants::headings * Constants::positions];
            float * dubinsLookup = NULL;

            void outputAlgoStats(AlgorithmStats& stats);
            void outputAlgoStat(std::string name, FunctionCallStats& stat);

            void validatePath();
            bool validateNode(const Node3D& node);
            
    };
}

#endif // PLANNER_H