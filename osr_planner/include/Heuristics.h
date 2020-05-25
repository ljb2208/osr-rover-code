#ifndef HEURISTICS_H
#define HEURISTICS_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include <cmath>
#include <algorithm>
#include <vector>
#include <tuple>
#include <queue>

#include "Settings.h"
#include "Node3D.h"
#include "Constants.h"
#include "Helper.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


using namespace std;

namespace OsrPlanner {
    
    class Heuristics
    {
        struct HCell {
            int x;
            int y;            
            int cost;
            int policy;
        };
  
        struct CompareHCells {        
            bool operator()(const HCell& lhs, const HCell& rhs) const {
                return lhs.cost > rhs.cost;
            }
        };

        public:
            Heuristics();
            void calculate(int height, int width);
            void calculate2DCosts(int goalX, int goalY);
            void calculateRSCosts();
            void setSettings(Settings* settings);
            void setMap(vector<vector<bool>> map, int width, int height);
            float getHeuristicValue(float startX, float startY, float startT, float goalX, float goalY, float goalT);

        private:            
            float get2DCost(float startX, float startY, float goalX, float goalY);
            float getRSDistance(float startX, float startY, float startT, float goalX, float goalY, float goalT);
            void publish2DPolicy();
            void loadRSCosts();
            string getRSCostFileName();

            bool canExplore(int x, int y);

            Settings* settings;            
            vector<vector<float>> twoDCosts;            
            vector<vector<int>> twoDPolicy;
            vector<vector<bool>> map;

            float* rsCosts = nullptr;

            int height;
            int width;

            float turnRadius;

            ros::NodeHandle n;
            ros::Publisher pubNodes2D;
            geometry_msgs::PoseArray poses2D;
            
    };
}

#endif // HEURISTICS_H