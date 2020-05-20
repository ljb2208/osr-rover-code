#ifndef HEURISTICS_H
#define HEURISTICS_H

#include <cmath>
#include <algorithm>
#include <vector>
#include <tuple>
#include <queue>

#include "Settings.h"
#include "Node3D.h"

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
            void calculate2DCostsNew(int goalX, int goalY);
            void setSettings(Settings* settings);
            void setMap(vector<vector<bool>> map, int width, int height);

        private:
            void publish2DPolicy();

            bool canExplore(int x, int y);

            Settings* settings;            
            vector<vector<float>> twoDCosts;            
            vector<vector<int>> twoDPolicy;
            vector<vector<bool>> map;

            int height;
            int width;


            ros::NodeHandle n;
            ros::Publisher pubNodes2D;
            geometry_msgs::PoseArray poses2D;
            
    };
}

#endif // HEURISTICS_H