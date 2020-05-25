#ifndef COLLISION_MAP_H
#define COLLISION_MAP_H

#include <cmath>
#include <chrono>
#include <vector>
#include <math.h>

#include "Settings.h"
#include "Node3D.h"

using namespace std;

namespace OsrPlanner {

    class CMPoint{
        public:
            CMPoint(){};
            CMPoint(float x, float y) {
                this->x = x;
                this->y = y;
            };
        
        float x;
        float y;
    };

    class CollisionMap{
        public:
            CollisionMap();
            
            void setMap(vector<vector<bool>> binMap, int width, int height);
            void setSettings(Settings* settings);

            bool isTraversable(float x, float y, float t);
            bool isTraversable(Node3D* node);

        private:
            vector<vector<bool>> binMap;
            Settings* settings;
            int width;
            int height;

            float vehLength;
            float vehWidth;
            float halfVehLength;
            float halfVehWidth;
            CMPoint vehPoints[4];                    
    };
}


#endif //COLLISION_MAP_H