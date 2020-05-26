#ifndef SETTINGS_H
#define SETTINGS_H

#include <string>
#include <osr_planner/PlannerSettingsConfig.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace OsrPlanner {

    class Settings{
        public:
            Settings();

            void setFromConfig(osr_planner::PlannerSettingsConfig &config);
            void setMapInfo(float resolution, geometry_msgs::Pose origin);

            float getCellSize(){return cellSize;};
            bool getManualMode(){return manualMode;}

            float getPenaltyTurning() { return penaltyTurning;}
            float getPenaltyReversing() { return penaltyReversing;}
            float getPenaltyCOD() { return penaltyCOD;}
            float getDubinsShotDistance() { return dubinsShotDistance;}
            float getDubinsWidth() { return dubinsWidth; }
            float getDubinsStepSize() { return dubinsStepSize; }
            bool getDubinsEnabled() { return dubins; }
            bool getDubinsShotEnabled() { return dubinsShot; }
            bool getTwoDEnabled() { return twoD; }
            bool getDubinsLookup() { return dubinsLookup; }

            float getLength() { return length; }
            float getWidth() { return width; }
            float getBloating() { return bloating; }
            float getTurningRadius() { return turningRadius; }
            float getMinRoadWidth() { return minRoadWidth; }

            bool getReverseEnabled() { return reverseEnabled; }
            bool getVisualizationEnabled() { return visualizationEnabled; }
            bool getVisualization2DEnabled() { return visualization2DEnabled; }
            float getVisualizationDelay() { return visualizationDelay; }

            int getIterations() { return iterations; }

            int getBBSize() { return bbSize; }

            std::string getRSCostFilePath() { return rsCostFilePath; }
            float getRSExpansionFactor() { return rsExpansionFactor; }

            float getVoronoiMaxDistance() { return voronoiMaxDistance; }
            float getVoronoiAlpha() { return voronoiAlpha; }
            float getVoronoiTheta() { return voronoiTheta; }

            float getSmoothAlpha() { return smoothAlpha; }
            float getSmoothVoronoi() { return smoothVoronoi; }
            float getSmoothObstacle() { return smoothObstacle; }
            float getSmoothSmooth() { return smoothSmooth; }
            float getSmoothCurvature() { return smoothCurvature; }    
            int getSmoothIterations() { return smoothIterations; }

            geometry_msgs::Pose getMapOrigin() { return mapOrigin; }
            
            // R = 6, 6.75 DEG
            float dy[3];// = { 0,        -0.0415893,  0.0415893};
            float dx[3];// = { 0.7068582,   0.705224,   0.705224};
            float dt[3];// = { 0,         0.1178097,   -0.1178097};

        private:
            void setDefaults();
            void recalculateMoveArrays();
            float cellSize;
            bool manualMode;
            bool visualizationEnabled;
            bool visualization2DEnabled;
            float visualizationDelay;
            bool reverseEnabled;

            float penaltyTurning;
            float penaltyReversing;
            float penaltyCOD;

            bool dubinsLookup;
            bool twoD;
            bool dubins;
            bool dubinsShot;
            float dubinsShotDistance;
            float dubinsWidth;
            float dubinsStepSize;

            float length;
            float width;
            float bloating;
            float turningRadius;

            float minRoadWidth;

            int iterations;

            int bbSize;

            std::string rsCostFilePath;
            float rsExpansionFactor;

            float voronoiMaxDistance;
            float voronoiAlpha;
            float voronoiTheta;

            float smoothAlpha;
            float smoothVoronoi;
            float smoothObstacle;
            float smoothSmooth;
            float smoothCurvature;    

            int smoothIterations;

            geometry_msgs::Pose mapOrigin;

    };
}


#endif // SETTINGS_H