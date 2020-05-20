#ifndef SETTINGS_H
#define SETTINGS_H

#include <osr_planner/PlannerSettingsConfig.h>

namespace OsrPlanner {

    class Settings{
        public:
            Settings();

            void setFromConfig(osr_planner::PlannerSettingsConfig &config);

            int getCellSize(){return cellSize;};
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
            

        private:
            void setDefaults();
            int cellSize;
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



    };
}


#endif // SETTINGS_H