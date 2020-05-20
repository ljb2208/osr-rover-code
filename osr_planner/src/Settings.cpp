#include "Settings.h"
#include <math.h>

using namespace OsrPlanner;

Settings::Settings()
{
    setDefaults();
}

void Settings::setFromConfig(osr_planner::PlannerSettingsConfig &config)
{
    manualMode = config.manual_mode;
    reverseEnabled = config.reverse_enabled;
    visualizationEnabled = config.visualization_enabled;
    visualization2DEnabled = config.visualization2D_enabled;
    visualizationDelay = config.visualization_delay;

    cellSize = config.cell_size;
    penaltyTurning = config.penalty_turning;
    penaltyReversing = config.penalty_reversing;
    penaltyCOD = config.penalty_cod;

    dubinsLookup = config.dubins_lookup;
    dubins = config.dubins;
    dubinsShot = config.dubins_shot;
    dubinsShotDistance = config.dubins_shot_dist;
    dubinsWidth = config.dubins_width;
    dubinsStepSize = config.dubins_step_size;

    twoD = config.twoD;

    bloating = config.bloating;
    length = config.length;
    width = config.width;
    turningRadius = config.turn_radius;

    bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);

    minRoadWidth = config.min_road_width;

    iterations = config.iterations;
}

void Settings::setDefaults()
{
    manualMode = true;
    reverseEnabled = true;
    visualizationEnabled = true;
    visualization2DEnabled = true;
    visualizationDelay = 0;

    cellSize = 1;
    penaltyTurning = 1.05;
    penaltyReversing = 2.0;
    penaltyCOD = 2.0;

    dubinsLookup = false;
    dubins = false;
    dubinsShot = true;
    dubinsShotDistance = 100;
    dubinsWidth = 15;
    dubinsStepSize = 1;

    twoD = true;

    bloating = 0;
    length = 2.65;
    width = 1.65;
    turningRadius = 6;

    bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);

    minRoadWidth = 2;

    iterations = 30000;
}