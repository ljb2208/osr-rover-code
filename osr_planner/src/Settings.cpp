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
    
    rsExpansionFactor = config.rs_expansion_factor;

    voronoiMaxDistance = config.voronoi_max;
    voronoiAlpha = config.voronoi_alpha;
    voronoiTheta = config.voronoi_theta;

    recalculateMoveArrays();
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

    rsExpansionFactor = 4;

    twoD = true;

    bloating = 0;
    length = 2.65;
    width = 1.65;
    turningRadius = 6;

    bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);

    minRoadWidth = 2;

    iterations = 30000;

    rsCostFilePath = "/home/lbarnett/catkin_ws/src/osr-rover-code/osr_planner/data";

    dy[0] = 0;
    dy[1] = -0.0415893;
    dy[2] = 0.0415893;
    dx[0] = 0.7068582;
    dx[1] = 0.705224;
    dx[2] = 0.705224;
    dt[0] = 0;    
    dt[1] = 0.1178097;
    dt[2] = -0.1178097;

    voronoiMaxDistance = 8;
    voronoiAlpha = 0.1;
    voronoiTheta = 5;
}

void Settings::recalculateMoveArrays()
{

}