#include "Settings.h"
#include <math.h>

using namespace OsrPlanner;

Settings::Settings()
{
    setDefaults();
}

void Settings::setMapInfo(float resolution, geometry_msgs::Pose origin)
{
    cellSize = resolution;
    mapOrigin = origin;
}

void Settings::setFromConfig(osr_planner::PlannerSettingsConfig &config)
{
    manualMode = config.manual_mode;
    reverseEnabled = config.reverse_enabled;
    visualizationEnabled = config.visualization_enabled;
    visualization2DEnabled = config.visualization2D_enabled;
    visualizationDelay = config.visualization_delay;

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

    smoothAlpha = config.smooth_alpha;
    smoothVoronoi = config.smooth_voronoi;
    smoothObstacle = config.smooth_obstacle;
    smoothSmooth = config.smooth_smooth;
    smoothCurvature = config.smooth_curvature;
    smoothIterations = config.smooth_iterations;

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

    ros::param::param<float>("min_road_width", minRoadWidth, 0.6);
    ros::param::param<float>("length", length, 0.7);
    ros::param::param<float>("width", width, 0.45);
    ros::param::param<float>("bloating", bloating, 0.0);
    ros::param::param<float>("turn_radius", turningRadius, 2.0);

    bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);

        

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
    voronoiAlpha = 10;
    voronoiTheta = 5;

    smoothAlpha = 0.1;
    smoothVoronoi = 0.2;
    smoothObstacle = 0.2;
    smoothSmooth = 0.2;
    smoothCurvature = 0.2;
    smoothIterations = 500;

    // GRIPS settings
    gripsMinNodeDistance = 3;
    gripsEta = 0.5;
    gripsEtaDiscount = 0.8;
    gripsGradientDescentRounds = 5;
    gripsMaxPruningRounds = 100;

}

void Settings::recalculateMoveArrays()
{

}