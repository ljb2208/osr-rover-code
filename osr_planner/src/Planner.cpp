#include "Planner.h"

using namespace OsrPlanner;

Planner::Planner()
{
    updateSettings();

    pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
    pubVoronoi = n.advertise<nav_msgs::OccupancyGrid>("/voronoi/map", 1, true);

    if (settings.getManualMode()) { 
        subMap = n.subscribe("/map", 1, &Planner::setMap, this);
    } else {
        subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
    }

    subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
    subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);

    reconfigureCB = boost::bind(&Planner::reconfigureCallback, this, _1, _2);
    reconfigureServer.setCallback(reconfigureCB);
}

Planner::~Planner()
{
    if (dubinsLookup != NULL)
        delete [] dubinsLookup;
}

void Planner::reconfigureCallback(osr_planner::PlannerSettingsConfig &config, uint32_t level)
{
    settings.setFromConfig(config);
    updateSettings();
}

void Planner::updateSettings()
{
    visualization.setSettings(&settings);
    path.setSettings(&settings);
    smoothedPath.setSettings(&settings);
    smoother.setSettings(&settings);

    initializeLookups();
    plan();
}

void Planner::initializeLookups() {
    if (settings.getDubinsLookup())
    {
        if (dubinsLookup != NULL)
            delete [] dubinsLookup;

        int sz = Constants::headings * Constants::headings * settings.getDubinsWidth() * settings.getDubinsWidth();

        dubinsLookup = new float [sz];
        Lookup::dubinsLookup(dubinsLookup, settings.getDubinsWidth(), settings.getCellSize(), settings.getTurningRadius());
    }

    Lookup::collisionLookup(collisionLookup, settings.getCellSize(), settings.getBBSize(), settings.getLength(), settings.getWidth());
}

void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    ROS_INFO("Received new map");    

    grid = map;

    configurationSpace.updateGrid(map);

    int height = map->info.height;
    int width = map->info.width;
    std::vector<std::vector<bool>> binMap;
    binMap.resize(width);    

    for (int x = 0; x < width; x++) { 
        binMap[x].resize(height);
        for (int y = 0; y < height; ++y) {
            binMap[x][y] = map->data[y * width + x] ? true : false;
        }
    }    

    heuristics.setMap(binMap, width, height);
    voronoiDiagram.initializeMap(width, height, binMap);
    voronoiDiagram.update();
    voronoiDiagram.visualize();    

    nav_msgs::OccupancyGrid occGrid;    
    occGrid.header.frame_id = "map";
    occGrid.info.resolution = 1.0f;
    occGrid.info.map_load_time = ros::Time::now();    
    occGrid.header.stamp = occGrid.info.map_load_time;
    occGrid.info.origin.position.x = 0;
    occGrid.info.origin.position.y = 0;
    occGrid.info.origin.position.z = 0;
    occGrid.info.origin.orientation.x = 0;
    occGrid.info.origin.orientation.y = 0;
    occGrid.info.origin.orientation.z = 0;
    occGrid.info.origin.orientation.w = 1;
    voronoiDiagram.getOccupancyGrid(occGrid);

    pubVoronoi.publish(occGrid);

}

void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
    float x = initial->pose.pose.position.x / settings.getCellSize();
    float y = initial->pose.pose.position.y / settings.getCellSize();
    float t = tf::getYaw(initial->pose.pose.orientation);

    // publish the start without covariance for rviz
    geometry_msgs::PoseStamped startN;
    startN.pose.position = initial->pose.pose.position;
    startN.pose.orientation = initial->pose.pose.orientation;
    startN.header.frame_id = "map";
    startN.header.stamp = ros::Time::now();

    ROS_INFO_STREAM("Start x/y/z: " << 
        initial->pose.pose.position.x << "/" <<
        initial->pose.pose.position.y << "/" <<
        initial->pose.pose.position.z << " orientation x/y/z/w: " << 
        initial->pose.pose.orientation.x << "/" <<
        initial->pose.pose.orientation.y << "/" <<
        initial->pose.pose.orientation.z << "/" <<
        initial->pose.pose.orientation.w      
        );

    ROS_INFO_STREAM("New start x:" << x << " y:" << y << " t:" << Helper::toDeg(t));

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
        validStart = true;
        start = *initial;

        if (settings.getManualMode()){ 
            plan();
        }

        // publish start for RViz
        pubStart.publish(startN);
    } else {
        ROS_WARN_STREAM("invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t));
    }
}

void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
    // retrieving goal position
    float x = end->pose.position.x / settings.getCellSize();
    float y = end->pose.position.y / settings.getCellSize();
    float t = tf::getYaw(end->pose.orientation);

    ROS_INFO_STREAM("Goal x/y/z: " << 
        end->pose.position.x << "/" <<
        end->pose.position.y << "/" <<
        end->pose.position.z << " orientation x/y/z/w: " << 
        end->pose.orientation.x << "/" <<
        end->pose.orientation.y << "/" <<
        end->pose.orientation.z << "/" <<
        end->pose.orientation.w      
        );

            
    ros::Time t0 = ros::Time::now();

    heuristics.calculate2DCosts((int)x, (int)y);

    ros::Time t1 = ros::Time::now();
    ros::Duration d1(t1 - t0);

    ROS_INFO_STREAM("2D Cost time: " << d1 * 1000);

    ros::Time t2 = ros::Time::now();

    heuristics.calculate2DCostsNew((int)x, (int)y);

    ros::Time t3 = ros::Time::now();
    ros::Duration d2(t3 - t2);

    ROS_INFO_STREAM("2D Cost time: " << d2 * 1000);

    ROS_INFO_STREAM("New goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t));

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
        validGoal = true;
        goal = *end;

        if (settings.getManualMode()){ 
            plan();
        }

    } else {
        ROS_WARN_STREAM("invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl);
    }   
}

void Planner::plan() {
    // if a start as well as goal are defined go ahead and plan
    if (validStart && validGoal) {

        // ___________________________
        // LISTS ALLOWCATED ROW MAJOR ORDER
        int width = grid->info.width;
        int height = grid->info.height;
        int depth = Constants::headings;
        int length = width * height * depth;
        
        // define list pointers and initialize lists
        Node3D* nodes3D = new Node3D[length]();
        Node2D* nodes2D = new Node2D[width * height]();

        // ________________________
        // retrieving goal position
        float x = goal.pose.position.x / settings.getCellSize();
        float y = goal.pose.position.y / settings.getCellSize();
        float t = tf::getYaw(goal.pose.orientation);
        
        // set theta to a value (0,2PI]
        t = Helper::normalizeHeadingRad(t);
        const Node3D nGoal(x, y, t, 0, 0, nullptr, &settings);
        
        // _________________________
        // retrieving start position
        x = start.pose.pose.position.x / settings.getCellSize();
        y = start.pose.pose.position.y / settings.getCellSize();
        t = tf::getYaw(start.pose.pose.orientation);
        // set theta to a value (0,2PI]
        t = Helper::normalizeHeadingRad(t);
        Node3D nStart(x, y, t, 0, 0, nullptr, &settings);
        

        // ___________________________
        // START AND TIME THE PLANNING
        ros::Time t0 = ros::Time::now();

        // CLEAR THE VISUALIZATION
        visualization.clear();
        // CLEAR THE PATH
        path.clear();
        smoothedPath.clear();
        // // FIND THE PATH
        AlgorithmStats stats;
        Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization, &settings, stats);

        ros::Time t1 = ros::Time::now();
        ros::Duration d1(t1 - t0);

        ROS_INFO_STREAM("Algorithm time: " << d1 * 1000);

        if (nSolution == nullptr)        
        {
            ROS_WARN("No solution found");
        }
        // // TRACE THE PATH
        smoother.tracePath(nSolution);
        // CREATE THE UPDATED PATH
        path.updatePath(smoother.getPath());
        // SMOOTH THE PATH
        smoother.smoothPath(voronoiDiagram);
        // CREATE THE UPDATED PATH
        smoothedPath.updatePath(smoother.getPath());
        ros::Time t2 = ros::Time::now();
        ros::Duration d2(t2 - t1);
        ROS_INFO_STREAM("Trace and smooth time: " << d2 * 1000);

        ros::Duration d3(t2 - t0);
        ROS_INFO_STREAM("Total planning time: " << d3 * 1000);                

        // _________________________________
        // PUBLISH THE RESULTS OF THE SEARCH
        ros::Time t3 = ros::Time::now();
        path.publishPath();
        path.publishPathNodes();
        path.publishPathVehicles();
        smoothedPath.publishPath();
        smoothedPath.publishPathNodes();
        smoothedPath.publishPathVehicles();
        visualization.publishNode3DCosts(nodes3D, width, height, depth);
        visualization.publishNode2DCosts(nodes2D, width, height);
        ros::Time t4 = ros::Time::now();

        ros::Duration d4(t4 - t3);
        ROS_INFO_STREAM("Total publishing time: " << d4 * 1000);      

        outputAlgoStats(stats);        

        delete [] nodes3D;
        delete [] nodes2D;

    } else {
        ROS_WARN("missing goal or start");
    }
}

void Planner::outputAlgoStats(AlgorithmStats& stats)
{
    ROS_INFO_STREAM("Algorith Stats. Iterations: " << stats.iterations);  
    outputAlgoStat("updateH ", stats.updateH);
    outputAlgoStat("viz.    ", stats.viz);
}
void Planner::outputAlgoStat(std::string name, FunctionCallStats& stat)
{
    ROS_INFO_STREAM(name << " : " << stat.numCalls << " min: " << stat.getMinCallTime() << " max: " << stat.maxCallTime << " avg: " 
            << stat.getAvgCallTime() << " total: " << stat.totalCallTime);
}