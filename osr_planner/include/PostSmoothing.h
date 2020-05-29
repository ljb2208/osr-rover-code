#pragma once
#include <fstream>
#include <chrono>
#include <vector>
#include <limits>
#include <math.h>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;


#include "Settings.h"
#include "Node3D.h"
#include "CollisionMap.h"
#include "DynamicVoronoi.h"
#include "Helper.h"
#include "Vector2D.h"

using namespace std;

namespace OsrPlanner
{
class PostSmoothing
{
public:
    const bool MINIMIZE_PATHLENGTH = true; // otherwise minimize curvature
    const bool FIX_COLLISIONS = false;

    enum RoundType
    {
        ROUND_GD,
        ROUND_PRUNING,
        ROUND_ORIGINAL,
        ROUND_UNKOWN
    };

    struct RoundStats
    {
        double pathLength = -1;
        double maxCurvature = -1;
        double time = -1;
        int nodes = -1;
        double medianNodeObstacleDistance = -1;
        double meanNodeObstacleDistance = -1;
        double minNodeObstacleDistance = -1;
        double maxNodeObstacleDistance = -1;
        double stdNodeObstacleDistance = -1;
        double medianTrajObstacleDistance = -1;
        double meanTrajObstacleDistance = -1;
        double minTrajObstacleDistance = -1;
        double maxTrajObstacleDistance = -1;
        double stdTrajObstacleDistance = -1;
        RoundType type = ROUND_UNKOWN;

        std::string typeName() const {
            switch (type) {
                case ROUND_GD:
                    return "gd";
                case ROUND_PRUNING:
                    return "pruning";
                case ROUND_ORIGINAL:
                    return "original";
                default:
                    return "unknown";
            }
        }
    };

    PostSmoothing();

    int insertedNodes;
    int pruningRounds;
    int collisionFixAttempts;
    int roundsWithCollisionFixAttempts;
    vector<int> nodesPerRound;        

    bool smooth(const vector<Node3D> &originalPathIntermediaries);
    bool smooth();    

    void setSettings(Settings* settings) { this->settings = settings;}
    void setCollisionMap(CollisionMap* collisionMap) { this->collisionMap = collisionMap; }
    void setVoronoi(DynamicVoronoi* voronoi) { this->voronoi = voronoi; }
    void tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());
    vector<Node3D> getPath() { return path; }
    vector<Node3D> getGradientPath() { return gPath; }

private:
    
    vector<Node3D> toSteeredTrajectoryPoints(const Node3D& start, const Node3D& end);
    vector<Node3D> toSteeredTrajectoryPoints(const vector<Node3D> &path);
    vector<Node3D> steer(const Node3D& start, const Node3D& end);
    // vector<Node3D> steer(const Node3D& start, const Node3D& end);
    double bilinearDistance(double x, double y);
    bool distanceGradient(double x, double y, double &dx, double &dy, double p = 0.1);
    double distanceToObs(int xi, int yi);


    void updateAngles(std::vector<Node3D> &path, bool AverageAngles = true, bool preventCollisions = true);
    double slope(double x1, double y1, double x2, double y2);
    double slope(const Node3D &a, const Node3D &b);
    bool collides(const Node3D &a, const Node3D &b);
    bool collides(const std::vector<Node3D> &path);
    double evaluate(const std::vector<Node3D> &path);
    double evaluatePathLengthMetric(const vector<Node3D>& trajectory);

    vector<Node3D> path;
    vector<Node3D> gPath;
    Settings* settings;
    CollisionMap* collisionMap;
    DynamicVoronoi* voronoi;

//    static void fixCollision(std::vector<GNode> &path,
//                             const std::vector<Tpoint> &originalPathIntermediaries,
//                             const Tpoint &node,
//                             unsigned int i)
//    {
//        auto closest = PlannerUtils::closestPoint(node, originalPathIntermediaries);
//        GNode repair;
//        if (closest.euclidianDistance(path[i - 1]) < MIN_NODE_DISTANCE)
//        {
//            path[i - 1].x_r = closest.x_r;
//            path[i - 1].y_r = closest.y_r;
////                    path[i-1].theta = closest.theta;
//            repair = path[i - 1];
//        } else if (closest.euclidianDistance(path[i]) < MIN_NODE_DISTANCE)
//        {
//            path[i].x_r = closest.x_r;
//            path[i].y_r = closest.y_r;
////                    path[i].theta = closest.theta;
//            repair = path[i];
//        } else {
//            repair = GNode(closest.x, closest.y, PlannerUtils::slope(path[i - 1], path[i]));
//            path.insert(path.begin() + i, repair);
//        }
//#ifdef DEBUG
//        QtVisualizer::drawNode(repair, Qt::cyan, 0.4);
//#endif
//    }
        
};
}