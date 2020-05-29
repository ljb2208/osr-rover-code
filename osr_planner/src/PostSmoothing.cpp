#include "PostSmoothing.h"

using namespace OsrPlanner;

PostSmoothing::PostSmoothing()
{
    insertedNodes = 0;
    pruningRounds = 0;
    collisionFixAttempts = 0;
    roundsWithCollisionFixAttempts = 0;        
}

void PostSmoothing::tracePath(const Node3D* node, int i, std::vector<Node3D> path)
{
    if (node == nullptr) {
        this->path = path;    
        return;
    }

    i++;
    path.insert(path.begin(), *node);
    tracePath(node->getPred(), i, path);
}

bool PostSmoothing::smooth()
{
    auto intermediary = toSteeredTrajectoryPoints(path);
    return smooth(intermediary);
}

bool PostSmoothing::smooth(const std::vector<Node3D> &originalPathIntermediaries)
{
    const bool averageAngles = true;

    insertedNodes = 0;
    pruningRounds = 0;
    collisionFixAttempts = 0;
    roundsWithCollisionFixAttempts = 0;
    nodesPerRound.clear();            

    updateAngles(path, averageAngles);
    // PlannerUtils::updateAngles(path, AverageAngles);

    double dx, dy;
    double eta = settings->getGripsEta(); // gradient descent step size
    for (int round = 0; round < settings->getGripsGradientDescentRounds(); ++round)
    {        
        // gradient descent along distance field
        for (size_t i = 1; i < path.size()-1; ++i)
        {
            // compute gradient
            distanceGradient(path[i].getX(), path[i].getY(), dx, dy, 1.);
            double distance = bilinearDistance(path[i].getX(), path[i].getY());
            distance = std::max(0.1, distance);

            float xval = -eta * dx / distance;
            float yval = eta * dy / distance;

            ROS_INFO_STREAM("Xval: " << xval << " Yval: " << yval << " distance: " << distance << " gradx: " << dx << " y: " << dy << "\n");

            path[i].setX(path[i].getX() - eta * dx / distance);
            path[i].setY(path[i].getY() + eta * dy / distance);

            // path[i].x_r -= eta * dx / distance;
            // path[i].y_r += eta * dy / distance;
        }
        eta *= settings->getGripsEtaDiscount(); // discount factor

        updateAngles(path, averageAngles);
        // PlannerUtils::updateAngles(path, AverageAngles);

        // add/remove nodes if necessary
        vector<Node3D> tpath = toSteeredTrajectoryPoints(path[0], path[1]);
        double lastDistance = bilinearDistance(tpath[0].getX(), tpath[0].getY());
        double lastDistance2 = bilinearDistance(tpath[1].getX(), tpath[1].getY());
        double lastDifference = lastDistance2 - lastDistance;
        vector<Node3D> npath;
        Vector2D lastNodePosition(tpath[0].getX(), tpath[0].getY());

        for (size_t i = 0; i < path.size() - 1; ++i)
        {
            Node3D current = path[i];
            Node3D next = path[i+1];            

            lastNodePosition = Vector2D(current.getX(), current.getY());
            Vector2D nextNodePosition = Vector2D(next.getX(), next.getY());

            npath.push_back(path[i]);

            tpath = toSteeredTrajectoryPoints(path[i], path[i+1]);

            for (auto &p : tpath)
            {
                double distance = bilinearDistance(p.getX(), p.getY());
                double difference = distance - lastDistance;
                if (lastDifference < 0 && difference > 0
                    && lastNodePosition.distance(p.getX(), p.getY()) >= settings->getGripsMinNodeDistance()
                    && nextNodePosition.distance(p.getX(), p.getY()) >= settings->getGripsMinNodeDistance())
                {
                    // local minimum
                    npath.emplace_back(Node3D(p.getX(), p.getY()));
                    lastNodePosition = Vector2D(p.getX(), p.getY());

                    ++insertedNodes;
                }
                lastDifference = difference;
                lastDistance = distance;
            }
        }

        npath.push_back(path[path.size()-1]);
        path = npath;

        updateAngles(path, averageAngles);
        // PlannerUtils::updateAngles(path, AverageAngles);        
    }

    gPath.clear();
    
    for (size_t z=0; z < path.size(); ++z)
    {
        gPath.push_back(path[z]);
    }    

    // try to remove nodes
    size_t lastPathLength;
    int pruningRound = 1;
    int fixes = 0;
    nodesPerRound.push_back((int)(path.size()));
    do
    {        
        if (pruningRound >= settings->getGripsMaxPruningRounds())
        {
            ROS_ERROR_STREAM("Giving up pruning after " << pruningRound <<" rounds. The smoothed trajectory most likely collides.");                        
            return false;
        }

        lastPathLength = path.size();
        ROS_INFO_STREAM("#### PRUNING ROUND " <<  pruningRound++);
        ++pruningRounds;

        fixes = 0;

        // determine unremovable nodes
        vector<int> unremovable;

        vector<int> local_unremovable{0};
        for (size_t i = 1; i < path.size() - 1; ++i)
        {
            if (collides(path[i - 1], path[i + 1]))
            {
                local_unremovable.push_back(i);
            }
        }

        local_unremovable.push_back((int) (path.size() - 1));
        unremovable = local_unremovable;

        updateAngles(path, averageAngles, true);

        // compute final trajectory
        vector<Node3D> finalPath;
        for (size_t ui = 1; ui < unremovable.size(); ++ui)
        {
            const auto i = unremovable[ui - 1];
            const auto j = unremovable[ui];

            if (finalPath.empty() || path[i] != finalPath.back())
                finalPath.push_back(path[i]);

            if (j - i <= 1)
                continue; // no intermediary nodes

            vector<double> distances(j - i + 1, numeric_limits<double>::max());
            vector<int> predecessors(j - i + 1);

            for (size_t pi = 0; pi < predecessors.size(); ++pi)
                predecessors[pi] = pi == 0 ? 0 : pi-1;

            distances[0] = 0; // source weight is zero

            // run Bellman-Ford to determine best path from source (i) to sink (j)
            for (auto u = i; u <= j - 1; ++u)
            {
                for (auto v = u + 1; v <= j; ++v)
                {
                    if (collides(path[u], path[v]))
                        continue; // a break has the same effect for linear steering and would be more efficient

                    double edgeWeight = evaluate(vector<Node3D>{path[u], path[v]});

                    if (distances[u - i] + edgeWeight < distances[v - i])
                    {
                        distances[v - i] = distances[u - i] + edgeWeight;
                        predecessors[v - i] = u - i;
                    }
                }
            }

            int k = j - i;
            auto insertPosition = finalPath.size();
            while (k > 0)
            {
                if (path[k + i] != finalPath.back())
                    finalPath.insert(finalPath.begin() + insertPosition, path[k + i]);
                if (k == predecessors[k])
                {
                    ROS_ERROR_STREAM("Failed to prune path due to loop in shortest path.");
                    break;
                }
                k = predecessors[k];
            }
        }
        if (path.back() != finalPath.back())
            finalPath.push_back(path.back());

        path = finalPath;
        nodesPerRound.push_back((int)path.size());        

        if (lastPathLength != path.size())
            ROS_INFO_STREAM("Continuing pruning because lastPathLength " << lastPathLength << " != path.size() " << path.size());
        if (fixes > 0)
            ROS_INFO_STREAM("Continuing pruning because fixes " << fixes << " > 0");
    }
    while (lastPathLength != path.size() || fixes > 0);

    ROS_INFO_STREAM("Post-smoothing SUCCEEDED after " << pruningRound << " pruning rounds.");

    return true;
}

vector<Node3D> PostSmoothing::toSteeredTrajectoryPoints(const vector<Node3D> &path)
{
    vector<Node3D> points;

    for (unsigned int i = 0; i < path.size()-1; ++i)
    {
        points.emplace_back(path[i]);
        
        vector<Node3D> tpath = steer(path[i], path[i+1]);                        
        points.insert(points.end(), tpath.begin(), tpath.end());
        points.emplace_back(path[i+1]);
    }

    return points;
}

vector<Node3D> PostSmoothing::toSteeredTrajectoryPoints(const Node3D& start, const Node3D& end)
{    
    return steer(start, end);                                   
}

vector<Node3D> PostSmoothing::steer(const Node3D& start, const Node3D& end)
{
    vector<Node3D> trajectory;
    ompl::base::ReedsSheppStateSpace reedsSheppPath(settings->getTurningRadius());
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();   
    State* rsPoint = (State*)reedsSheppPath.allocState();   

    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());

    rsEnd->setXY(end.getX(), end.getY());
    rsEnd->setYaw(end.getT());    

    double distance = reedsSheppPath.distance(rsStart, rsEnd);
    
    int limit = (int) (distance / 0.1);    
        
    float x, y , t;

    for (int i=0; i <= limit; i++)
    {
        double seg = i * 0.1;
        reedsSheppPath.interpolate(rsStart, rsEnd, seg, rsPoint);       

        x = rsPoint->getX();
        y = rsPoint->getY();
        t = rsPoint->getYaw();

        Node3D point = Node3D(x, y, t);
        trajectory.push_back(point);        
    }

    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);
    reedsSheppPath.freeState(rsPoint);

    return trajectory;
}

// vector<Node3D> PostSmoothing::steer(const Node3D& start, const Node3D& end)
// {
//     vector<Node3D> trajectory;
//     ompl::base::ReedsSheppStateSpace reedsSheppPath(settings->getTurnRadius());
//     State* rsStart = (State*)reedsSheppPath.allocState();
//     State* rsEnd = (State*)reedsSheppPath.allocState();   
//     State* rsPoint = (State*)reedsSheppPath.allocState();   

//     rsStart->setXY(start->getX(), start->getY());
//     rsStart->setYaw(start->getT());

//     rsEnd->setXY(end.getX(), end.getY());
//     rsEnd->setYaw(end.getT());    

//     double distance = reedsSheppPath.distance(rsStart, rsEnd);
    
//     int limit = (int) (distance / 0.1);    
        
//     float x, y , t;

//     for (int i=0; i <= limit; i++)
//     {
//         double seg = i * 0.1;
//         reedsSheppPath.interpolate(rsStart, rsEnd, seg, rsPoint);       

//         x = rsPoint->getX();
//         y = rsPoint->getY();
//         t = rsPoint->getYaw();

//         if (!collisionMap->isTraversable(x, y, t))
//         {
//             reedsSheppPath.freeState(rsStart);
//             reedsSheppPath.freeState(rsEnd);
//             reedsSheppPath.freeState(rsPoint);
//             trajectory.clear();
//             return trajectory;
//         }

//         Node3D point = Node3D(x, y, t);
//         trajectory.push_back(point);        
//     }

//     reedsSheppPath.freeState(rsStart);
//     reedsSheppPath.freeState(rsEnd);
//     reedsSheppPath.freeState(rsPoint);

//     return trajectory;
// }

double PostSmoothing::bilinearDistance(double x, double y)
{
    int xi = (int) x;
    int yi = (int) y;
    double u_ratio = x - xi;
    double v_ratio = y - yi;
    double u_opposite = 1. - u_ratio;
    double v_opposite = 1. - v_ratio;
    xi = std::max(std::min(collisionMap->getMapWidth(), xi), (int) 0);
    yi = std::max(std::min(collisionMap->getMapHeight(), yi), (int) 0); // repeat voxels at edge
    int xp = std::max(std::min(collisionMap->getMapWidth(), xi + 1), (int) 0);
    int yp = std::max(std::min(collisionMap->getMapHeight(), yi + 1), (int) 0);
    double tl = distanceToObs(xi, yi), tr = distanceToObs(xp, yi);
    double bl = distanceToObs(xi, yp), br = distanceToObs(xp, yp);
    return   (tl * u_opposite + tr * u_ratio) * v_opposite
        + (bl * u_opposite + br * u_ratio) * v_ratio;
}

double PostSmoothing::distanceToObs(int xi, int yi)
{    
    return voronoi->getDistance(xi, yi);
}

/**
 * Computes gradient of distance field at position x,y.
 * @param x Position coordinate x.
 * @param y Position coordinate y.
 * @param dx Resulting gradient coordinate x.
 * @param dy Resulting gradient coordinate y.
 * @param p Sampling precision.
 * @return True, if x and y are within grid boundaries.
 */
bool PostSmoothing::distanceGradient(double x, double y, double &dx, double &dy, double p)
{
    if (x < 0 || y < 0 || x > collisionMap->getMapWidth() || y > collisionMap->getMapHeight())
        return false;

    // compute distances left, right, top, bottom
    double dl = bilinearDistance(x-p, y), dr = bilinearDistance(x+p, y);
    double db = bilinearDistance(x, y-p), dt = bilinearDistance(x, y+p);

    dx = (dl - dr) / (p * 2.);
    dy = (dt - db) / (p * 2.);

    return true;
}

void PostSmoothing::updateAngles(std::vector<Node3D> &path, bool AverageAngles, bool preventCollisions)
{
    if (path.size() < 2)
        return;

    double theta_old = path[0].getT();
    path[0].setT(slope(path[0], path[1]));
    if (preventCollisions && collides(path[0], path[1]))
        path[0].setT(theta_old); // revert setting

    for (size_t i = 1; i < path.size() - 1; ++i)
    {
        theta_old = path[i].getT();

        if (AverageAngles)
        {
            double l = slope(path[i - 1], path[i]);
            double r = slope(path[i], path[i + 1]);
            if (std::abs(l - r) >= M_PI)
            {
                if (l > r)
                    l += 2. * M_PI;
                else
                    r += 2. * M_PI;
            }
            path[i].setT((l + r) * 0.5);
        }
        else
            path[i].setT(slope(path[i - 1], path[i]));

        if (preventCollisions && (collides(path[i-1], path[i]) || collides(path[i], path[i+1])))
            path[i].setT(theta_old); // revert setting
    }

    theta_old = path[path.size() - 1].getT();
    path[path.size() - 1].setT(slope(path[path.size() - 2], path[path.size() - 1]));
    if (preventCollisions && collides(path[path.size() - 1], path[path.size() - 2]))
        path[path.size() - 1].setT(theta_old); // revert setting
}

double PostSmoothing::slope(double x1, double y1, double x2, double y2)
{
    double dy = y2 - y1;
    double dx = x2 - x1;
    double x = std::atan2(dy, dx);
    return x;
}        

double PostSmoothing::slope(const Node3D &a, const Node3D &b)
{
    double dy = b.getY() - a.getY();
    double dx = b.getX() - a.getX();
    double x = std::atan2(dy, dx);
    return x;
}

bool PostSmoothing::collides(const Node3D &a, const Node3D &b)
{    
    vector<Node3D> path = steer(a, b);    
    bool c = collides(path);
    return c;
}

bool PostSmoothing::collides(const std::vector<Node3D> &path)
{
    std::vector<Vector2D> collisions;
    
    for (unsigned int i = 1; i < path.size(); ++i)
    {
        if (!collisionMap->isTraversable(path[i].getX(), path[i].getY(), path[i].getT()))        
        {
            collisions.emplace_back(Vector2D(path[i].getX(), path[i].getY()));
            continue;
        }

        // check intermediary points
        if (i < path.size()-1)
        {
            double dx = (path[i+1].getX() - path[i].getX());
            double dy = (path[i+1].getY() - path[i].getY());
            double size = std::sqrt(dx*dx + dy*dy);
            const double scale = 0.15; //1.5;
            dx = dx / size * scale;
            dy = dy / size * scale;

            auto steps = (int)(size / std::sqrt(dx*dx + dy*dy));

            for (int j = 1; j <= steps ; ++j)
            {
                // if (GNode_base::isblock(path[i].x + dx * j, path[i].y + dy * j))
                if (!collisionMap->isTraversable(path[i].getX() + dx * j, path[i].getY() + dy * j, 0))
                    //  || GNode_base::isblock(path[i].x + dx * j + .5, path[i].y + dy * j + .5))
                {
                    collisions.emplace_back(Vector2D(path[j].getX(), path[j].getY()));
                    continue;
                }
            }
        }
    }
    return !collisions.empty();
}

double PostSmoothing::evaluate(const std::vector<Node3D> &path)
{
    vector<Node3D> trajectory = toSteeredTrajectoryPoints(path);
    return evaluatePathLengthMetric(trajectory);
}

double PostSmoothing::evaluatePathLengthMetric(const vector<Node3D>& trajectory)
{
    double xold_, yold_, dist_, dx_, dy_, s;
    xold_ = 0;
    yold_ = 0;
    dist_ = 0;
    
    /// Save the path!!
    for (std::size_t i = 0; i < trajectory.size(); i++)
    {
        if (i == 0)
        {
            xold_ = trajectory[i].getX();
            yold_ = trajectory[i].getY();
            continue;
        }

        dx_ = (trajectory[i].getX() - xold_);
        dy_ = (trajectory[i].getY() - yold_);

        s = std::sqrt(dx_ * dx_ + dy_ * dy_);

        dist_ += s;

        xold_ = trajectory[i].getX();
        yold_ = trajectory[i].getY();
        //        cout<<"Distance : "<<dist_<<endl;
    }

    return dist_;
}
