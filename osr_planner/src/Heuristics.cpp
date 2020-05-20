#include "Heuristics.h"

using namespace OsrPlanner;
using namespace std;

Heuristics::Heuristics()
{
    pubNodes2D = n.advertise<geometry_msgs::PoseArray>("/test2DPoses", 100);
}

void Heuristics::calculate(int height, int width)
{

}
    
void Heuristics::setSettings(Settings* settings)
{
    this->settings = settings;
}

void Heuristics::setMap(vector<vector<bool>> map, int width, int height)
{
    this->map = map;
    this->height = height;
    this->width = width;

    calculate2DCostsNew(37, 49);
}

void Heuristics::calculate2DCosts(int goalX, int goalY)
{
    vector<tuple<int, int, int>> delta;
    delta.push_back(make_tuple<int, int, int>(-1, 0, 2)); // up
    delta.push_back(make_tuple<int, int, int>(0, -1, 3)); // left
    delta.push_back(make_tuple<int, int, int>(1, 0, 0)); // down
    delta.push_back(make_tuple<int, int, int>(0, 1, 1)); // right


    twoDCosts.clear();
    twoDPolicy.clear();

    float maxVal = (float) height * (float) width;
    float costStep = 1.0;

    // init values
    for (int x=0; x < width; x++)
    {
        twoDCosts.push_back(vector<float>());
        twoDPolicy.push_back(vector<int>());

        for (int y=0; y < height; y++)
        {
            twoDCosts[x].push_back(maxVal);    
            twoDPolicy[x].push_back(-1);
        }
    }

    bool change = true;
    
    while (change)
    {
        change = false;

        for (int x=0; x < width; x++)
        {
            for (int y=0; y < height; y++)
            {
                if (goalX == x && (int) goalY == y)
                {
                    if (twoDCosts[x][y] > 0)
                    {
                        twoDCosts[x][y] = 0;
                        twoDPolicy[x][y] = 4; // reached goal
                        change = true;
                    }
                }
                else if (map[x][y] == false)
                {
                    vector<tuple<int, int, int>>::iterator it;

                    for (tuple<int, int, int> d : delta)                    
                    {

                        int x2 = x + get<0>(d);
                        int y2 = y + get<1>(d);

                        if (x2 >= 0 && x2 < width && y2 >= 0 && y2 < height && map[x2][y2] == false)
                        {
                            float v2 = twoDCosts[x2][y2] + costStep;

                            if (v2 < twoDCosts[x][y])
                            {
                                change = true;
                                twoDCosts[x][y] = v2;
                                twoDPolicy[x][y] = get<2>(d);
                            }
                        }
                    }
                }
            }
        }
    }

    publish2DPolicy();
}

void Heuristics::calculate2DCostsNew(int goalX, int goalY)
{
    vector<tuple<int, int, int>> delta;
    delta.push_back(make_tuple<int, int, int>(-1, 0, 0)); // up
    delta.push_back(make_tuple<int, int, int>(0, -1, 1)); // left
    delta.push_back(make_tuple<int, int, int>(1, 0, 2)); // down
    delta.push_back(make_tuple<int, int, int>(0, 1, 3)); // right

    vector<vector<bool>> explored;

    twoDCosts.clear();
    twoDPolicy.clear();

    float maxVal = (float) height * (float) width;
    float costStep = 1.0;

    // init values
    for (int x=0; x < width; x++)
    {
        twoDCosts.push_back(vector<float>());
        twoDPolicy.push_back(vector<int>());
        explored.push_back(vector<bool>());

        for (int y=0; y < height; y++)
        {
            twoDCosts[x].push_back(maxVal);    
            twoDPolicy[x].push_back(-1);
            explored[x].push_back(map[x][y]);
        }
    }    

    // set goal cost to zero
    twoDCosts[goalX][goalY] = 0;
    twoDPolicy[goalX][goalY] = 4; // reached goal

    int x = goalX;
    int y = goalY;

    priority_queue<HCell, vector<HCell>, CompareHCells> pq;
    pq.push({x, y, 0, 4});

    while (!pq.empty())
    {
        HCell cell = pq.top();
        pq.pop();        

        x = cell.x;
        y = cell.y;

        if (explored[x][y])
            continue;        

        explored[x][y] = true;
        int cost = twoDCosts[x][y] + costStep;

        // add new nodes
        vector<tuple<int, int, int>>::iterator it;        

        for (tuple<int, int, int> d : delta)    
        {
            int dx = x + get<0>(d);
            int dy = y + get<1>(d);
            int policy = get<2>(d);

            if (canExplore(dx, dy))
            {
                // check if already explored but potentially lower cost
                if (explored[dx][dy])
                {
                    if (cost < (twoDCosts[dx][dy]))
                        explored[dx][dy] = false;
                    else
                        continue;
                }                

                twoDCosts[dx][dy] = cost;
                twoDPolicy[dx][dy] = policy;
                pq.push({dx, dy, cost, policy});
            }
        }

    }
        

    publish2DPolicy();
}

bool Heuristics::canExplore(int x, int y)
{
    if (x < 0 || x >= width)
        return false;
    
    if (y < 0 || y >= height)
        return false;

    if (map[x][y])
        return false;

    return true;
}

void Heuristics::publish2DPolicy()
{
    poses2D.header.frame_id = "path";
    poses2D.header.stamp = ros::Time::now();

    for (int x=0; x < width; x++)
    {
        for (int y=0; y < height; y++)
        {
            if (twoDPolicy[x][y] < 0 || twoDPolicy[x][y] > 3)
                continue;

            geometry_msgs::Pose pose;
            pose.position.x = x;
            pose.position.y = y;

            float t = 0 + twoDPolicy[x][y] * 1.5708;
            pose.orientation = tf::createQuaternionMsgFromYaw(t);
            poses2D.poses.push_back(pose);
        }
    }

    pubNodes2D.publish(poses2D);
}