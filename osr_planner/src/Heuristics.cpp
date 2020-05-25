#include "Heuristics.h"


#include <iostream>
#include <fstream>

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

    if (settings->getTurningRadius() != turnRadius)
    {
        turnRadius = settings->getTurningRadius();
        // calculateRSCosts();
    }
}

void Heuristics::setMap(vector<vector<bool>> map, int width, int height)
{
    this->map = map;
    this->height = height;
    this->width = width;

    loadRSCosts();
}

float Heuristics::getHeuristicValue(float startX, float startY, float startT, float goalX, float goalY, float goalT)
{
    float hVal = 0;

    float twoDCost = get2DCost(startX, startY, goalX, goalY);
    float eucDist = Helper::euclidianDistance(startX, startY, goalX, goalY);
    float rsDist = getRSDistance(startX, startY, startT, goalX, goalY, goalT);

    hVal = max(eucDist, rsDist);
    hVal = max(hVal, twoDCost);

    return hVal;
}

float Heuristics::get2DCost(float startX, float startY, float goalX, float goalY)
{
    int iStartX = (int) startX;
    int iStartY = (int) startY;

    float twoDCost = twoDCosts[iStartX][iStartY];
    float eucDistSG = Helper::euclidianDistance(startX, startY, goalX, goalY);
    float eucDistMod = Helper::euclidianDistance(iStartX, iStartY, goalX, goalY);
    float eucDist = Helper::euclidianDistance(startX, startY, iStartX, iStartY);

    if (eucDistSG > eucDistMod)
    {
        twoDCost += eucDist;
    }
    else if (eucDistSG < eucDistMod)
    {
        twoDCost -= eucDist;
    }
    
    return twoDCost;
}

float Heuristics::getRSDistance(float startX, float startY, float startT, float goalX, float goalY, float goalT)
{    
    ompl::base::ReedsSheppStateSpace reedsSheppPath(settings->getTurningRadius());
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();   

    rsStart->setXY(startX, startY);
    rsStart->setYaw(startT);

    rsEnd->setXY(goalX, goalY);
    rsEnd->setYaw(goalT);

    float reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);     

    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);     

    return reedsSheppCost;

    rsStart->setXY((int) startX, (int) startY);
    rsEnd->setXY((int)goalX, (int)goalY);

    float reedsSheppCost2 = reedsSheppPath.distance(rsStart, rsEnd);          


    
    // check rounding 
    // also use ratio for returning cost between two headings



    float xVal = startX - goalX;
    float yVal = startY - goalY;

    // rotate point around T0    
    float sinT = sin(-goalT);
    float cosT = cos(-goalT);


    float xfAdj = xVal * cosT  - yVal * sinT;
    float yfAdj = yVal * cosT + xVal * sinT;

    int xAdj = (int)(xVal * cosT  - yVal * sinT);
    int yAdj = (int)(yVal * cosT + xVal * sinT);

    xAdj += width;
    yAdj += height;

    float tVal = startT - goalT;

    if (tVal < 0)
        tVal = 2 * M_PI + tVal;
    else if (tVal >= 2 * M_PI)
        tVal = tVal - (2 * M_PI);    

    float tTest = tVal / Constants::deltaHeadingRad;
    int tIndex = (int) (tVal / Constants::deltaHeadingRad);
    int tIndex1 = tIndex + 1;

    if (tIndex1 >= Constants::headings)
    {
        tIndex1 = 0;
    }    


    int idx = (xAdj * height * 2 * Constants::headings) + (yAdj * Constants::headings);
    int idx2 = idx + tIndex1;
    idx += tIndex;

    float val1 = rsCosts[idx];
    float val2 = rsCosts[idx2];

    float decValue = tTest - (int) tTest;

    //  float adjVal = (1 - decValue) * val1 + decValue * val2;
    float adjVal = val1;
    float eucDistance = Helper::euclidianDistance(xVal, yVal, (int) xVal, (int) yVal);
    adjVal += eucDistance;

    float diff = (adjVal - reedsSheppCost) / reedsSheppCost;

    


    rsStart->setXY(xAdj, yAdj);
    rsStart->setYaw(tIndex * Constants::deltaHeadingRad);

    rsEnd->setXY(80, 80);
    rsEnd->setYaw(0);

    float reedsSheppCost3 = reedsSheppPath.distance(rsStart, rsEnd);      

    rsStart->setXY(xfAdj + width, yfAdj + height);
    rsStart->setYaw(tIndex * Constants::deltaHeadingRad);

    rsEnd->setXY(80, 80);
    rsEnd->setYaw(0);

    float reedsSheppCost4 = reedsSheppPath.distance(rsStart, rsEnd);      

    if (diff > 0.1 || diff < -0.1)
    {
        int why = 0;
    }

    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);

    return diff;    
}

void Heuristics::calculate2DCosts(int goalX, int goalY)
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

void Heuristics::calculateRSCosts()
{
    if (rsCosts != nullptr)
        delete [] rsCosts;

    ofstream outfile;    
    outfile.open(getRSCostFileName());    

    ofstream debugoutfile;    
    debugoutfile.open("debug.txt");    

    ompl::base::ReedsSheppStateSpace reedsSheppPath(settings->getTurningRadius());
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();    
    rsEnd->setXY(0, 0);
    rsEnd->setYaw(0);
    rsStart->setXY(0, 0);

    rsCosts = new float[width*2*height*2*Constants::headings];

    int idx = 0;            

    for (int x=-width; x < width; x++)
    {
        for (int y=-height; y < height; y++)
        {    
            for (int tc=0; tc < Constants::headings; tc++)
            { 
                //idx = (x+ width) * (y+height) * constant::headings + (y+height) * Constant::headings + tc;
                float startT = 0 + tc * Constants::deltaHeadingRad;               
                rsStart->setXY((float)x, (float)y);
                rsStart->setYaw(startT);

                float reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);                
                rsCosts[idx] = reedsSheppCost;                                
                outfile << reedsSheppCost << "\n";
                debugoutfile << x << "," << y << "," << tc << "," << reedsSheppCost << "," << idx << "\n";

                idx++;
            }            
        }
    }

    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);

    outfile.close();
    debugoutfile.close();
}

void Heuristics::loadRSCosts() 
{
    ifstream inFile(getRSCostFileName());

    if (!inFile.good())
    {
        calculateRSCosts();
        return;
    }

    if (rsCosts != nullptr)
        delete [] rsCosts;


    int count = width*2*height*2*Constants::headings;

    rsCosts = new float[width*2*height*2*Constants::headings];

    int idx = 0;
    char cost[20];

    while (!inFile.eof())
    {        
        inFile.getline(cost, 20);

        if (!inFile.good())
            break;
        
        float val = stof(cost);            
        rsCosts[idx] = val;
        idx++;                    
    }

    inFile.close();
}

string Heuristics::getRSCostFileName()
{
    string fileName = settings->getRSCostFilePath();    
    fileName.append("/");
    fileName.append("r");
    fileName.append(to_string(settings->getTurningRadius()));
    fileName.append("-w");
    fileName.append(to_string(width));
    fileName.append("-h");
    fileName.append(to_string(height));
    fileName.append("-hc");
    fileName.append(to_string(Constants::headings));
    fileName.append(".txt");

    return fileName;
}