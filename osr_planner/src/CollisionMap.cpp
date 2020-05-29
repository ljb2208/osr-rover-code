#include "CollisionMap.h"

using namespace OsrPlanner;

CollisionMap::CollisionMap()
{

}


void CollisionMap::setMap(vector<vector<bool>> binMap, int width, int height)
{
    this->binMap = binMap;
    this->width = width;
    this->height = height;

    calculateVehicleBounds();
}

void CollisionMap::setSettings(Settings* settings)
{
    this->settings = settings;

    calculateVehicleBounds();
}

void CollisionMap::calculateVehicleBounds()
{
    vehPoints.clear();

    vehLength = (settings->getLength() + settings->getBloating() * 2) / settings->getCellSize();
    vehWidth = (settings->getWidth() + settings->getBloating() * 2) / settings->getCellSize();

    halfVehLength = (vehLength / 2);
    halfVehWidth = vehWidth / 2;

    maxVehDistance = sqrt(pow(halfVehLength, 2) + pow(halfVehWidth, 2));

    int xLen = ceil(halfVehLength);
    int yLen = ceil(halfVehWidth);

    // calculate cells relative to 0,0
    for (int x = -xLen; x <= xLen; x++)
    {
        for (int y = -yLen; y <= yLen; y++)
        {
            vehPoints.push_back(CMPoint(x, y));
        }
    }    
}

bool CollisionMap::isTraversable(Node3D* node)
{
    return isTraversable(node->getX(), node->getY(), node->getT());
}

bool CollisionMap::isTraversable(float x, float y, float t)
{
    float sint = sin(t);
    float cost = cos(t);

    for (size_t i=0; i < vehPoints.size(); ++i)
    {
        CMPoint pnt = vehPoints[i];
        float newX = pnt.x * cost - pnt.y * sint;
        float newY = pnt.y * sint + pnt.x * cost;

        newX += x;
        newY += y;

        if (binMap[newX][newY])
        {
            // cout << "Collision at " << newX << "/" << newY << " x/y/t" << x << "/" << y << "/" << t << "\n";
            return false;
        }
    }
    
    return true;
}