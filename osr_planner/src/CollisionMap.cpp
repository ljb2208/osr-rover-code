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
}

void CollisionMap::setSettings(Settings* settings)
{
    this->settings = settings;

    vehLength = (settings->getLength() + settings->getBloating() * 2) / settings->getCellSize();
    vehWidth = (settings->getWidth() + settings->getBloating() * 2) / settings->getCellSize();

    halfVehLength = vehLength / 2;
    halfVehWidth = vehWidth / 2;

    // calculate 4 points of vehicle relative to centre at  0,0
    vehPoints[0] = CMPoint(halfVehLength, halfVehWidth);
    vehPoints[1] = CMPoint(halfVehLength, -halfVehWidth);
    vehPoints[2] = CMPoint(-halfVehLength, halfVehWidth);
    vehPoints[3] = CMPoint(-halfVehLength, -halfVehWidth);    
}

bool CollisionMap::isTraversable(Node3D* node)
{
    return isTraversable(node->getX(), node->getY(), node->getT());
}

bool CollisionMap::isTraversable(float x, float y, float t)
{
    float sint = sin(t);
    float cost = cos(t);

    CMPoint newVehPoints[4];

    for (int i=0; i < 4; i++)
    {
        CMPoint pnt = vehPoints[i];

        float newX = pnt.x * cost - pnt.y * sint;
        float newY = pnt.y * sint + pnt.x * cost;

        newX += x;
        newY += y;
 
        newVehPoints[i] = CMPoint(newX, newY);
    }

    int xStart = (int) Helper::minVal(newVehPoints[0].x, newVehPoints[1].x, newVehPoints[2].x, newVehPoints[3].x);
    int xEnd = (int) Helper::maxVal(newVehPoints[0].x, newVehPoints[1].x, newVehPoints[2].x, newVehPoints[3].x);

    int yStart = (int) Helper::minVal(newVehPoints[0].y, newVehPoints[1].y, newVehPoints[2].y, newVehPoints[3].y);
    int yEnd = (int) Helper::maxVal(newVehPoints[0].y, newVehPoints[1].y, newVehPoints[2].y, newVehPoints[3].y);

    for (int xVal = xStart; xVal <= xEnd; xVal++)
    {
        for (int yVal = yStart; yVal <= yEnd; yVal++)
        {
            if (binMap[xVal][yVal])
                return false;
        }
    }

    return true;
}