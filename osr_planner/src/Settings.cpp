#include "Settings.h"
#include <math.h>

using namespace OsrPlanner;

Settings::Settings()
{
    setDefaults();
}

void Settings::setDefaults()
{
    manualMode = true;
    reverseEnabled = true;
    visualizationEnabled = true;
    visualization2DEnabled = true;

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