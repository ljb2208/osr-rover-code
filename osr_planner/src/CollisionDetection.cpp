#include "CollisionDetection.h"

using namespace OsrPlanner;

CollisionDetection::CollisionDetection(Settings* settings) {
  this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup, settings->getCellSize(), settings->getBBSize(), settings->getLength(), settings->getWidth());
}

bool CollisionDetection::configurationTest(float x, float y, float t) {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int cX;
  int cY;

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {
        return false;
      }
    }
  }

  return true;
}

void CollisionDetection::getConfiguration(const Node2D* node, float& x, float& y, float& t) {
    x = node->getX();
    y = node->getY();
    // avoid 2D collision checking
    t = 99;
}

void CollisionDetection::getConfiguration(const Node3D* node, float& x, float& y, float& t) {
    x = node->getX();
    y = node->getY();
    t = node->getT();
}