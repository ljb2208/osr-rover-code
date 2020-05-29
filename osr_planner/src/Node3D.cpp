#include "Node3D.h"

using namespace OsrPlanner;

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3;
// possible movements

// R = 6, 6.75 DEG
// const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
// const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
// const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};


const float Node3D::dy[] = { 0,    -0.0415893,  0.0415893};
const float Node3D::dx[] = { 1.0,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,     0.1178097,   -0.1178097};


//###################################################
//                                         IS ON GRID
//###################################################
bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height && (int)(t / Constants::deltaHeadingRad) >= 0 && (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}


//###################################################
//                                        IS IN RANGE
//###################################################
bool Node3D::isInRange(const Node3D& goal) const {
  int random = rand() % 10 + 1;
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < settings->getDubinsShotDistance();
}

bool Node3D::isInRangeRS(const Node3D& goal) const
{
    int range = std::max((int) getH(), 10);
    int random = rand() % range;            

    return  random < settings->getRSExpansionFactor();    
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node3D* Node3D::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor positions forward
  if (i < 3) {
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else {
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
  }

  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, this->settings, i);
}

//###################################################
//                                      MOVEMENT COST
//###################################################

void Node3D::updateG(float vCost, bool updateAll)
{
  if (updateAll)
    updateG();

  g += vCost;
}

void Node3D::updateG() {
  // forward driving
  if (prim < 3) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim > 2) {
        g += dx[0] * settings->getPenaltyTurning() * settings->getPenaltyCOD();
      } else {
        g += dx[0] * settings->getPenaltyTurning();
      }
    } else {
      g += dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < 3) {
        g += dx[0] * settings->getPenaltyTurning() * settings->getPenaltyReversing() * settings->getPenaltyCOD();
      } else {
        g += dx[0] * settings->getPenaltyTurning() * settings->getPenaltyReversing();
      }
    } else {
      g += dx[0] * settings->getPenaltyReversing();
    }
  }
}

bool Node3D::reachedGoal(const Node3D& goal) {
    float distance = Helper::euclidianDistance(goal.getX(), goal.getY(), x, y);

    float x_val = std::abs(goal.x - x);
    float y_val = std::abs(goal.y - y);

    if (x_val <= 1.0 && y_val <= 1.0)
    {
      int q= 0;
    }

    return distance <= 0.5 && (std::abs(t - goal.t) <= Constants::deltaHeadingRad ||
          std::abs(t - goal.t) >= Constants::deltaHeadingNegRad);

    return (int)(x + 0.5) == (int)(goal.x + 0.5) &&
         (int)(y + 0.5) == (int)(goal.y + 0.5) &&
         (std::abs(t - goal.t) <= Constants::deltaHeadingRad ||
          std::abs(t - goal.t) >= Constants::deltaHeadingNegRad);
}
//###################################################
//                                 3D NODE COMPARISON
//###################################################
bool Node3D::operator == (const Node3D& rhs) const {
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}

