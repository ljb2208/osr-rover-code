#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <boost/heap/binomial_heap.hpp>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "Node3D.h"
#include "Node2D.h"
#include "Visualization.h"
#include "CollisionDetection.h"
#include "Settings.h"
#include "AlgorithmStats.h"

namespace OsrPlanner {
class Node3D;
class Node2D;
class Visualization;
class Settings;

/*!
 * \brief A class that encompasses the functions central to the search.
 */
class HASAlgorithm {
 public:
  /// The deault constructor
  HASAlgorithm();

  struct CompareNodes {
      /// Sorting 3D nodes by increasing C value - the total estimated cost
      bool operator()(const Node3D* lhs, const Node3D* rhs) const {
         return lhs->getC() > rhs->getC();
      }
      /// Sorting 2D nodes by increasing C value - the total estimated cost
      bool operator()(const Node2D* lhs, const Node2D* rhs) const {
         return lhs->getC() > rhs->getC();
      }
   };


  // HYBRID A* ALGORITHM
  /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
     \param width the width of the grid in number of cells
     \param height the height of the grid in number of cells
     \param configurationSpace the lookup of configurations and their spatial occupancy enumeration
     \param dubinsLookup the lookup of analytical solutions (Dubin's paths)
     \param visualization the visualization object publishing the search to RViz
     \return the pointer to the node satisfying the goal condition
  */
   Node3D* runAlgo(Node3D& start, const Node3D& goal, Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                             Visualization& visualization,
                             Settings* settings,
                             AlgorithmStats& stats);


   float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualization& visualization, Settings* settings);
   void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualization& visualization, Settings* settings, AlgorithmStats& stats);
   Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace, Settings* settings);
   void dumpNodesToFile(int index, const boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq);
   void checkNodes(int index, const boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq);
   void deleteNode3D(boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq, const Node3D& node);
   void deleteNode2D(boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>>& pq, const Node2D& node);

};
}
#endif // ALGORITHM_H
