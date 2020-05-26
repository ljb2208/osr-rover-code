#ifndef HAS_ALGORITHM_H
#define HAS_ALGORITHM_H

#include <boost/heap/binomial_heap.hpp>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "Node3D.h"
#include "Node2D.h"
#include "Visualization.h"
#include "Settings.h"
#include "AlgorithmStats.h"
#include "Heuristics.h"
#include "Helper.h"
#include "CollisionMap.h"
#include "DynamicVoronoi.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

namespace OsrPlanner {

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
                             DynamicVoronoi& voronoiDiagram,
                             CollisionMap& collisionMap,                             
                             Visualization& visualization,                             
                             AlgorithmStats& stats,
                             Heuristics& heuristics);

   void setSettings(Settings* settings) { this->settings = settings;}   
   void dumpNodesToFile(int index, const boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq);
   void checkNodes(int index, const boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq);   
   void processNode(Node3D* nodes3D, const Node3D& goal, Node3D* nPred, Node3D* nSucc, int width, int height, DynamicVoronoi& voronoiDiagram, CollisionMap& collisionMap, Heuristics& heuristics, int& iPred, bool updateG);
   Node3D* validateRSPath(const Node3D& goal, Node3D* nPred, CollisionMap& collisionMap);

   private:
        // OPEN LIST AS BOOST IMPLEMENTATION
         typedef boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>> priorityQueue;
         priorityQueue pq;
         Settings* settings;

         ros::NodeHandle n;  
         ros::Publisher pubRS;            
         geometry_msgs::PoseArray rsPoses;         
         int getDirection(bool reverse, float tOrig, float tNew);

};
}
#endif // HAS_ALGORITHM_H
