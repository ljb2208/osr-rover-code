#include "Algorithm.h"

#include <boost/heap/binomial_heap.hpp>
#include <chrono>
#include <iostream>
#include <fstream>

using namespace OsrPlanner;

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualization& visualization, Settings* settings);
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualization& visualization, Settings* settings, AlgorithmStats& stats);
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace, Settings* settings);


//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
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

void dumpNodesToFile(int index, const boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq);
void checkNodes(int index, const boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq);
void deleteNode3D(boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq, const Node3D& node);
void deleteNode2D(boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>>& pq, const Node2D& node);

//###################################################
//                                        3D A*
//###################################################
Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualization& visualization,
                               Settings* settings,
                               AlgorithmStats& stats) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = settings->getReverseEnabled() ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;
  int maxIterations = settings->getIterations();
  bool dumpNodes = false;

  // VISUALIZATION DELAY
  ros::Duration d(settings->getVisualizationDelay());  

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;

  // update h value
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, settings, stats);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  int fileIndex = 0;

  // float max = 0.f;

  // continue until O empty
  while (!O.empty()) {

    if (dumpNodes)
    {             
        dumpNodesToFile(fileIndex, O);
        fileIndex++;
        dumpNodes = false;
    }

    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width, height);
    iterations++;
    stats.iterations++;

    // RViz visualization
    if (settings->getVisualizationEnabled()) {
        auto startV = stats.viz.getTime();
        visualization.publishNode3DPoses(*nPred);
        visualization.publishNode3DPose(*nPred);        
        d.sleep();      
        stats.viz.updateCallTime(startV);
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST

      if (*nPred == goal || iterations > maxIterations) {
        // DEBUG
        return nPred;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        if (settings->getDubinsShotEnabled() && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace, settings);

          if (nSucc != nullptr && nSucc->reachedGoal(goal)) {
            //DEBUG
            // std::cout << "max diff " << max << std::endl;
            return nSucc;
          }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);

          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                // calculate H value
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, settings, stats);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // if (nodes3D[iSucc].isOpen())
                // {
                //     if (nodes3D[iSucc].getC() <= nSucc->getC())
                //     {
                //         // discard new node as it already exists with lower cost
                //         delete nSucc;
                //         continue;
                //     }
                //     else
                //     {                        
                //         deleteNode3D(O, nodes3D[iSucc]);
                //     }                                        
                // }                

                nSucc->open();                
                nodes3D[iSucc] = *nSucc;                                
                O.push(&nodes3D[iSucc]);

                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualization& visualization,
            Settings* settings) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (settings->getVisualization2DEnabled()) {  
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);

                // if (nodes2D[iSucc].isOpen())
                // {
                //     if (nodes2D[iSucc].getC() <= nSucc->getC())
                //     {
                //         // discard new node as it already exists with lower cost
                //         delete nSucc;
                //         continue;
                //     }
                //     else
                //     {                        
                //         deleteNode2D(O, nodes2D[iSucc]);
                //     }                                        
                // }                
  

              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualization& visualization, Settings* settings, AlgorithmStats& stats) {
    
    auto tStart = stats.updateH.getTime();

    float dubinsCost = 0;
    float reedsSheppCost = 0;
    float twoDCost = 0;
    float twoDoffset = 0;

    // if dubins heuristic is activated calculate the shortest path
    // constrained without obstacles
    if (settings->getDubinsEnabled()) {
        ompl::base::DubinsStateSpace dubinsPath(settings->getTurningRadius());
        State* dbStart = (State*)dubinsPath.allocState();
        State* dbEnd = (State*)dubinsPath.allocState();
        dbStart->setXY(start.getX(), start.getY());
        dbStart->setYaw(start.getT());
        dbEnd->setXY(goal.getX(), goal.getY());
        dbEnd->setYaw(goal.getT());
        dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    }

    // if reversing is active use a
    if (settings->getReverseEnabled() && !settings->getDubinsEnabled()) {        
        ompl::base::ReedsSheppStateSpace reedsSheppPath(settings->getTurningRadius());
        State* rsStart = (State*)reedsSheppPath.allocState();
        State* rsEnd = (State*)reedsSheppPath.allocState();
        rsStart->setXY(start.getX(), start.getY());
        rsStart->setYaw(start.getT());
        rsEnd->setXY(goal.getX(), goal.getY());
        rsEnd->setYaw(goal.getT());
        reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);    
    }

    // if twoD heuristic is activated determine shortest path
    // unconstrained with obstacles
    if (settings->getTwoDEnabled() && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {    
        // create a 2d start node
        Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
        // create a 2d goal node
        Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
        // run 2d astar and return the cost of the cheapest path for that node
        nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization, settings));    
    }

    if (settings->getTwoDEnabled()) {
        // offset for same node in cell
        twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                        ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
        twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;

    }

    // return the maximum of the heuristics, making the heuristic admissable
    start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
    stats.updateH.updateCallTime(tStart);
}

//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace, Settings* settings) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, settings->getTurningRadius(), &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  float dubinsStepSize = settings->getDubinsStepSize();

  Node3D* dubinsNodes = new Node3D [(int)(length / dubinsStepSize) + 1];

  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }    
    
  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}

void dumpNodesToFile(int index, const boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>>& pq)
{
    std::ofstream outfile;

    std::string filename = "nodes3d";
    filename.append(std::to_string(index));
    filename.append(".txt");
    outfile.open(filename);
    outfile << "Index\tX\tY\tT\tG\tH\tC\tIdx\tPrim\tOpen\tClosed\n";
    boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>::ordered_iterator it;

    int i = 0;

    for(it = pq.ordered_begin(); it != pq.ordered_end(); ++it)
    {        
        Node3D* n = *it;
        outfile << i << "\t";
        outfile << n->getX() << "\t";
        outfile << n->getY() << "\t";
        outfile << n->getT() << "\t";
        outfile << n->getG() << "\t";
        outfile << n->getH() << "\t";
        outfile << n->getC() << "\t";
        outfile << n->getIdx() << "\t";
        outfile << n->getPrim() << "\t";
        outfile << n->isOpen() << "\t";
        outfile << n->isClosed() << "\t";
        outfile << "\n";
        i++;
    }

    outfile.close();

    
}

void checkNodes(int index, const boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq)
{
     boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>::ordered_iterator it;    

    float priorC = 0;
    bool dump = false;

    for(it = pq.ordered_begin(); it != pq.ordered_end(); ++it)
    {
        Node3D* n = *it;

        if (n->getC() < priorC)
        {
            std::cout << "Issue with heap";
            dump = true;
        }

        priorC = n->getC();
    }

    if (index < 0 && dump)
        dumpNodesToFile(index, pq);
}

void deleteNode3D(boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq, const Node3D& node)
{    
    for(auto it = pq.begin(); it != pq.end(); ++it)
    {
        Node3D* n = *it;

        if (n->getIdx() == node.getIdx())
        {
            boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>::handle_type h;
            h = boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>::s_handle_from_iterator(it);
            pq.erase(h);           
            return;
        }
    }
}

void deleteNode2D(boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>>& pq, const Node2D& node)
{    
    for(auto it = pq.begin(); it != pq.end(); ++it)
    {
        Node2D* n = *it;

        if (n->getIdx() == node.getIdx())
        {
            boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>>::handle_type h;
            h = boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>>::s_handle_from_iterator(it);
            pq.erase(h);           
            return;
        }
    }
}