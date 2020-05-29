#include "HASAlgorithm.h"

#include <chrono>
#include <iostream>
#include <fstream>

using namespace OsrPlanner;


HASAlgorithm::HASAlgorithm()
{
    pubRS = n.advertise<geometry_msgs::PoseArray>("/rsPath", 100);
    rsPoses.header.frame_id = "path";
}

//###################################################
//                                        3D A*
//###################################################
Node3D* HASAlgorithm::runAlgo(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               DynamicVoronoi& voronoiDiagram,
                               CollisionMap& collisionMap,
                               Visualization& visualization,                               
                               AlgorithmStats& stats,
                               Heuristics& heuristics) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred;  
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = settings->getReverseEnabled() ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;
  int maxIterations = settings->getIterations();
  bool dumpNodes = false;

  // VISUALIZATION DELAY
  ros::Duration d(settings->getVisualizationDelay());  
  
  pq.clear();

  // update h value  
  start.setH(heuristics.getHeuristicValue(start.getX(), start.getY(), start.getT(), goal.getX(), goal.getY(), goal.getT()));
  // mark start as open
  start.open();
  // push on priority queue aka open list
  pq.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  int fileIndex = 0;

  // float max = 0.f;

  // continue until O empty
  while (!pq.empty()) {

    if (dumpNodes)
    {             
        dumpNodesToFile(fileIndex, pq);
        fileIndex++;
        dumpNodes = false;
    }

    // pop node with lowest cost from priority queue
    nPred = pq.top();
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
      pq.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      pq.pop();

      // check if at goal or max iterations
      if (nPred->reachedGoal(goal) || iterations > maxIterations) {
        // DEBUG
        return nPred;
      }

      // iterate through possible directions of travel and process node
      for (int i = 0; i < dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          processNode(nodes3D, goal, nPred, nSucc, width, height, voronoiDiagram, collisionMap, heuristics, iPred, true);
      }

      // perform Reeds Shepp expansion
      if (nPred->isInRangeRS(goal))
      {                    
          stats.rsShots++;
          nSucc = validateRSPath(goal, nPred, collisionMap);

          if (nSucc != nullptr)
          {
            processNode(nodes3D, goal, nPred, nSucc, width, height, voronoiDiagram, collisionMap, heuristics, iPred, false);
            stats.rsShotsSuccessful++;
          }
      }            
    }
  }

  if (pq.empty()) {
    return nullptr;
  }

  return nullptr;
}

void HASAlgorithm::processNode(Node3D* nodes3D, const Node3D& goal, Node3D* nPred, Node3D* nSucc, int width, int height, DynamicVoronoi& voronoiDiagram, CollisionMap& collisionMap, Heuristics& heuristics, int& iPred, bool updateG)
{    
    // set index of the successor
    int iSucc = nSucc->setIdx(width, height);

    // ensure successor is on grid and traversable
    if (nSucc->isOnGrid(width, height) && isTraversable(nSucc, voronoiDiagram, collisionMap)) {

      // ensure successor is not on closed list or it has the same index as the predecessor
      if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

        // calculate new G value
        float vCost = voronoiDiagram.getCost(nSucc->getX(), nSucc->getY());
        
        nSucc->updateG(vCost, updateG);

        float newG = nSucc->getG();

        // if successor not on open list or found a shorter way to the cell
        if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

          // calculate H value
          nSucc->setH(heuristics.getHeuristicValue(nSucc->getX(), nSucc->getY(), nSucc->getT(), goal.getX(), goal.getY(), goal.getT()));                

          // if the successor is in the same cell but the C value is larger
          if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
            delete nSucc;
            return;
          }
          // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
          else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
            nSucc->setPred(nPred->getPred());
          }

          if (nSucc->getPred() == nSucc) {
            std::cout << "looping";
          }

          nSucc->open();                
          nodes3D[iSucc] = *nSucc;                                
          pq.push(&nodes3D[iSucc]);

          delete nSucc;
        } else { delete nSucc; }
      } else { delete nSucc; }
    } else { delete nSucc; }
}

void HASAlgorithm::dumpNodesToFile(int index, const boost::heap::binomial_heap<Node3D*,
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

void HASAlgorithm::checkNodes(int index, const boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>>& pq)
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


Node3D* HASAlgorithm::validateRSPath(const Node3D& goal, Node3D* nPred, CollisionMap& collisionMap)
{
    Node3D* nSucc = nullptr;
    ompl::base::ReedsSheppStateSpace reedsSheppPath(settings->getTurningRadius());
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();   
    State* rsPoint = (State*)reedsSheppPath.allocState();   

    rsStart->setXY(nPred->getX(), nPred->getY());
    rsStart->setYaw(nPred->getT());

    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());    

    double distance = reedsSheppPath.distance(rsStart, rsEnd);

    if (distance < 1.)
    {
      int q = 0;
    }
    distance = min(1/ distance, distance);

    double d = distance;
    bool first = true;
    float x, y , t;

    if (settings->getVisualizationEnabled())
      rsPoses.poses.clear();

    while (d < 1)
    {
      reedsSheppPath.interpolate(rsStart, rsEnd, d, rsPoint); 

      if (first)
      {
        x = rsPoint->getX();
        y = rsPoint->getY();
        t = rsPoint->getYaw();        
        first = false;        
      }

      if (settings->getVisualizationEnabled())
      {
        geometry_msgs::Pose pose;
        pose.position.x = rsPoint->getX();
        pose.position.y =  rsPoint->getY();
        pose.orientation = tf::createQuaternionMsgFromYaw(rsPoint->getYaw());
        rsPoses.poses.push_back(pose);
      }

      if (!collisionMap.isTraversable(rsPoint->getX(), rsPoint->getY(), rsPoint->getYaw()))
      {
        reedsSheppPath.freeState(rsStart);
        reedsSheppPath.freeState(rsEnd);
        reedsSheppPath.freeState(rsPoint);
        return nullptr;
      }

      d += distance;
    }

    if (settings->getVisualizationEnabled())
    {
      rsPoses.header.stamp = ros::Time::now();
      pubRS.publish(rsPoses);
    }

    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);
    reedsSheppPath.freeState(rsPoint);

    if (t < 0)
      t += 2 * M_PI;

    bool reverse = false;

    float origDistance = Helper::euclidianDistance(goal.getX(), goal.getY(), nPred->getX(), nPred->getY());
    float newDistance = Helper::euclidianDistance(goal.getX(), goal.getY(), x, y);

    if (newDistance > origDistance)
      reverse = true;

    nSucc = new Node3D(x, y, t, nPred->getG() + d, 0, nPred, settings, getDirection(reverse, nPred->getT(), t));   

    return nSucc;
}

bool HASAlgorithm::isTraversable(Node3D* node, DynamicVoronoi& voronoiDiagram, CollisionMap& collisionMap)
{
   if (voronoiDiagram.getDistance(node->getX(), node->getY()) > collisionMap.getMaxVehDistance())
    return true;

  return collisionMap.isTraversable(node);

}


int HASAlgorithm::getDirection(bool reverse, float tOrig, float tNew)
{
  //TO DO - fix 
    int dir = 0;

    if (tNew < 0 || tOrig < 0)
    {
        int z = 0;
    }

    float tDiff = tNew - tOrig;

    // check to see if potentially passing through 360 degrees
    if (std::abs(tDiff) > M_PI)
    {
      if (tNew > tOrig)
        tOrig += 2 * M_PI;
      else
        tNew += 2 * M_PI;
      
      
      tDiff = tNew - tOrig;
    }    

    if (tDiff > 0)
    {
      if (reverse)
        dir = 4;
      else
        dir = 1;
    }
    else if (tDiff < 0)
    {
      if (reverse)
        dir = 5;
      else
        dir = 2;      
    }
    else
    {
      if (reverse)
        dir = 3;
    }

    return dir;
    
}