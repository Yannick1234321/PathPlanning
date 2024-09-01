#include "rrt_star.h"
#include <cmath>

RRTSTAR::RRTSTAR(Node &start, Node &goal, const double step)
{
  _startNode = &start;
  _goalNode = &goal;
  _step = step;
}
RRTSTAR::~RRTSTAR(){}

void RRTSTAR::init(Point start, Point goal)
{

}

void RRTSTAR::setObastacle()
{

}

void RRTSTAR::checkObstacle()
{

}

bool RRTSTAR::rewire()
{

}

bool RRTSTAR::chooseParent()
{

}

void RRTSTAR::plan()
{
  _node_list.push_back(_startNode);

  while(1)
  {
    std::cout << "========================" << std::endl;
    createRandomNumber();
    std::cout << LOG << "_randomNum = " << _randomNum << std::endl;
    Node* nearestNode;
    Node* newNode = new Node;
    if (_randomNum >= 0.8)
    {
      double dis;
      nearestNode = getNearestNode(_goalNode);
      if (ifArrivedGoal(nearestNode) == true)
      {
        std::cout << LOG << "find valid path" << std::endl;
        break;
      }
      newNode->position.x = _goalNode->position.x;
      newNode->position.y = _goalNode->position.y;
      newNode->position.z = _goalNode->position.z;

    }
    else
    {
      createRandomNode();
      newNode->position.x = _randomNodeX;
      newNode->position.y = _randomNodeY;
      newNode->position.z = _randomNodeZ;

      nearestNode = getNearestNode(newNode);
    }

    if (setNodeByStep(nearestNode, newNode) == false)
    {
      continue;
    }
    newNode->parent = nearestNode;
    _node_list.push_back(newNode);
    std::cout << LOG << "newNode.position.x = " << newNode->position.x << std::endl;
    std::cout << LOG << "newNode.position.y = " << newNode->position.x << std::endl;
  }
}

void RRTSTAR::broadcastPath()
{
  int count = _node_list.size();
  Node* currentNode = _goalNode;
  while (currentNode->parent != nullptr)
  {
    std::cout << LOG << "currentNode.parent = " << currentNode->parent << std::endl;
    std::cout << LOG << "currentNode.parent->position.x  = " << currentNode->parent->position.x << std::endl;
    std::cout << LOG << "currentNode.parent->position.y  = " << currentNode->parent->position.y << std::endl;
    std::cout << LOG << "count = " << count << std::endl;
    Point p;
    p.x = currentNode->position.x;
    p.y = currentNode->position.y;
    p.z = currentNode->position.z;
    std::cout << LOG << (currentNode->parent == 0) << std::endl;
    if (currentNode->parent == 0)
    return;
    currentNode = currentNode->parent;

    _path.push_back(p);
    count--;
  }
  return;

}

void RRTSTAR::broadcastAllPath()
{
  for (short i = 0; i < _node_list.size(); i++)
  {
    Point p;
    p.x = _node_list[i]->position.x;
    p.y = _node_list[i]->position.y;
    p.z = _node_list[i]->position.z;
    _allPath.push_back(p);
  }

}

void RRTSTAR::createRandomNumber()
{
  //random number generation
  std::random_device goal_rd;
  std::mt19937 goal_engine(goal_rd());
  std::uniform_int_distribution<int> goal_dis(0, 100);

  _randomNum = double(goal_dis(goal_engine)) / 100;
  std::cout << LOG << "_randomNum = " << _randomNum << std::endl;
}

void RRTSTAR::createRandomNode()
{
  std::random_device area_rd;
  std::mt19937 area_engine(area_rd());
  std::uniform_int_distribution<int> area_dis(-150, 150);

  _randomNodeX = double(area_dis(area_engine))/10;
  _randomNodeY = double(area_dis(area_engine))/10;
  _randomNodeZ = double(area_dis(area_engine))/10;

  std::cout << LOG << "_randomNodeX = " << _randomNodeX << std::endl;
  std::cout << LOG << "_randomNodeY = " << _randomNodeY << std::endl;
}

bool RRTSTAR::setNodeByStep(Node* nearestNode, Node* newNode)
{
  double theta = std::atan2((newNode->position.y - nearestNode->position.y),
                            (newNode->position.x - nearestNode->position.x));
  std::cout << LOG << "theta = " << theta << std::endl;
  double x,y,z;
  x = nearestNode->position.x + _step * cos(theta);
  y = nearestNode->position.y + _step * sin(theta);
  std::cout << LOG << "cos(thetha) * step = " << _step * cos(theta) << std::endl;
  std::cout << LOG << "sin(thetha) * step = " << _step * sin(theta) << std::endl;
  std::cout << LOG << "x = " << x << ", y = " << y << std::endl;
  //checkObstacle();
  if ((-15.0 <= x) && (x <= 15.0)&&
      (-15.0 <= y) && (y <= 15.0))
  {
    newNode->position.x = x;
    newNode->position.y = y;
    std::cout << LOG << "return true" << std::endl;
    return true;
  }
  std::cout << LOG << "return false" << std::endl;
  return false;


}

Node* RRTSTAR::getNearestNode(Node* node)
{
  short index = 0;
  double min_dis = std::numeric_limits<double>::max();
  double dis = std::numeric_limits<double>::max();
  for (short i = 0; i < _node_list.size(); i++)
  {
    dis = std::hypot(std::abs(_node_list[i]->position.x - node->position.x),
                    (std::abs(_node_list[i]->position.y - node->position.y)));
    if (dis < min_dis)
    {
      min_dis = dis;
      index = i;
    }
  }
  return _node_list[index];
}

bool RRTSTAR::ifArrivedGoal(Node* nearestNode)
{
  double dis = std::hypot(std::abs(nearestNode->position.x - _goalNode->position.x),
                          std::abs(nearestNode->position.y - _goalNode->position.y));
  std::cout << LOG << "not finish path planning" << std::endl;
  if (dis < 2)
  {
  std::cout << LOG << "finish path planning" << std::endl;
  _goalNode->parent = nearestNode;
  _node_list.push_back(_goalNode);
  return true;
  }
}