#include "rrt_star.h"
#include <cmath>

RRTSTAR::RRTSTAR(Node &start, Node &goal, const double step, const double size_x_min, const double size_x_max,
                 const double size_y_min, const double size_y_max, const double near_area_raduis)
{
  _startNode = &start;
  _goalNode = &goal;
  _step = step;
  _size_x_min = size_x_min;
  _size_x_max = size_x_max;
  _size_y_min = size_y_min;
  _size_y_max = size_y_max;
  areaDis = near_area_raduis;

  std::cout << LOG << "start.x = " << _startNode->position.x << std::endl;
  std::cout << LOG << "start.y = " << _startNode->position.y << std::endl;
  std::cout << LOG << "goal.x = " << _goalNode->position.x << std::endl;
  std::cout << LOG << "goal.y = " << _goalNode->position.y << std::endl;
  std::cout << LOG << "_step = " << _step << std::endl;
  std::cout << LOG << "_size_x_min = " << _size_x_min << std::endl;
  std::cout << LOG << "_size_x_max = " << _size_x_max << std::endl;
  std::cout << LOG << "_size_y_min = " << _size_y_min << std::endl;
  std::cout << LOG << "_size_y_max = " << _size_y_max << std::endl;
  std::cout << LOG << "areaDis = " << areaDis << std::endl;

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

void RRTSTAR::rewire(Node* newNode, std::vector<int> near_idx)
{
  for (short i = 0; i < near_idx.size(); i++)
  {
    if (_node_list[near_idx[i]]->parent == nullptr)
    continue;
    double newNodeAsParentNodeCost = newNode->cost + calNodeDis(_node_list[near_idx[i]], newNode);
    double origenNodeParentCost = _node_list[near_idx[i]]->cost;
    std::cout << LOG << "newNodeAsParentNodeCost = " << newNodeAsParentNodeCost << std::endl;
    std::cout << LOG << "origenNodeParentCost = " << origenNodeParentCost << std::endl;
    if (newNodeAsParentNodeCost < origenNodeParentCost)
    {
      _node_list[near_idx[i]]->parent = newNode;
      _node_list[near_idx[i]]->cost = newNodeAsParentNodeCost;
    }
  }
}

void RRTSTAR::chooseParent(Node* newNode, std::vector<int> near_idx)
{
  double min_cost = std::numeric_limits<double>::max();
  int min_idx;
  for (short i = 0; i < near_idx.size(); i++)
  {
    double dis = calNodeDis(newNode, _node_list[near_idx[i]]);
    double all_cost = _node_list[near_idx[i]]->cost + dis;

    std::cout << LOG << "dis = " << dis << std::endl;
    std::cout << LOG << "_node_list[near_idx[i]]->cost = " << _node_list[near_idx[i]]->cost << std::endl;
    std::cout << LOG << "all_cost = " << all_cost << std::endl;
    if (all_cost < min_cost)
    {
      min_cost = all_cost;
      min_idx = near_idx[i];
    }
  }
  std::cout << LOG << "min_cost = " << min_cost << std::endl;
  std::cout << LOG << "min_idx = " << min_idx << std::endl;
  newNode->parent = _node_list[min_idx];
  newNode->cost = min_cost;

}

std::vector<int> RRTSTAR::findNearestIndexs(Node* newNode)
{
  std::vector<int> near_idx;
  for (short i = 0; i < _node_list.size(); i++)
  {
    double dis = calNodeDis(newNode, _node_list[i]);
    if (dis < areaDis)
    {
      near_idx.push_back(i);
    }
  }
  return near_idx;
}

double RRTSTAR::calNodeDis(Node* node1, Node* node2)
{
  double dis = std::hypot(std::abs(node1->position.x - node2->position.x),
                         (std::abs(node1->position.y - node2->position.y)));
  return dis;
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
    std::vector<int> near_idx = findNearestIndexs(newNode);

    chooseParent(newNode, near_idx);
    rewire(newNode, near_idx);
    // newNode->parent = nearestNode;
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
  std::uniform_int_distribution<int> area_dis_x(_size_x_min * 10, _size_x_max * 10);
  std::uniform_int_distribution<int> area_dis_y(_size_y_min * 10, _size_y_max * 10);

  _randomNodeX = double(area_dis_x(area_engine))/10;
  _randomNodeY = double(area_dis_y(area_engine))/10;
  // _randomNodeZ = double(area_dis(area_engine))/10;

  std::cout << LOG << "_randomNodeX = " << _randomNodeX << std::endl;
  std::cout << LOG << "_randomNodeY = " << _randomNodeY << std::endl;
}

bool RRTSTAR::setNodeByStep(Node* nearestNode, Node* newNode)
{
  if ((calNodeDis(nearestNode, newNode) < _step)&&
      (_size_x_min <= newNode->position.x)&&
      (_size_y_min <= newNode->position.y)&&
      (newNode->position.x <= _size_x_max)&&
      (newNode->position.y <= _size_y_max))
  {
    return true;
  }
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
  if ((_size_x_min <= x) && (x <= _size_x_max)&&
      (_size_y_min <= y) && (y <= _size_y_max))
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
      node->cost += min_dis;
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