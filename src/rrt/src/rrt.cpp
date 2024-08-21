#include "rrt.h"


RRT::RRT(Point start, Point goal, double step)
{
  _start = start;
  _goal = goal;
  _currentNode.position = start;
  _currentNode.parent = 0;
  _goalNode.position = _goal;
  _step = step;
}
RRT::~RRT(){}

void RRT::init(Point start, Point goal)
{

}

void RRT::setObastacle()
{

}

void RRT::checkObstacle()
{

}

void RRT::plan()
{
  _node_list.push_back(_currentNode);

  while(1)
  {
    std::cout << "========================" << std::endl;
    createRandomNumber();
    std::cout << LOG << "_randomNum = " << _randomNum << std::endl;
    if (_randomNum >= 0.8)
    {
      double dis;
      _index = getNearestNode();
      dis = std::hypot(std::abs(_node_list[_index].position.x - _goalNode.position.x),
                       std::abs(_node_list[_index].position.y - _goalNode.position.y));
      std::cout << LOG << "not finish path planning" << std::endl;
      if (dis < 2)
      {
        std::cout << LOG << "finish path planning" << std::endl;
        _goalNode.parent = &_node_list[_index];
        _node_list.push_back(_goalNode);
        return;
      }

      double theta = ((_goalNode.position.y - _node_list[_index].position.y) /
                      (_goalNode.position.x - _node_list[_index].position.x));
      double x,y,z;
      x = _node_list[_index].position.x + _step * cos(theta);
      y = _node_list[_index].position.y + _step * sin(theta);

      //checkObstacle();

      _currentNode.position.x = x;
      _currentNode.position.y = y;
      // _currentNode.parent = &_node_list[_index];

    }
    else
    {
      createRandomNode();
      _currentNode.position.x = _randomNodeX;
      _currentNode.position.y = _randomNodeY;
      _currentNode.position.z = _randomNodeZ;

      _index = getNearestNode();
      if (setNodeByStep(_index) == false)
      {
        continue;
      }
    }
    _currentNode.parent = &(_node_list[_index]);
    _node_list.push_back(_currentNode);
    std::cout << LOG << "_currentNode.position.x = " << _currentNode.position.x << std::endl;
    std::cout << LOG << "_currentNode.position.y = " << _currentNode.position.x << std::endl;
  }
}

void RRT::broadcastPath()
{
  int count = _node_list.size();
  Node currentNode = _goalNode;
  while (currentNode.parent != nullptr)
  {
    std::cout << LOG << "currentNode.parent = " << currentNode.parent << std::endl;
    std::cout << LOG << "currentNode.parent->position.x  = " << currentNode.parent->position.x << std::endl;
    std::cout << LOG << "currentNode.parent->position.y  = " << currentNode.parent->position.y << std::endl;
    std::cout << LOG << "count = " << count << std::endl;
    Point p;
    p.x = currentNode.position.x;
    p.y = currentNode.position.y;
    p.z = currentNode.position.z;
    std::cout << LOG << (currentNode.parent == 0) << std::endl;
    // std::cout << LOG << "=====1111====" <<  _node_list[0].parent << std::endl;
    if (currentNode.parent == 0)
    return;
    currentNode = *currentNode.parent;

    _path.push_back(p);
    count--;
  }
  return;
  // for (short i = 0; i <_node_list.size(); i++)
  // {
  //   Point p;
  //   p.x = _node_list[i].position.x;
  //   p.y = _node_list[i].position.y;
  //   p.z = _node_list[i].position.z;
  //   _path.push_back(p);
  // }

}

void RRT::broadcastAllPath()
{
  for (short i = 0; i < _node_list.size(); i++)
  {
    Point p;
    p.x = _node_list[i].position.x;
    p.y = _node_list[i].position.y;
    p.z = _node_list[i].position.z;
    _allPath.push_back(p);
  }

}

void RRT::createRandomNumber()
{
  //random number generation
  std::random_device goal_rd;
  std::mt19937 goal_engine(goal_rd());
  std::uniform_int_distribution<int> goal_dis(0, 100);

  _randomNum = double(goal_dis(goal_engine)) / 100;
  std::cout << LOG << "_randomNum = " << _randomNum << std::endl;
}

void RRT::createRandomNode()
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

bool RRT::setNodeByStep(int index)
{
  double theta = ((_currentNode.position.y - _node_list[index].position.y) /
                  (_currentNode.position.x - _node_list[index].position.x));
  std::cout << LOG << "theta = " << theta << std::endl;
  double x,y,z;
  x = _node_list[index].position.x + _step * cos(theta);
  y = _node_list[index].position.y + _step * sin(theta);
  std::cout << LOG << "cos(thetha) * step = " << _step * cos(theta) << std::endl;
  std::cout << LOG << "sin(thetha) * step = " << _step * sin(theta) << std::endl;
  std::cout << LOG << "x = " << x << ", y = " << y << std::endl;
  // std::cout << "x = " << (-15.0 <= x <= 15.0) << std::endl;
  // std::cout << "y = " << (-15.0 <= y <= 15.0) << std::endl;
  //checkObstacle();
  if ((-15.0 <= x) && (x <= 15.0)&&
      (-15.0 <= y) && (y <= 15.0))
  {
    _currentNode.position.x = x;
    _currentNode.position.y = y;
    std::cout << LOG << "return true" << std::endl;
    return true;
  }
  std::cout << LOG << "return false" << std::endl;
  return false;


}

int RRT::getNearestNode()
{
  short index = 0;
  double min_dis = std::numeric_limits<double>::max();
  double dis = std::numeric_limits<double>::max();
  for (short i = 0; i < _node_list.size(); i++)
  {
    dis = std::hypot(std::abs(_node_list[i].position.x - _currentNode.position.x),
                    (std::abs(_node_list[i].position.y - _currentNode.position.y)));
    if (dis < min_dis)
    {
      min_dis = dis;
      index = i;
    }
  }
  return index;
}