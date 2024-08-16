#include "rrt.h"


RRT::RRT(Point start, Point goal, double step)
{
  _start = start;
  _goal = goal;
  _currentNode.position = start;
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
      dis = std::hypot(std::abs(_node_list.back().position.x - _goalNode.position.x),
                      std::abs(_node_list.back().position.y - _goalNode.position.y));
      std::cout << LOG << "not finish path planning" << std::endl;
      if (dis < 2)
      {
        std::cout << LOG << "finish path planning" << std::endl;
        _node_list.push_back(_goalNode);
        return;
      }

      double theta = ((_goalNode.position.y - _node_list.back().position.y) /
                      (_goalNode.position.x - _node_list.back().position.x));
      double x,y,z;
      x = _node_list.back().position.x + _step * cos(theta);
      y = _node_list.back().position.y + _step * sin(theta);

      //checkObstacle();

      _currentNode.position.x = x;
      _currentNode.position.y = y;


    }
    else
    {
      createRandomNode();
      _currentNode.position.x = _randomNodeX;
      _currentNode.position.y = _randomNodeY;
      _currentNode.position.z = _randomNodeZ;

      if (setNodeByStep() == false)
      {
        continue;
      }
    }
    _currentNode.parent = &(_node_list.back());
    _node_list.push_back(_currentNode);
    std::cout << LOG << "_currentNode.position.x = " << _currentNode.position.x << std::endl;
    std::cout << LOG << "_currentNode.position.y = " << _currentNode.position.x << std::endl;
  }
}

void RRT::broadcastPath()
{
  for (short i = 0; i <_node_list.size(); i++)
  {
    Point p;
    p.x = _node_list[i].position.x;
    p.y = _node_list[i].position.y;
    p.z = _node_list[i].position.z;
    _path.push_back(p);
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

bool RRT::setNodeByStep()
{
  double theta = ((_currentNode.position.y - _node_list.back().position.y) /
                  (_currentNode.position.x - _node_list.back().position.x));
  std::cout << LOG << "theta = " << theta << std::endl;
  double x,y,z;
  x = _node_list.back().position.x + _step * cos(theta);
  y = _node_list.back().position.y + _step * sin(theta);
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