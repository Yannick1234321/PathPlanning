#include "rrt_star_grid.h"
#include <cmath>
RRTSTARGRID::RRTSTARGRID(ros::NodeHandle& nh, const double step, const double near_area_raduis)
{
  _n = nh;
  _step = step;
  areaDis = near_area_raduis;

  // std::cout << LOG << "start.x = " << _startNode->position.x << std::endl;
  // std::cout << LOG << "start.y = " << _startNode->position.y << std::endl;
  // std::cout << LOG << "goal.x = " << _goalNode->position.x << std::endl;
  // std::cout << LOG << "goal.y = " << _goalNode->position.y << std::endl;
  std::cout << LOG << "_step = " << _step << std::endl;
  std::cout << LOG << "areaDis = " << areaDis << std::endl;

}
RRTSTARGRID::~RRTSTARGRID(){}

void RRTSTARGRID::init(Point start, Point goal)
{

}
void RRTSTARGRID::setStartPoint(Node& start)
{
  _startNode->position.x = start.position.x;
  _startNode->position.y = start.position.y;
  _startFlag = true;
  std::cout << LOG << "Received Strat Point." << std::endl;
  std::cout << LOG << "StartPoint : x = " << _startNode->position.x << std::endl;
  std::cout << LOG << "StartPoint : y = " << _startNode->position.y << std::endl;

}
void RRTSTARGRID::setGoal(Node& goal)
{
  _goalNode->position.x = goal.position.x;
  _goalNode->position.y = goal.position.y;
  _goalFlag = true;
  std::cout << LOG << "Received Goal Point." << std::endl;
  std::cout << LOG << "GoalPoint : x = " << _goalNode->position.x << std::endl;
  std::cout << LOG << "GoalPoint : y = " << _goalNode->position.y << std::endl;

}
bool RRTSTARGRID::isInitFinished()
{
  if ((_startFlag == true) && (_goalFlag == true) && (_mapFlag == true))
  {
    return true;
  }
  return false;
}
void RRTSTARGRID::resetInitStartGoal()
{
  _startFlag = false;
  _goalFlag = false;
  std::cout << "_node_list.capacity() = " << _node_list.capacity() << std::endl;
  for (long unsigned int i = 0; i < _node_list.size(); i++)
  {
    delete _node_list[i];
    _node_list[i] = nullptr;
  }
  
  std::cout << "_node_list.capacity() = " << _node_list.capacity() << std::endl;

  _node_list.clear();
  std::cout << "_node_list.capacity() = " << _node_list.capacity() << std::endl;
  _node_list.shrink_to_fit();
  std::cout << "_node_list.capacity() = " << _node_list.capacity() << std::endl;
  _path.clear();
  _allPath.clear();

}

void RRTSTARGRID::setMapFlag()
{
  _mapFlag = true;
}

void RRTSTARGRID::setObstacle(const double index_x, const double index_y, const int value)
{
  uint32_t index = index_y * _GLX_SIZE + index_x;
  _map[(int)index] = value;
}

bool RRTSTARGRID::isObstacleFree(const Node* nearestNode, const Node* newNode)
{

  Eigen::Vector2d pt_nearestNode(nearestNode->position.x, nearestNode->position.y);
  Eigen::Vector2d pt_newNode(newNode->position.x, newNode->position.y);
  Eigen::Vector2i idx_nearestNode = coord2gridIndex(pt_nearestNode);
  Eigen::Vector2i idx_newNode = coord2gridIndex(pt_newNode);

  if (idx_nearestNode(0) == idx_newNode(0)&&        //两点重合，舍弃
      idx_nearestNode(1) == idx_newNode(1))
  {
    return false;
  }

  int x_min, x_max, y_min, y_max;
  //赋值x
  if (idx_nearestNode(0) <= idx_newNode(0))
  {
    x_min = idx_nearestNode(0);
    x_max = idx_newNode(0);
  }
  else if(idx_nearestNode(0) > idx_newNode(0))
  {
    x_min = idx_newNode(0);
    x_max = idx_nearestNode(0);
  }
//赋值y，用于比较
  if (idx_nearestNode(1) <= idx_newNode(1))
  {
    y_min = idx_nearestNode(1);
    y_max = idx_newNode(1);
  }
  else if(idx_nearestNode(1) > idx_newNode(1))
  {
    y_min = idx_newNode(1);
    y_max = idx_nearestNode(1);
  }
  int value;
  //判断两个栅格之间是否存在障碍物
  if ( (y_min == y_max))
  {
    for (int i = x_min; i < x_max; i++)
    {
      uint32_t index = y_min * _GLX_SIZE + i;
      value = _map[index];
      if (value != 0)
      {
        return false;
      }
    }
  }
  else if (x_min == x_max)
  {
    for (int i = y_min; i < y_max; i++)
    {
      uint32_t index = i * _GLX_SIZE + x_min;
      value = _map[index];
      if (value != 0)
      {
        return false;
      }
    }
  }
  else
  {
    if (x_min != 0) x_min = x_min - 1;
    if (x_max != _GLX_SIZE - 1) x_max = x_max + 1;
    if (y_min != 0) y_min = y_min - 1;
    if (y_max != _GLY_SIZE - 1) y_max = y_max + 1;
    for (int i = x_min -1; i <= x_max + 1; i++)
    {
      for (int j = y_min -1; j <= y_max + 1; j++)
      {
        uint32_t index = j * _GLX_SIZE + i;
        value = _map[index];
        if (value != 0)
        {
          return false;
        }
      }
    }

  }
  return true;
}

void RRTSTARGRID::rewire(Node* newNode, std::vector<int> near_idx)
{
  for (long unsigned int i = 0; i < near_idx.size(); i++)
  {
    if (_node_list[near_idx[i]]->parent == nullptr)
    continue;
    if (isObstacleFree(_node_list[near_idx[i]], newNode) == false)
    {
      continue;
    }
    double newNodeAsParentNodeCost = newNode->cost + calNodeDis(_node_list[near_idx[i]], newNode);
    double origenNodeParentCost = _node_list[near_idx[i]]->cost;
    if (newNodeAsParentNodeCost < origenNodeParentCost)
    {
      _node_list[near_idx[i]]->parent = newNode;
      _node_list[near_idx[i]]->cost = newNodeAsParentNodeCost;
    }
  }
}

void RRTSTARGRID::chooseParent(Node* newNode, std::vector<int> near_idx)
{
  double min_cost = std::numeric_limits<double>::max();
  int min_idx;
  for (long unsigned int i = 0; i < near_idx.size(); i++)
  {
    double dis = calNodeDis(newNode, _node_list[near_idx[i]]);
    double all_cost = _node_list[near_idx[i]]->cost + dis;

    if (all_cost < min_cost)
    {
      min_cost = all_cost;
      min_idx = near_idx[i];
    }
  }
  newNode->parent = _node_list[min_idx];
  newNode->cost = min_cost;

}

std::vector<int> RRTSTARGRID::findNearestIndexs(Node* newNode)
{
  std::vector<int> near_idx;
  for (long unsigned int i = 0; i < _node_list.size(); i++)
  {
    double dis = calNodeDis(newNode, _node_list[i]);
    if (dis < areaDis)
    {
      near_idx.push_back(i);
    }
  }
  return near_idx;
}

double RRTSTARGRID::calNodeDis(Node* node1, Node* node2)
{
  double dis = std::hypot(std::abs(node1->position.x - node2->position.x),
                         (std::abs(node1->position.y - node2->position.y)));
  return dis;
}

// void RRTSTARGRID::plan(ros::NodeHandle& n)
void RRTSTARGRID::plan()
{
  ros::Publisher pub_checked = _n.advertise<nav_msgs::Odometry>("/point_checked", 1);
  _node_list.push_back(_startNode);

  while(1)
  {
    createRandomNumber();
    Node* nearestNode;
    Node* newNode = new Node;


    if (_randomNum >= 0.9)
    {
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
      // delete newNode;
      // newNode = nullptr;
      continue;
    }

    if (isObstacleFree(nearestNode, newNode) == false)
    {
      // delete newNode;
      // newNode = nullptr;
      continue;
    }

    std::vector<int> near_idx = findNearestIndexs(newNode);

    chooseParent(newNode, near_idx);
    // rewire(newNode, near_idx);
    // newNode->parent = nearestNode;
    // Node* cpyNode = new Node(newNode);
    // _node_list.push_back(cpyNode);
    _node_list.push_back(newNode);
    // delete newNode;
    // newNode = nullptr;

    nav_msgs::Odometry point;
    point.pose.pose.position.x = newNode->position.x;
    point.pose.pose.position.y = newNode->position.y;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.child_frame_id = "base_link";
    pub_checked.publish(point);
  }
  std::cout << LOG << "find valid path" << std::endl;
}

void RRTSTARGRID::broadcastPath()
{
  int count = _node_list.size();
  Node* currentNode = _goalNode;
  while (currentNode->parent != nullptr)
  {
    // std::cout << LOG << "currentNode.parent = " << currentNode->parent << std::endl;
    // std::cout << LOG << "currentNode.parent->position.x  = " << currentNode->parent->position.x << std::endl;
    // std::cout << LOG << "currentNode.parent->position.y  = " << currentNode->parent->position.y << std::endl;
    // std::cout << LOG << "count = " << count << std::endl;
    Point p;
    p.x = currentNode->position.x;
    p.y = currentNode->position.y;
    p.z = currentNode->position.z;
    if (currentNode->parent == 0)
    return;
    currentNode = currentNode->parent;

    _path.push_back(p);
    count--;
  }
  return;

}

void RRTSTARGRID::broadcastAllPath()
{
  for (long unsigned int i = 0; i < _node_list.size(); i++)
  {
    Point p;
    p.x = _node_list[i]->position.x;
    p.y = _node_list[i]->position.y;
    p.z = _node_list[i]->position.z;
    _allPath.push_back(p);
  }

}

void RRTSTARGRID::createRandomNumber()
{
  //random number generation
  std::random_device goal_rd;
  std::mt19937 goal_engine(goal_rd());
  std::uniform_int_distribution<int> goal_dis(0, 100);

  _randomNum = double(goal_dis(goal_engine)) / 100;
}

void RRTSTARGRID::createRandomNode()
{
  std::random_device area_rd;
  std::mt19937 area_engine(area_rd());
  std::uniform_int_distribution<int> area_dis_x(_size_x_min * 10, _size_x_max * 10);
  std::uniform_int_distribution<int> area_dis_y(_size_y_min * 10, _size_y_max * 10);

  _randomNodeX = double(area_dis_x(area_engine))/10;
  _randomNodeY = double(area_dis_y(area_engine))/10;
  // _randomNodeZ = double(area_dis(area_engine))/10;

}

bool RRTSTARGRID::setNodeByStep(Node* nearestNode, Node* newNode)
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
  double x,y;
  x = nearestNode->position.x + _step * cos(theta);
  y = nearestNode->position.y + _step * sin(theta);
  //checkObstacle();
  if ((_size_x_min <= x) && (x <= _size_x_max)&&
      (_size_y_min <= y) && (y <= _size_y_max))
  {
    newNode->position.x = x;
    newNode->position.y = y;
    newNode->cost = nearestNode->cost + _step;

    return true;
  }
  return false;


}

Node* RRTSTARGRID::getNearestNode(Node* node)
{
  short index = 0;
  double min_dis = std::numeric_limits<double>::max();
  double dis = std::numeric_limits<double>::max();
  for (long unsigned int i = 0; i < _node_list.size(); i++)
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

bool RRTSTARGRID::ifArrivedGoal(Node* nearestNode)
{
  double dis = std::hypot(std::abs(nearestNode->position.x - _goalNode->position.x),
                          std::abs(nearestNode->position.y - _goalNode->position.y));
  if (dis < 0.3)
  {
    _goalNode->parent = nearestNode;
    _node_list.push_back(_goalNode);
    return true;
  }
  return false;
}

void RRTSTARGRID::mapInit(const double resolution,
                          const double origin_x,
                          const double origin_y,
                          const double size_x_min,
                          const double size_x_max,
                          const double size_y_min,
                          const double size_y_max,
                          const double GLX_SIZE,
                          const double GLY_SIZE)
{
  _resolution = resolution;
  _inv_resolution = 1 / _resolution;
  _size_x_min = size_x_min;
  _size_x_max = size_x_max;
  _size_y_min = size_y_min;
  _size_y_max = size_y_max;
  _origin_x = origin_x;                //OccupancyGrid.info.origin.position.为东北坐标系下的偏移量
  _origin_y = origin_y;                //实际的原点坐标是在该值取负，表示东北坐标系下的实际坐标
  _GLX_SIZE = GLX_SIZE;
  _GLY_SIZE = GLY_SIZE;
  _GLXY_SIZE = _GLX_SIZE * _GLY_SIZE;
  std::cout << "_GLXY_SIZE = " << _GLXY_SIZE << std::endl;
  _map = new int[_GLXY_SIZE];
  std::memset(_map, -1, _GLXY_SIZE * sizeof(int));

  return;
}

GridPosition RRTSTARGRID::nodeRasterized(Node* node)
{
  GridPosition gp;
  gp.height = std::round(node->position.y) - _size_y_min;
  gp.width = std::round(node->position.x) - _size_x_min;
  return gp;
}
void RRTSTARGRID::pgmToOccupancyGrid(const std::string& pgm_file, nav_msgs::OccupancyGrid& grid)
{
  cv::Mat image = cv::imread(pgm_file, cv::IMREAD_GRAYSCALE);
  grid.data.resize(image.rows * image.cols, 0);
  for (int i = 0; i < image.rows; ++i)
  {
    for (int j = 0; j < image.cols; ++j)
    {
      grid.data[i * image.cols + j] = image.at<uchar>(i, j);
    }
  }
}

//将大地坐标系，转换为栅格地图的index序列号,并四舍五入
Eigen::Vector2i RRTSTARGRID::coord2gridIndex(const Eigen::Vector2d pt)
{
  Eigen::Vector2i idx;
  idx(0) = std::round(((pt(0) - _origin_x)/_resolution));
  idx(1) = std::round(((pt(1) - _origin_y)/_resolution));
  return idx;

}

Eigen::Vector2d RRTSTARGRID::gridIndex2coord(const Eigen::Vector2i idx)
{
  Eigen::Vector2d pt;

  return pt;
}