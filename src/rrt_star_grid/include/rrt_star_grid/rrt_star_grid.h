#ifndef __RRT_STAR_GRID_H__
#define __RRT_STAR_GRID_H__

#include <vector>
#include <random>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <malloc.h>


#define LOG "[" << __FUNCTION__ << "][" << __LINE__ << "]"
enum GridProperty
{
  obstacle = 0,
  empty,
  inNodeList
};
struct GridPosition
{
  int height;
  int width;
  GridPosition() : height(0), width(0) {}
  ~GridPosition() {}
};
struct Point
{
  double x;
  double y;
  double z;
  Point() : x(0), y(0), z(0) {}
  ~Point() {}
};

struct Node
{
  Point position;
  Node *parent;
  double cost;
  Node() : parent(nullptr), cost(0) {}
  Node(const Node* otherNode) : position(otherNode->position), parent(nullptr), cost(otherNode->cost)
  {
    if (otherNode->parent!= nullptr)
    {
      parent = otherNode->parent;
    }
  }
  ~Node()
  {
    // if (parent!= nullptr)
    // {
    //   delete parent;
    //   parent = nullptr;
    // }
  }
  // Node* operator=(const Node& other)
  // {
  //   position = other.position;
  //   *parent = new Node(other.parent);
  //   cost = other.cost;
  //   return this;
  // }
};

class RRTSTARGRID
{
public:
  std::vector<Point> _path;
  std::vector<Point> _allPath;

  RRTSTARGRID(ros::NodeHandle& n, const double step, const double near_area_raduis);
  // RRTSTARGRID(Node &start, Node &goal, const double step, const double near_area_raduis);
  void init(Point start, Point goal);
  void plan();
  // void plan(ros::NodeHandle& n);
  void broadcastPath();
  void broadcastAllPath();
  void mapInit(const double resolution,
               const double origin_x,
               const double origin_y,
               const double size_x_min,
               const double size_x_max,
               const double size_y_min,
               const double size_y_max,
               const double GLX_SIZE,
               const double GLY_SIZE);
  void setObstacle(const double index_x, const double index_y, const int value);
  void setStartPoint(Node& start);
  void setGoal(Node& goal);
  void resetInitStartGoal();
  void setMapFlag();
  bool isInitFinished();
  ~RRTSTARGRID();

private:
  ros::NodeHandle _n;
  std::vector<Node*> _node_list;
  // Node* newNode;
  Node* _startNode = new Node;
  Node* _goalNode = new Node;
  bool _startFlag = false;
  bool _goalFlag = false;
  bool _mapFlag = false;

  double _step;
  double _size_x_min;
  double _size_x_max;
  double _size_y_min;
  double _size_y_max;
  int _index;
  double areaDis = 3;

  double _randomNum;
  double _randomNodeX;
  double _randomNodeY;
  double _randomNodeZ;


  //栅格化rasterize
  double _resolution, _inv_resolution;    //分辨率  分辨率个数/每米
  int _GLX_SIZE, _GLY_SIZE;               //边界长度（单位，像素个数）
  int _GLXY_SIZE;                         //XY坐标系面积（单位，像素个数^2)
  int * _map;
  double _origin_x, _origin_y;              //地图原点信息

  int _width;
  int _height;



  // 随机函数产生的是一种伪随机数，它实际是一种序列发生器，有固定的算法，只有当种子不同时，序列才不同，
  // 所以不应该把种子固定在程序中，应该用随机产生的数做种子，如程序运行时的时间等。
//   std::random_device goal_rd;                   // random_device可以生成用来作为种子的随机的无符号整数值。
//   std::mt19937 goal_gen;                        // mt19937是一种高效的随机数生成算法
//   std::uniform_int_distribution<int> goal_dis;  // 随机数源，随机数源调用随机数算法来生成随机数

//   std::random_device area_rd;
//   std::mt19937 area_rd;
//   std::uniform_int_distribution<double> area_dis;

  bool isObstacleFree(const Node* nearestNode, const Node* newNode);
  void createRandomNumber();
  void createRandomNode();
  bool setNodeByStep(Node*, Node*);
  Node* getNearestNode(Node*);
  std::vector<int> findNearestIndexs(Node*);
  bool ifArrivedGoal(Node*);
  void rewire(Node*, std::vector<int>);
  void chooseParent(Node*, std::vector<int>);
  double calNodeDis(Node*, Node*);
  GridPosition nodeRasterized(Node*);
  void pgmToOccupancyGrid(const std::string& pgm_file, nav_msgs::OccupancyGrid& grid);
  //栅格化rasterize
  Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d);
  Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i);
};


#endif // __RRT_STAR_GRID_H__