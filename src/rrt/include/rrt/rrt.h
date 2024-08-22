#include <vector>
#include <random>
#include <iostream>

#define LOG "[" << __FUNCTION__ << "][" << __LINE__ << "]"
struct Point
{
  double x;
  double y;
  double z;
};

struct Node
{
  Point position;
  Node *parent;
};

class RRT
{
public:
  std::vector<Point> _path;
  std::vector<Point> _allPath;

  RRT(Point start, Point goal, double step);
  void init(Point start, Point goal);
  void plan();
  void broadcastPath();
  void broadcastAllPath();
  ~RRT();

private:
  std::vector<Node> _node_list;
  Point _start;
  Point _goal;
  Node _currentNode;
  Node _goalNode;

  double _step;
  int _index;

  double _randomNum;
  double _randomNodeX;
  double _randomNodeY;
  double _randomNodeZ;

  // 随机函数产生的是一种伪随机数，它实际是一种序列发生器，有固定的算法，只有当种子不同时，序列才不同，
  // 所以不应该把种子固定在程序中，应该用随机产生的数做种子，如程序运行时的时间等。
//   std::random_device goal_rd;                   // random_device可以生成用来作为种子的随机的无符号整数值。
//   std::mt19937 goal_gen;                        // mt19937是一种高效的随机数生成算法
//   std::uniform_int_distribution<int> goal_dis;  // 随机数源，随机数源调用随机数算法来生成随机数

//   std::random_device area_rd;
//   std::mt19937 area_rd;
//   std::uniform_int_distribution<double> area_dis;

  void setObastacle();
  void checkObstacle();
  void createRandomNumber();
  void createRandomNode();
  bool setNodeByStep(Node);
  Node getNearestNode(Node);
  bool ifArrivedGoal(Node);
};
