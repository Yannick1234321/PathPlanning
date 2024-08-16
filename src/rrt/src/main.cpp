#include "rrt.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_node");
  ros::NodeHandle n("");
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 1);
  Point start, goal;
  start.x = 0.0;
  start.y = 0.0;
  goal.x = 10.0;
  goal.y = 10.0;
  RRT rrtPlanner(start, goal, 0.2);
  rrtPlanner.plan();
  std::cout << "1111" << std::endl;
  rrtPlanner.broadcastPath();
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  std::cout << LOG << "rrtPlanner._path.size() = " << rrtPlanner._path.size() << std::endl;
  for(short i = 0; i < rrtPlanner._path.size(); i++)
  {
    std::cout << "i = " << i << std::endl;
    path.poses[i].pose.position.x = rrtPlanner._path[i].x;
    path.poses[i].pose.position.y = rrtPlanner._path[i].y;

  }
  std::cout << LOG <<"2222" << std::endl;
  path_pub.publish(path);
  std::cout << LOG <<"3333" << std::endl;
  while(1)
  {
    sleep(2);
  }
  return 0;
}
