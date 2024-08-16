#include "rrt.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

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
  rrtPlanner.broadcastPath();
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  std::cout << LOG << "rrtPlanner._path.size() = " << rrtPlanner._path.size() << std::endl;
  for(short i = 0; i < rrtPlanner._path.size(); i++)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = rrtPlanner._path[i].x;
    p.pose.position.y = rrtPlanner._path[i].y;
    path.poses.push_back(p);

  }
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    std::cout << LOG <<"pub path" << std::endl;
    path_pub.publish(path);
    loop_rate.sleep();
  }
  return 0;
}
