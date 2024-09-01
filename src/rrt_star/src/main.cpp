#include "rrt_star.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_star_node");
  ros::NodeHandle n("");
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 1);
  ros::Publisher all_path_pub = n.advertise<nav_msgs::Path>("/allPath", 1);

  Node start, goal;
  start.position.x = -14.9;
  start.position.y = -14.9;
  goal.position.x = 14.9;
  goal.position.y = 14.9;
  RRTSTAR rrtstarPlanner(start, goal, 1.0);
  rrtstarPlanner.plan();
  rrtstarPlanner.broadcastPath();
  rrtstarPlanner.broadcastAllPath();
  nav_msgs::Path path, allPath;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  std::cout << LOG << "rrtstarPlanner._path.size() = " << rrtstarPlanner._path.size() << std::endl;
  std::cout << LOG << "rrtstarPlanner._allPath.size() = " << rrtstarPlanner._allPath.size() << std::endl;
  for(short i = 0; i < rrtstarPlanner._path.size(); i++)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = rrtstarPlanner._path[i].x;
    p.pose.position.y = rrtstarPlanner._path[i].y;
    path.poses.push_back(p);
  }
  allPath.header.frame_id = "map";
  allPath.header.stamp = ros::Time::now();
  for(short i = 0; i < rrtstarPlanner._allPath.size(); i++)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = rrtstarPlanner._allPath[i].x;
    p.pose.position.y = rrtstarPlanner._allPath[i].y;
    allPath.poses.push_back(p);
  }
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    std::cout << LOG <<"pub path" << std::endl;
    path_pub.publish(path);
    all_path_pub.publish(allPath);
    loop_rate.sleep();
  }
  return 0;
}
