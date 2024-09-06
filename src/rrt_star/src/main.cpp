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

  double start_x, start_y, goal_x, goal_y, step;
  double size_x_min, size_x_max, size_y_min, size_y_max;
  double near_area_radius;

  n.param<double>("rrt_star_node/start_x", start_x, 0.0);
  n.param<double>("rrt_star_node/start_y", start_y, 0.0);
  n.param<double>("rrt_star_node/goal_x", goal_x, 15.0);
  n.param<double>("rrt_star_node/goal_y", goal_y, 15.0);
  n.param<double>("rrt_star_node/step", step, 2.0);
  n.param<double>("rrt_star_node/size_x_min", size_x_min, -16.0);
  n.param<double>("rrt_star_node/size_x_max", size_x_max, 16.0);
  n.param<double>("rrt_star_node/size_y_min", size_y_min, -16.0);
  n.param<double>("rrt_star_node/size_y_max", size_y_max, 16.0);
  n.param<double>("rrt_star_node/near_area_radius", near_area_radius, 3.0);

  std::cout << LOG << "start_x = " << start_x << std::endl;
  Node start, goal;
  start.position.x = start_x;
  start.position.y = start_y;
  goal.position.x = goal_x;
  goal.position.y = goal_y;
  RRTSTAR rrtstarPlanner(start, goal, step, size_x_min, size_x_max, size_y_min, size_y_max, near_area_radius);
  rrtstarPlanner.plan();
  rrtstarPlanner.broadcastPath();
  rrtstarPlanner.broadcastAllPath();
  nav_msgs::Path path, allPath;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  std::cout << LOG << "rrtstarPlanner._path.size() = " << rrtstarPlanner._path.size() << std::endl;
  std::cout << LOG << "rrtstarPlanner._allPath.size() = " << rrtstarPlanner._allPath.size() << std::endl;
  for(long unsigned int i = 0; i < rrtstarPlanner._path.size(); i++)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = rrtstarPlanner._path[i].x;
    p.pose.position.y = rrtstarPlanner._path[i].y;
    path.poses.push_back(p);
  }
  allPath.header.frame_id = "map";
  allPath.header.stamp = ros::Time::now();
  for(long unsigned int i = 0; i < rrtstarPlanner._allPath.size(); i++)
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
