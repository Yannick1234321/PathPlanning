#include "rrt_star_grid.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

bool mapFlag = false;
RRTSTARGRID *rrtstargridPlanner;
nav_msgs::Path path, allPath;
ros::Publisher path_pub;
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr msg);
void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr msg);

// ros::init(argc, argv, "rrt_star_grid_node");
// ros::NodeHandle* n;
// ros::NodeHandle n("");


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_star_grid_node");
  ros::NodeHandle n("");
  ros::Subscriber map_sub = n.subscribe("/map", 1, &mapCallback);
  ros::Subscriber sub_start = n.subscribe("/initialpose", 1, &startCallback);
  ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1, &goalCallback);
  path_pub = n.advertise<nav_msgs::Path>("/path", 1);
  ros::Publisher all_path_pub = n.advertise<nav_msgs::Path>("/allPath", 1);

  double start_x, start_y, goal_x, goal_y, step;
  double size_x_min, size_x_max, size_y_min, size_y_max;
  double near_area_radius;


  n.param<double>("rrt_star_grid_node/start_x", start_x, -5.5);
  n.param<double>("rrt_star_grid_node/start_y", start_y, 0.0);
  n.param<double>("rrt_star_grid_node/goal_x", goal_x, 21.80);
  n.param<double>("rrt_star_grid_node/goal_y", goal_y, 1.5);
  // n.param<double>("rrt_star_grid_node/step", step, 0.5);
  n.param<double>("rrt_star_grid_node/step", step, 1.0);
  n.param<double>("rrt_star_grid_node/size_x_min", size_x_min, -30.0);
  n.param<double>("rrt_star_grid_node/size_x_max", size_x_max, 30.0);
  n.param<double>("rrt_star_grid_node/size_y_min", size_y_min, -30.0);
  n.param<double>("rrt_star_grid_node/size_y_max", size_y_max, 30.0);
  // n.param<double>("rrt_star_grid_node/near_area_radius", near_area_radius, 1.5);
  n.param<double>("rrt_star_grid_node/near_area_radius", near_area_radius, 3.0);

  rrtstargridPlanner = new RRTSTARGRID(n, step, near_area_radius);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    path_pub.publish(path);
    // all_path_pub.publish(allPath);
    std::cout << "pub path" << std::endl;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}


void plan()
{
  std::cout << "Start Planing" << std::endl;
  std::time_t start_t, stop_t;
  start_t = std::time(NULL);
  rrtstargridPlanner->plan();
  stop_t = std::time(NULL);
  std::cout << LOG << "cost time = " << (stop_t - start_t) << std::endl;
  rrtstargridPlanner->broadcastPath();
  // rrtstargridPlanner->broadcastAllPath();

  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();

  std::cout << LOG << "rrtstargridPlanner->_path.size() = " << rrtstargridPlanner->_path.size() << std::endl;
  // std::cout << LOG << "rrtstargridPlanner->_allPath.size() = " << rrtstargridPlanner->_allPath.size() << std::endl;
  path.poses.clear();
  for(long unsigned int i = 0; i < rrtstargridPlanner->_path.size(); i++)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = rrtstargridPlanner->_path[i].x;
    p.pose.position.y = rrtstargridPlanner->_path[i].y;
    path.poses.push_back(p);
  }
  // allPath.header.frame_id = "map";
  // allPath.header.stamp = ros::Time::now();
  // for(long unsigned int i = 0; i < rrtstargridPlanner->_allPath.size(); i++)
  // {
  //   geometry_msgs::PoseStamped p;
  //   p.header.frame_id = "map";
  //   p.pose.position.x = rrtstargridPlanner->_allPath[i].x;
  //   p.pose.position.y = rrtstargridPlanner->_allPath[i].y;
  //   allPath.poses.push_back(p);
  // }

}
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr msg)
{
  rrtstargridPlanner->mapInit(msg->info.resolution,
                              msg->info.origin.position.x,
                              msg->info.origin.position.y,
                              msg->info.origin.position.x,
                              msg->info.origin.position.x+(msg->info.width*msg->info.resolution),
                              msg->info.origin.position.y,
                              msg->info.origin.position.y+(msg->info.height*msg->info.resolution),
                              msg->info.width,
                              msg->info.height);

  std::cout << "msg->info.height = " << msg->info.height << std::endl;
  std::cout << "msg->info.width = " << msg->info.width << std::endl;

  for (unsigned int height = 0; height < msg->info.height; height++)
  {
    for (unsigned int width = 0; width < msg->info.width; width++)
    {
      int i = msg->data[height * msg->info.width + width];
      if (i != -1)
      {
        rrtstargridPlanner->setObstacle(width, height, i);
      }
    }
  }

  std::cout << LOG << "Finished inputting Map" << std::endl;

  rrtstargridPlanner->setMapFlag();
}

void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
  Node start;
  start.position.x = msg->pose.pose.position.x;
  start.position.y = msg->pose.pose.position.y;
  rrtstargridPlanner->setStartPoint(start);
  // if (rrtstargridPlanner->isInitFinished() == true)
  // {
  //   plan();
  //   rrtstargridPlanner->resetInitStartGoal();
  // }

}
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
  Node goal;
  goal.position.x = msg->pose.position.x;
  goal.position.y = msg->pose.position.y;
  rrtstargridPlanner->setGoal(goal);
  if (rrtstargridPlanner->isInitFinished() == true)
  {
    plan();
    path_pub.publish(path);
    rrtstargridPlanner->resetInitStartGoal();
  }

}


// int main(int argc, char **argv)
// {
//   // ros::init(argc, argv, "rrt_star_grid_node");
//   // ros::NodeHandle n("");
//   ros::Subscriber map_sub = n.subscribe("/map", 1, &mapCallback);
//   ros::Subscriber sub_start = n.subscribe("/initialpose", 1, &startCallback);
//   ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1, &goalCallback);
//   path_pub = n.advertise<nav_msgs::Path>("/path", 1);
//   ros::Publisher all_path_pub = n.advertise<nav_msgs::Path>("/allPath", 1);

//   double start_x, start_y, goal_x, goal_y, step;
//   double size_x_min, size_x_max, size_y_min, size_y_max;
//   double near_area_radius;


//   n.param<double>("rrt_star_grid_node/start_x", start_x, -5.5);
//   n.param<double>("rrt_star_grid_node/start_y", start_y, 0.0);
//   n.param<double>("rrt_star_grid_node/goal_x", goal_x, 21.80);
//   n.param<double>("rrt_star_grid_node/goal_y", goal_y, 1.5);
//   n.param<double>("rrt_star_grid_node/step", step, 0.1);
//   n.param<double>("rrt_star_grid_node/size_x_min", size_x_min, -30.0);
//   n.param<double>("rrt_star_grid_node/size_x_max", size_x_max, 30.0);
//   n.param<double>("rrt_star_grid_node/size_y_min", size_y_min, -30.0);
//   n.param<double>("rrt_star_grid_node/size_y_max", size_y_max, 30.0);
//   n.param<double>("rrt_star_grid_node/near_area_radius", near_area_radius, 3.0);

//   rrtstargridPlanner = new RRTSTARGRID(step, near_area_radius);

//   ros::Rate loop_rate(1);
//   while (ros::ok())
//   {
//     path_pub.publish(path);
//     // all_path_pub.publish(allPath);
//     std::cout << "pub path" << std::endl;
//     loop_rate.sleep();
//     ros::spinOnce();
//   }
//   return 0;
// }
