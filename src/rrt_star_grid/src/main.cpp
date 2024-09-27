#include "rrt_star_grid.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "Bizier.h"

bool mapFlag = false;
RRTSTARGRID *rrtstargridPlanner;
nav_msgs::Path path, smoothPath;
ros::Publisher path_pub;
ros::Publisher pub_smooth_path;
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr msg);
void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr msg);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_star_grid_node");
  ros::NodeHandle n("");
  ros::Subscriber map_sub = n.subscribe("/map", 1, &mapCallback);
  ros::Subscriber sub_start = n.subscribe("/initialpose", 1, &startCallback);
  ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1, &goalCallback);
  path_pub = n.advertise<nav_msgs::Path>("/path", 1);
  pub_smooth_path = n.advertise<nav_msgs::Path>("/smooth_path", 1);

  double step, near_area_radius;


  n.param<double>("rrt_star_grid_node/step", step, 0.5);
  // n.param<double>("rrt_star_grid_node/step", step, 1.0);
  n.param<double>("rrt_star_grid_node/near_area_radius", near_area_radius, 1.5);
  // n.param<double>("rrt_star_grid_node/near_area_radius", near_area_radius, 3.0);

  rrtstargridPlanner = new RRTSTARGRID(n, step, near_area_radius);

  ros::Rate loop_rate(1);
  // Bizier bizier(10);
  while (ros::ok())
  {
    path_pub.publish(path);
    // nav_msgs::Path smooth_path;
    // smooth_path.header.frame_id = "map";
    // smooth_path.header.stamp = ros::Time::now();
    // bizier.solve(path, smooth_path);
    // pub_smooth_path.publish(smooth_path);
    // if (smooth_path.poses.size() != 0)
    // std::cout << LOG << "smooth_path.size() = " << smooth_path.poses.size() << std::endl;
    // std::cout << "pub path" << std::endl;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}


void plan()
{
  std::cout << "Start Planing" << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();                                      /// start time
  rrtstargridPlanner->plan();
  auto end_time = std::chrono::high_resolution_clock::now();                                        /// end time
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);     /// calculate duration
  std::cout << "Planing time = " << duration.count() << " ms" << std::endl;                         /// print duration
  rrtstargridPlanner->broadcastPath();

  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();

  path.poses.clear();
  for(long unsigned int i = 0; i < rrtstargridPlanner->_path.size(); i++)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = rrtstargridPlanner->_path[i].x;
    p.pose.position.y = rrtstargridPlanner->_path[i].y;
    path.poses.push_back(p);
  }
  Bizier bizier(10);
  nav_msgs::Path smooth_path;
  smooth_path.header.frame_id = "map";
  smooth_path.header.stamp = ros::Time::now();
  bizier.solve(path, smooth_path);
  pub_smooth_path.publish(smooth_path);
  if (smooth_path.poses.size() != 0)
  std::cout << "origin_path.size() = " << rrtstargridPlanner->_path.size() << std::endl;
  std::cout << "smooth_path.size() = " << smooth_path.poses.size() << std::endl;

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

  // std::cout << "msg->info.height = " << msg->info.height << std::endl;
  // std::cout << "msg->info.width = " << msg->info.width << std::endl;

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

  std::cout << "Finished Initializing Map"<< ", costmap width: " << msg->info.width << ", height: " << msg->info.height <<", size: " << msg->info.width*msg->info.height << std::endl;

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