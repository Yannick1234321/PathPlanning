#ifndef BIZIER_H
#define BIZIER_H

#include <vector>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "crossLine.h"


class Bizier
{
public:
  Bizier(int num_of_points = 20) : _num_of_points(num_of_points) {_dt = 1.0 / (num_of_points);}
  ~Bizier(){};

  bool solve(const nav_msgs::Path& path, nav_msgs::Path& result);

private:
  // 单位t内生成的点数
  int _num_of_points;
  double _dt; //单位t
  double _t0; //起始时间
  double _tf; //终止时间

  //输入路径，变量个数小于等于6
  std::vector<geometry_msgs::PoseStamped> _six_point;
  //初初始化外，更新路径的每组点
  std::vector<std::vector<geometry_msgs::PoseStamped>> _vector_four_point;
  std::vector<geometry_msgs::PoseStamped> _four_point;
  //取两条直线的交点  作为Q1点
  geometry_msgs::PoseStamped _get_Q1(geometry_msgs::Point p1_line_1, geometry_msgs::Point p2_line_1,
                                     geometry_msgs::Point p1_line_2, geometry_msgs::Point p2_line_2);
  //Q0点选择第一个点或_six_point变量的最后一个点
  geometry_msgs::PoseStamped _get_Q0(geometry_msgs::Point p1_line_1, geometry_msgs::Point p2_line_1,
                               geometry_msgs::Point p1_line_2, geometry_msgs::Point p2_line_2);
  //生成一段6个点的平滑曲线，输入_six_point变量
  std::vector<geometry_msgs::PoseStamped> _generate_path(std::vector<geometry_msgs::PoseStamped> six_point);

  //计算贝塞尔曲线的控制点，输入Q0，Q1，输出控制点
  std::vector<geometry_msgs::PoseStamped> _get_control_points_discrete( std::vector<geometry_msgs::PoseStamped> Q0_discrete_path, std::vector<geometry_msgs::PoseStamped> Q1_discrete_path);
  //离散化曲线，输入平滑曲线，输出离散化后的曲线
  std::vector<geometry_msgs::PoseStamped> _discretize_path();

  //合并路径，输入平滑曲线，输出合并后的路径
  nav_msgs::Path _merge_path(std::vector<geometry_msgs::PoseStamped> discrete_path, nav_msgs::Path& path);

  //计算两端路径间的辅助点
  void AuxiliaryPoint(Point2f &p4, Point2f &p5, Point2f &q2, Point2f &q3, Point2f &q1);

};





#endif // BIZIER_H