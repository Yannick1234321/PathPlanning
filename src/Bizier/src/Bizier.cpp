#include "Bizier.h"


bool Bizier::solve(const nav_msgs::Path& path, nav_msgs::Path& result)
{
  result.poses.clear();
  if(path.poses.size() < 6)
  {
    std::cout << "Path has less than 2 poses" << std::endl;
    return false;
  }

  //例如传进来24个点，则初始化_six_point为前6个点，后面的点以4个点为一组，存放在_vector_four_point中
  //共有1+18/4 = 1+4 = 5组路径，包括1组前6个点的路径，4组后面的路径
  _six_point.clear();
  _four_point.clear();
  _vector_four_point.clear();
  for (int i = 0; i < path.poses.size(); i++)
  {
    if (i < 6)
    {
      _six_point.push_back(path.poses[i]);
    }
    else
    {
      int idx4 = ((i - 6) % 4);
      _four_point.push_back(path.poses[i]);
      if (idx4 == 3)
      {
        _vector_four_point.push_back(_four_point);
        _four_point.clear();
      }
    }
  }
  //生成路径
  //前六个点的路径
  std::vector<geometry_msgs::PoseStamped> path_temp = _generate_path(_six_point);
  _merge_path(path_temp, result);
  //后面的路径
  for (int i = 0; i < _vector_four_point.size(); i++)
  {
    geometry_msgs::PoseStamped P4, P5, Q0, Q1, Q2, Q3, Q4, Q5;
    P4 = _six_point[4];
    P5 = _six_point[5];
    Q0 = P5;
    Q2 = _vector_four_point[i][0];
    Q3 = _vector_four_point[i][1];
    Q4 = _vector_four_point[i][2];
    Q5 = _vector_four_point[i][3];
    //计算Q1
    Point2f pt1, pt2, pt3, pt4, pt;
    pt1.x = P4.pose.position.x;
    pt1.y = P4.pose.position.y;
    pt2.x = P5.pose.position.x;
    pt2.y = P5.pose.position.y;
    pt3.x = Q2.pose.position.x;
    pt3.y = Q2.pose.position.y;
    pt4.x = Q3.pose.position.x;
    pt4.y = Q3.pose.position.y;
    // getCross(pt1.x, pt1.y, pt2.x, pt2.y, pt3.x, pt3.y, pt4.x, pt4.y, pt);
    AuxiliaryPoint(pt1, pt2, pt3, pt4, pt);
    Q1.pose.position.x = pt.x;
    Q1.pose.position.y = pt.y;
    //生成路径
    _six_point.clear();
    _six_point.push_back(Q0);
    _six_point.push_back(Q1);
    _six_point.push_back(Q2);
    _six_point.push_back(Q3);
    _six_point.push_back(Q4);
    _six_point.push_back(Q5);
    std::vector<geometry_msgs::PoseStamped> path_temp_back = _generate_path(_six_point);
    _merge_path(path_temp_back, result);
  }
  return true;

}

geometry_msgs::PoseStamped Bizier::_get_Q1(geometry_msgs::Point p1_line_1, geometry_msgs::Point p2_line_1,
                                          geometry_msgs::Point p1_line_2, geometry_msgs::Point p2_line_2)
{

  	Point2f pt1, pt2, pt3, pt4, pt;
  	pt1.x = p1_line_1.x;
  	pt1.y = p1_line_1.y;

  	pt2.x = p2_line_1.x;
  	pt2.y = p2_line_1.y;

  	pt3.x = p1_line_2.x;
  	pt3.y = p1_line_2.y;

    pt4.x = p2_line_2.x;
  	pt4.y = p2_line_2.y;

    getCross(pt1.x, pt1.y, pt2.x, pt2.y, pt3.x, pt3.y, pt4.x, pt4.y, pt);
    geometry_msgs::PoseStamped q1;
    q1.pose.position.x = pt.x;
    q1.pose.position.y = pt.y;
    return q1;
}

std::vector<geometry_msgs::PoseStamped> Bizier::_get_control_points_discrete( std::vector<geometry_msgs::PoseStamped> Q0_discrete_path, std::vector<geometry_msgs::PoseStamped> Q1_discrete_path)
{
  std::vector<geometry_msgs::PoseStamped> control_points;
  for (double t = 0; t < 1; t += _dt)
  {
    int idx = t / _dt;
    geometry_msgs::PoseStamped control_point;
    control_point.pose.position.x = (1-t)*Q0_discrete_path[idx].pose.position.x + t*Q1_discrete_path[idx].pose.position.x;
    control_point.pose.position.y = (1-t)*Q0_discrete_path[idx].pose.position.y + t*Q1_discrete_path[idx].pose.position.y;
    control_points.push_back(control_point);
  }
  // for (double idx = 0; idx < _num_of_points; idx++)
  // {
  //   geometry_msgs::PoseStamped control_point;
  //   control_point.pose.position.x = Q0_discrete_path[idx].pose.position.x + Q1_discrete_path[_num_of_points-1-idx].pose.position.x;
  //   control_point.pose.position.y = Q0_discrete_path[idx].pose.position.y + Q1_discrete_path[_num_of_points-1-idx].pose.position.y;
  //   control_points.push_back(control_point);
  // }
  return control_points;
}

std::vector<geometry_msgs::PoseStamped> Bizier::_generate_path(std::vector<geometry_msgs::PoseStamped> six_point)
{
  double t = 0;
  geometry_msgs::PoseStamped p0, p1, p2, p3, p4, p5;
  p0 = six_point[0];
  p1 = six_point[1];
  p2 = six_point[2];
  p3 = six_point[3];
  p4 = six_point[4];
  p5 = six_point[5];
  std::vector<geometry_msgs::PoseStamped> Q0_Q1_Q2_Q3_Q4_Q5_discrete_path;
  for (double t = 0; t < 1; t += _dt)
  {
    //一阶贝塞尔曲线
    geometry_msgs::PoseStamped p01, p12, p23, p34, p45;
    p01.pose.position.x = (1-t)*p0.pose.position.x + t*p1.pose.position.x;
    p01.pose.position.y = (1-t)*p0.pose.position.y + t*p1.pose.position.y;
    p12.pose.position.x = (1-t)*p1.pose.position.x + t*p2.pose.position.x;
    p12.pose.position.y = (1-t)*p1.pose.position.y + t*p2.pose.position.y;
    p23.pose.position.x = (1-t)*p2.pose.position.x + t*p3.pose.position.x;
    p23.pose.position.y = (1-t)*p2.pose.position.y + t*p3.pose.position.y;
    p34.pose.position.x = (1-t)*p3.pose.position.x + t*p4.pose.position.x;
    p34.pose.position.y = (1-t)*p3.pose.position.y + t*p4.pose.position.y;
    p45.pose.position.x = (1-t)*p4.pose.position.x + t*p5.pose.position.x;
    p45.pose.position.y = (1-t)*p4.pose.position.y + t*p5.pose.position.y;
    //二阶贝塞尔曲线
    geometry_msgs::PoseStamped p012, p123, p234, p345;
    p012.pose.position.x = (1-t)*p01.pose.position.x + t*p12.pose.position.x;
    p012.pose.position.y = (1-t)*p01.pose.position.y + t*p12.pose.position.y;
    p123.pose.position.x = (1-t)*p12.pose.position.x + t*p23.pose.position.x;
    p123.pose.position.y = (1-t)*p12.pose.position.y + t*p23.pose.position.y;
    p234.pose.position.x = (1-t)*p23.pose.position.x + t*p34.pose.position.x;
    p234.pose.position.y = (1-t)*p23.pose.position.y + t*p34.pose.position.y;
    p345.pose.position.x = (1-t)*p34.pose.position.x + t*p45.pose.position.x;
    p345.pose.position.y = (1-t)*p34.pose.position.y + t*p45.pose.position.y;
    //三阶贝塞尔曲线
    geometry_msgs::PoseStamped p0123, p1234, p2345;
    p0123.pose.position.x = (1-t)*p012.pose.position.x + t*p123.pose.position.x;
    p0123.pose.position.y = (1-t)*p012.pose.position.y + t*p123.pose.position.y;
    p1234.pose.position.x = (1-t)*p123.pose.position.x + t*p234.pose.position.x;
    p1234.pose.position.y = (1-t)*p123.pose.position.y + t*p234.pose.position.y;
    p2345.pose.position.x = (1-t)*p234.pose.position.x + t*p345.pose.position.x;
    p2345.pose.position.y = (1-t)*p234.pose.position.y + t*p345.pose.position.y;
    //四阶贝塞尔曲线
    geometry_msgs::PoseStamped p01234, p12345;
    p01234.pose.position.x = (1-t)*p0123.pose.position.x + t*p1234.pose.position.x;
    p01234.pose.position.y = (1-t)*p0123.pose.position.y + t*p1234.pose.position.y;
    p12345.pose.position.x = (1-t)*p1234.pose.position.x + t*p2345.pose.position.x;
    p12345.pose.position.y = (1-t)*p1234.pose.position.y + t*p2345.pose.position.y;
    //五阶贝塞尔曲线
    geometry_msgs::PoseStamped p012345;
    p012345.pose.position.x = (1-t)*p01234.pose.position.x + t*p12345.pose.position.x;
    p012345.pose.position.y = (1-t)*p01234.pose.position.y + t*p12345.pose.position.y;
    //返回路径
    Q0_Q1_Q2_Q3_Q4_Q5_discrete_path.push_back(p012345);
  }
  return Q0_Q1_Q2_Q3_Q4_Q5_discrete_path;





  // //一阶贝塞尔曲线
  // std::vector<geometry_msgs::PoseStamped> Q0_Q1_discrete_path, Q1_Q2_discrete_path, Q2_Q3_discrete_path, Q3_Q4_discrete_path, Q4_Q5_discrete_path;
  // for (double t = 0; t < 1; t += _dt)
  // {
  //   geometry_msgs::PoseStamped q0_q1_point;
  //   geometry_msgs::PoseStamped q1_q2_point;
  //   geometry_msgs::PoseStamped q2_q3_point;
  //   geometry_msgs::PoseStamped q3_q4_point;
  //   geometry_msgs::PoseStamped q4_q5_point;
  //   q0_q1_point.pose.position.x = (1-t)*p0.pose.position.x + t*p1.pose.position.x;
  //   q0_q1_point.pose.position.y = (1-t)*p0.pose.position.y + t*p1.pose.position.y;
  //   q1_q2_point.pose.position.x = (1-t)*p1.pose.position.x + t*p2.pose.position.x;
  //   q1_q2_point.pose.position.y = (1-t)*p1.pose.position.y + t*p2.pose.position.y;
  //   q2_q3_point.pose.position.x = (1-t)*p2.pose.position.x + t*p3.pose.position.x;
  //   q2_q3_point.pose.position.y = (1-t)*p2.pose.position.y + t*p3.pose.position.y;
  //   q3_q4_point.pose.position.x = (1-t)*p3.pose.position.x + t*p4.pose.position.x;
  //   q3_q4_point.pose.position.y = (1-t)*p3.pose.position.y + t*p4.pose.position.y;
  //   q4_q5_point.pose.position.x = (1-t)*p4.pose.position.x + t*p5.pose.position.x;
  //   q4_q5_point.pose.position.y = (1-t)*p4.pose.position.y + t*p5.pose.position.y;
  //   Q0_Q1_discrete_path.push_back(q0_q1_point);
  //   Q1_Q2_discrete_path.push_back(q1_q2_point);
  //   Q2_Q3_discrete_path.push_back(q2_q3_point);
  //   Q3_Q4_discrete_path.push_back(q3_q4_point);
  //   Q4_Q5_discrete_path.push_back(q4_q5_point);
  // }
  // //二阶贝塞尔曲线
  // std::vector<geometry_msgs::PoseStamped> Q0_Q1_Q2_discrete_path, Q1_Q2_Q3_discrete_path, Q2_Q3_Q4_discrete_path, Q3_Q4_Q5_discrete_path;
  // Q0_Q1_Q2_discrete_path = _get_control_points_discrete(Q0_Q1_discrete_path, Q1_Q2_discrete_path);
  // Q1_Q2_Q3_discrete_path = _get_control_points_discrete(Q1_Q2_discrete_path, Q2_Q3_discrete_path);
  // Q2_Q3_Q4_discrete_path = _get_control_points_discrete(Q2_Q3_discrete_path, Q3_Q4_discrete_path);
  // Q3_Q4_Q5_discrete_path = _get_control_points_discrete(Q3_Q4_discrete_path, Q4_Q5_discrete_path);
  // //三阶贝塞尔曲线
  // std::vector<geometry_msgs::PoseStamped> Q0_Q1_Q2_Q3_discrete_path, Q1_Q2_Q3_Q4_discrete_path, Q2_Q3_Q4_Q5_discrete_path;
  // Q0_Q1_Q2_Q3_discrete_path = _get_control_points_discrete(Q0_Q1_Q2_discrete_path, Q1_Q2_Q3_discrete_path);
  // Q1_Q2_Q3_Q4_discrete_path = _get_control_points_discrete(Q1_Q2_Q3_discrete_path, Q2_Q3_Q4_discrete_path);
  // Q2_Q3_Q4_Q5_discrete_path = _get_control_points_discrete(Q2_Q3_Q4_discrete_path, Q3_Q4_Q5_discrete_path);
  // //四阶贝塞尔曲线
  // std::vector<geometry_msgs::PoseStamped> Q0_Q1_Q2_Q3_Q4_discrete_path, Q1_Q2_Q3_Q4_Q5_discrete_path;
  // Q0_Q1_Q2_Q3_Q4_discrete_path = _get_control_points_discrete(Q0_Q1_Q2_Q3_discrete_path, Q1_Q2_Q3_Q4_discrete_path);
  // Q1_Q2_Q3_Q4_Q5_discrete_path = _get_control_points_discrete(Q1_Q2_Q3_Q4_discrete_path, Q2_Q3_Q4_Q5_discrete_path);
  // //五阶贝塞尔曲线
  // std::vector<geometry_msgs::PoseStamped> Q0_Q1_Q2_Q3_Q4_Q5_discrete_path;
  // Q0_Q1_Q2_Q3_Q4_Q5_discrete_path = _get_control_points_discrete(Q0_Q1_Q2_Q3_Q4_discrete_path, Q1_Q2_Q3_Q4_Q5_discrete_path);

  //返回路径
  return Q0_Q1_Q2_Q3_Q4_Q5_discrete_path;
}

nav_msgs::Path Bizier::_merge_path(std::vector<geometry_msgs::PoseStamped> discrete_path, nav_msgs::Path& path)
{
  for (int i = 0; i < discrete_path.size(); i++)
  {
    path.poses.push_back(discrete_path[i]);
  }
  return path;
}

void Bizier::AuxiliaryPoint(Point2f &p4, Point2f &p5, Point2f &q2, Point2f &q3, Point2f &q1)
{
// 1）Q1要与P4-P5共线
// 2）Q1不能在P4-P5中间 ---> Q1 - P4 ≥ P5 - P4 ≥ Q1 - Q0
// 3）Q1与Q0的距离比Q2与Q1小 ---> || Q1 - Q0 || ≤ || Q2 - Q1 ||
// 4）Q1与Q0的距离小于或等于P5与P4的距离 ---> Q1 - Q0 = n * (P5 - P4), n∈(0,1]
// 5）P5与Q0重合 ---> P5 = Q0
  // float dx1 = std::abs(p5.x - p4.x);
	// float dy1 = std::abs(p5.y - p4.y);
  // float dis1 = std::hypot(dx1, dy1);
	// float dx2 = std::abs(q2.x - q3.x);
  // float dy2 = std::abs(q2.y - q3.y);
  // float dis2 = std::hypot(dx2, dy2);
  q1.x = p5.x + (p5.x - p4.x)/2.5;
  q1.y = p5.y + (p5.y - p4.y)/2.5;

}

