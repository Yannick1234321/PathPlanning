#ifndef CROSSLINE_H
#define CROSSLINE_H

#include <iostream>
using namespace std;


//定义Point2f结构体
struct Point2f
{
	float x;
	float y;
};

// 定义直线参数结构体
struct LinePara
{
	float k;
	float b;

};

// 获取直线参数
void getLinePara(float& x1, float& y1, float& x2, float& y2, LinePara & LP)
{
	double m = 0;

	// 计算分子
	m = x2 - x1;

	if (0 == m)
	{
		LP.k = 10000.0;
		LP.b = y1 - LP.k * x1;
	}
	else
	{
		LP.k = (y2 - y1) / (x2 - x1);
		LP.b = y1 - LP.k * x1;
	}


}

// 获取交点
bool getCross(float & x1, float &y1, float & x2, float & y2, float & x3, float &y3, float & x4, float & y4,  Point2f & pt){

	LinePara para1, para2;
	getLinePara(x1, y1, x2, y2, para1);
	getLinePara(x3, y3, x4, y4, para2);

	// 判断是否平行
	if (abs(para1.k - para2.k) > 0.5)
	{
		pt.x = (para2.b - para1.b) / (para1.k - para2.k);
		pt.y = para1.k * pt.x + para1.b;

		return true;

	}
	else
	{
		return false;
	}

}

// 测试用例
// void main()
// {
// 	Point2f pt1, pt2, pt3, pt4, pt;
// 	pt1.x = 1.0;
// 	pt1.y = 1.0;

// 	pt2.x = 2.0;
// 	pt2.y = 2.0;

// 	pt3.x = 0.0;
// 	pt3.y = 2.0;

// 	pt4.x = 2.0;
// 	pt4.y = 0.0;

// 	getCross(pt1.x, pt1.y, pt2.x, pt2.y, pt3.x, pt3.y, pt4.x, pt4.y, pt);

// 	cout << pt.x << " , " << pt.y << endl;
// }

#endif // CROSSLINE_H