#ifndef INFLATION_LAYER_H
#define INFLATION_LAYER_H

#include <vector>
#include <cmath>
#include "cost_values.h"
#include <map>
#include <cstring>
#include <iostream>

/**
 * @brief 膨胀像素，用于描述障碍物周围像素块的属性
 * @param idx 像素在costmap->data中的索引
 * @param x_cell 像素的x坐标（相对于costmap的width）
 * @param y_cell 像素的y坐标（相对于costmap的height）
 * @param x_obs_closest 距离该像素块最近的障碍物x坐标
 * @param y_obs_closest 距离该像素块最近的障碍物y坐标
 */
class CellData
{
public:
    CellData(int idx, int x_cell, int y_cell, int x_obs_closest, int y_obs_closest):
        _idx(idx),
        _x_cell(x_cell),
        _y_cell(y_cell),
        _x_obs_closest(x_obs_closest),
        _y_obs_closest(y_obs_closest)
    {}
    int _idx;
    int _x_cell;
    int _y_cell;
    int _x_obs_closest;
    int _y_obs_closest;
};

class InflationLayer
{
public:
    InflationLayer(int* costmap, double inflation_radius, int costmap_size_width, int costmap_size_height, double wight, double resolution);
    void OnInitialize();
    void OnUpdate();
    void computeCaches();
    void deleteCaches();
    int calCostBasedOnDistance(int cell_distance);

private:
    double _inflation_radius;              //膨胀半径，单位为米
    double _resolution;                    //分辨率
    int _costmap_size_width;            //costmap的宽度
    int _costmap_size_height;           //costmap的高度
    int _seen_size;                       //seen数组的大小
    bool *_seen;                           //是否已经探索过的栅格
    int * _costmap;                        //costmap指针
    // double _cell_inflation_radius;

    //缓存
    double _cached_cell_inflation_radius;        //膨胀半径缓存，单位为像素
    double **_cached_cell_inflation_cost;        //对应栅格的膨胀代价
    double **_cached_cell_inflation_distance;    //对应栅格的膨胀距离
    double _wight;                               //权重，用于计算超出膨胀半径的代价衰减程度，数值越大，衰减以越快，数值越小，衰减以越慢


    //遍历costmap，筛选inflation
    std::map<double, std::vector<CellData>> _inflation_cells; //key为膨胀半径，value为膨胀像素的属性
    double costLookup(int x_cell, int y_cell, int x_obs_closest, int y_obs_closest);
    double distanceLookup(int x_cell, int y_cell, int x_obs_closest, int y_obs_closest);
    void enqueue(int index, int x_cell, int y_cell, int x_obs_closest, int y_obs_closest);
};




#endif // INFLATION_LAYER_H