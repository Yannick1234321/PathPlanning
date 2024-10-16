#include "inflation_layer.h"

InflationLayer::InflationLayer(int* costmap, double inflation_radius, int costmap_size_width, int costmap_size_height, double wight, double resolution):
    _costmap(costmap),
    _inflation_radius(inflation_radius),
    _costmap_size_width(costmap_size_width),
    _costmap_size_height(costmap_size_height),
    _wight(wight),
    _resolution(resolution),
    _cached_cell_inflation_cost(nullptr),
    _cached_cell_inflation_distance(nullptr),
    _seen(nullptr)
{
    _cached_cell_inflation_radius = 0;
    // _cached_cell_inflation_radius = inflation_radius * 1 / _resolution;
    std::cout << "_inflation_radius = " << _inflation_radius << std::endl;
    std::cout << "_costmap_size_width = " << _costmap_size_width << std::endl;
    std::cout << "_costmap_size_height = " << _costmap_size_height << std::endl;
    std::cout << "_wight = " << _wight << std::endl;
    std::cout << "_resolution = " << _resolution << std::endl;
    OnInitialize();
}



void InflationLayer::OnInitialize()
{
    // _resolution = costmap_->getResolution();                                //从costmap指针中获取costmap的分辨率
    // _cell_inflation_radius_ = ;     //从costmap指针中获取栅格的膨胀半径
    computeCaches();                                                       //计算当前膨胀半径下的距离-代价值（第一向限）

    if (_seen)                                                              //如果seen_指针不为空，则释放前一次分配的内存
        delete[] _seen;
        _seen = nullptr;
    _seen_size = int(_costmap_size_width * _costmap_size_height);
    _seen = new bool[_seen_size];                                          //这里只是创建对应数组位的空间，并没有分配内存
    // memset(_seen, 0, _seen_size * sizeof(bool));                         //这才是分配内存
}
void InflationLayer::deleteCaches()
{
    if (_cached_cell_inflation_cost != nullptr)
    {
        for (int i = 0; i < _cached_cell_inflation_radius + 2; i++)
        {
            if (_cached_cell_inflation_cost[i] != nullptr)
            {
                delete[] _cached_cell_inflation_cost[i];
                _cached_cell_inflation_cost = nullptr;
            }
        }
        delete[] _cached_cell_inflation_cost;
        _cached_cell_inflation_cost = nullptr;
    }
    if (_cached_cell_inflation_distance != nullptr)
    {
        for (int i = 0; i < _cached_cell_inflation_radius + 2; i++)
        {
            if (_cached_cell_inflation_distance[i] != nullptr)
            {
                delete[] _cached_cell_inflation_distance[i];
                _cached_cell_inflation_distance = nullptr;
            }
        }
        delete[] _cached_cell_inflation_distance;
        _cached_cell_inflation_distance = nullptr;
    }
}
void InflationLayer::computeCaches()
{
    if (_inflation_radius == 0) //如果膨胀半径为0，则直接返回，不需要膨胀
        return;
    if (_inflation_radius != _cached_cell_inflation_radius) //如果膨胀半径发生变化，则重新计算缓存
    {
        deleteCaches();

        _cached_cell_inflation_cost = new double*[int(_inflation_radius + 2)];
        _cached_cell_inflation_distance = new double*[int(_inflation_radius + 2)];
        //遍历二维数组的所有值
        for (int i = 0; i < int(_inflation_radius + 2); i++)       //遍历每行
        {
            _cached_cell_inflation_cost[i] = new double[int(_inflation_radius + 2)];
            _cached_cell_inflation_distance[i] = new double[int(_inflation_radius + 2)];
            for (int j = 0; j < int(_inflation_radius + 2); j++)   //遍历每列
            {
                _cached_cell_inflation_distance[i][j] = std::hypot(i,j); //计算距离
            }
        }
        _cached_cell_inflation_radius = _inflation_radius;         //缓存更新标志位
    }
    else
    {
        for (int i = 0; i < int(_inflation_radius + 2); i++)       //遍历每行
        {
            for (int j = 0; j < int(_inflation_radius + 2); j++)   //遍历每列
            {
                _cached_cell_inflation_cost[i][j] = calCostBasedOnDistance(_cached_cell_inflation_distance[i][j]);
            }
        }
    }
}

int InflationLayer::calCostBasedOnDistance(int cell_distance)
{
    double euclid_distance = cell_distance * _resolution;
    if (euclid_distance == 0)                                       //obstacle
    {
        return LETHAL_OBSTACLE;
    }
    else if (euclid_distance <= _inflation_radius)                  //inflated obstacle
    {
        return INSCRIBED_INFLATED_OBSTACLE;
    }
    else if (euclid_distance > _inflation_radius)                   //buffer zone
    {
        return int(std::exp(-1 * _wight * (euclid_distance - _inflation_radius)));
    }
}

double InflationLayer::costLookup(int x_cell, int y_cell, int x_obs_closest, int y_obs_closest)
{
    int dx = std::abs(x_cell - x_obs_closest);
    int dy = std::abs(y_cell - y_obs_closest);
    return _cached_cell_inflation_cost[dx][dy];
}

double InflationLayer::distanceLookup(int x_cell, int y_cell, int x_obs_closest, int y_obs_closest)
{
    int dx = std::abs(x_cell - x_obs_closest);
    int dy = std::abs(y_cell - y_obs_closest);
    return _cached_cell_inflation_distance[dx][dy];
}

void InflationLayer::enqueue(int index, int x_cell, int y_cell, int x_obs_closest, int y_obs_closest)
{
    if (_seen[index] == false)
    {
        double distance = distanceLookup(x_cell, y_cell, x_obs_closest, y_obs_closest);
        if (distance > _inflation_radius)
        {
            return;
        }
        _inflation_cells[distance].push_back(CellData(index, x_cell, y_cell, x_obs_closest, y_obs_closest));
    }
}

void InflationLayer::OnUpdate()
{
    if (_inflation_radius == 0) //如果膨胀半径为0，则直接返回，不需要膨胀
        return;

    if (_seen == nullptr)
    {
        _seen = new bool[int(_costmap_size_width * _costmap_size_height)];
    }
    else if (_seen_size != int(_costmap_size_width * _costmap_size_height))
    {
        delete[] _seen;
        _seen = new bool[int(_costmap_size_width * _costmap_size_height)];
        _seen_size = int(_costmap_size_width * _costmap_size_height);
    }
    std::memset(_seen, false, _seen_size * sizeof(bool));

    //开始检测costmap地图
    _inflation_cells.clear();
    std::vector<CellData> &obs_cell = _inflation_cells[0.0];        //初始化一个空的障碍物列表（引用），对这个变量修改，直接影响到_inflation_cells[0.0]中对应的值变量
    //将所有的障碍物像素点存入_inflation_cells[0.0]中
    for (int i = 0; i < _costmap_size_width; i++)
    {
        for (int j = 0; j < _costmap_size_height; j++)
        {
            int idx = j * _costmap_size_width + i;
            if (_costmap[idx] == 100)
            {
                obs_cell.push_back(CellData(idx, i, j, i,j));
            }
        }
    }
    std::map<double, std::vector<CellData>>::iterator bin;   //迭代器，从_infation_cells中取出元素
    for (bin = _inflation_cells.begin(); bin != _inflation_cells.end(); bin++)
    {
        for (int i = 0; i < bin->second.size(); i++)            //遍历每一个cell，初始一轮是遍历所有的障碍物点本点
        {
            CellData &cell = bin->second[i];                      //取出当前cell，第一轮是障碍物

            int index = cell._idx;                               //当前cell的索引
            if (_seen[index] == true)
            {
                continue;
            }

            _seen[index] = true;                                 //标记当前cell为已访问

            int x_cell, y_cell;                                  //当前cell的坐标
            int x_obs_closest, y_obs_closest;                     //当前cell与最近障碍物的坐标
            x_cell = cell._x_cell;
            y_cell = cell._y_cell;
            x_obs_closest = cell._x_obs_closest;
            y_obs_closest = cell._y_obs_closest;

                                                                    //计算当前cell的代价值
            double cost = costLookup(x_cell, y_cell, x_obs_closest, y_obs_closest);
            double old_cost = _costmap[index];                     //取出costmap中当前cell的代价值

            if (old_cost == NO_INFORMATION)                       //如果costmap中当前cell的代价值是NO_INFORMATION（即障碍物本体）
            {
                // _costmap[index] = cost;                            //则将当前cell的代价值更新到costmap中
                _costmap[index] = 100;                            //则将当前cell的代价值更新到costmap中
            }
            else
            {
                _costmap[index] = std::max(99.0, old_cost);        //否则，取代价值大的那个代替原代价值
            }
            // attempt to put the neighbors of the current cell onto the inflation list
            //检查当前单元格的邻居（上、下、左、右）是否是障碍物

            if (x_cell > 0)
                enqueue(index - 1, x_cell - 1, y_cell, x_obs_closest, y_obs_closest);                           //左
            if (y_cell > 0)
                enqueue(index - _costmap_size_width, x_cell, y_cell - 1, x_obs_closest, y_obs_closest);         //下
            if (x_cell < _costmap_size_width - 1)
                enqueue(index + 1, x_cell + 1, y_cell, x_obs_closest, y_obs_closest);                           //右
            if (y_cell < _costmap_size_height - 1)
                enqueue(index + _costmap_size_width, x_cell, y_cell + 1, x_obs_closest, y_obs_closest);         //上
        }
    }




}
