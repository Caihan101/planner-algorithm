#ifndef ASTAR_H
#define ASTAR_H
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <queue>
#include <unordered_map>
#include <ctime>

#define TEST_ printf("test\n");

using namespace std;

namespace cvtAstar{

//Node的类别
enum NodeType{
    free = 0,
    obstacle,
    inOpenList,
    inCloseList
};

//Node astar寻路中的离散量
struct Node{
    
    //储存位置信息
    pair<int, int> point;

    // F = G + H； G：从起始点到当前Node的路径cost， H：从当前Node到终点的cost花费，由欧式距离计算
    int F, G, H; 

    //父节点，用于路径反转
    Node* parent; 

    //构造函数
    Node(pair<int, int> _point = make_pair(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

//比较两个节点的h值，用于初始化优先级队列openlist，其中具有最小H的Node会被自动排列至队首
struct compare
{
    bool operator() (pair<int, pair<int, int>> a, pair<int, pair<int, int>> b) // Comparison function for priority queue
    {
        return a.first > b.first; // min h
    }
};


class Astar{

public:

    //初始化astar
    void InitAstar(const vector<pair<int, int>>& obsPoints, int height, int width);

    //astar寻路
    vector<pair<double, double>> pathPlanning(pair<int, int> startPoint, pair<int, int> targetPoint);

    void getCloseAndOpenSize();
    
private:

    void mapUpdate(const vector<pair<int, int>>& obsPoints);

    //寻路
    Node* FindPath();

    //找到终点时，反转路径
    void GetPath(Node* TailNode, vector<pair<double, double>>& path);

    //将点的坐标转为索引值
    inline int point2index(pair<int, int> point) {return point.first * width_ + point.second;}

    //将点的索引值转为坐标
    inline pair<int, int> index2point(int index) {return pair<int, int>(int(index / width_), index % width_);}

    void releaseMemory();
    

public:

    vector<pair<int, int>> updateObsPoints;

private:
    //与地图相关参数
    int height_;
    int width_;
    vector<vector<int>> map_;
    
    //起始点信息
    pair<int, int> startPoint_, targetPoint_;
    vector<vector<int>> neighbor_;

    //astar得到的路径
    vector<Node*> PathList;

    //openlist 储存待访问的点，具体作用可以查询astar算法
    priority_queue<pair<long, pair<int, int>>, vector<pair<int, pair<int, int>>>, compare> OpenList;

    //openlist的散列表，可以理解为一个哈希表，储存Node和其对应的index关系
    unordered_map<long, Node*> OpenDict;


};
}

#endif