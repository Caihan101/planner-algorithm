#pragma once

#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <random>
#include <cfloat>
#include <eigen3/Eigen/Eigen>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace cvtRRTStar {

enum NodeType{
    free = 0,
    obstacle
};

template <typename T1, typename T2>
inline float euclideanDistance(pair<T1, T1> &p, pair<T2, T2> &q)
{
    pair<double, double> v = make_pair(p.first - q.first, p.second - q.second);
    return sqrt(powf(v.first, 2) + powf(v.second, 2));
}


struct Node {
    
    Node *parent = nullptr;
    pair<double, double> position = {0, 0};
    float cost{0.0};

};


class RRTStar {
public:
    RRTStar();

    void initialize(const vector<pair<int, int>>& obsPoints, int height, int width,
                    pair<double, double> startPoint, pair<double, double> targetPoint);

    vector<pair<double, double>> pathPlanning();

    int node_size;
    int iter_count;

private:

    void mapUpdate(const vector<pair<int, int>>& obsPoints);

    inline Node* getRandomNode();

    inline Node* nearest(pair<double, double> &point);

    inline pair<double, double> newConfig(Node *q, Node *qNearest);

    inline void add(Node *qNearest, Node *qNew);

    inline bool reached();

    inline void deleteNodes(Node *root);

    template <typename T>
    inline bool isPointInMap(T x, T y);

    inline bool isThereObstacleBetween(pair<double, double> &p, pair<double, double> &q);

    template <typename T>
    inline bool isPointInObstacle(pair<T, T> &p);

    std::pair<double, double> start_point_;
    std::pair<double, double> goal_point_;

    
    
    int width_;
    int height_;
    vector<vector<int>> map_;

    double radius_;
    float end_dist_therehold;


    vector<Node *> nodes_;
    vector<Node *> path;
    Node *root, *lastNode;

    int max_iter_;
    int step_size_;

};

}  // namespace rrt_star_global_planner
