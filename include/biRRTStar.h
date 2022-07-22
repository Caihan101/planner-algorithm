#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <random>
#include <cfloat>
#include <eigen3/Eigen/Eigen>
#include <iostream>

#include "KDTree.hpp"

using namespace std;
using namespace Eigen;

namespace cvtBiRRTStar
{
    
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

struct Node 
{

    Node *parent = nullptr;
    pair<double, double> position = {0, 0};
    float cost{0.0};

};

class BiRRTStar {
public:
    BiRRTStar();

    void initialize(const vector<pair<int, int>>& obsPoints, int height, int width,
                    pair<double, double> startPoint, pair<double, double> targetPoint);

    vector<pair<double, double>> pathPlanning();


    int node_size;
    int iter_count;

private:

    void mapUpdate(const vector<pair<int, int>>& obsPoints);

    inline Node* getRandomNode();

    inline Node* nearest(pair<double, double> &point, bool isStartTree);

    inline pair<double, double> newConfig(Node *q, Node *qNearest);

    inline void start_add(Node *qNearest, Node *qNew);

    inline void end_add(Node *qNearest, Node *qNew);

    inline bool reached();

    inline void deleteNodes(Node *root);

    inline bool isThereObstacleBetween(pair<double, double> &p, pair<double, double> &q);

    template <typename T>
    inline bool isPointInMap(T x, T y);


    void getNearestTwoNode(Node &p, Node &q);

    template <typename T>
    inline bool isPointInObstacle(pair<T, T> &p);


    std::pair<double, double> start_point_;
    std::pair<double, double> goal_point_;

    
    int width_;
    int height_;
    vector<vector<int>> map_;

    double radius_;
    float end_dist_therehold;

    vector<Node *> start_path;
    vector<Node *> end_path;

    vector<Node *> start_nodes_;
    vector<Node *> end_nodes_;
    

    Node *start_root, *start_lastNode;
    Node *end_root, *end_lastNode;

    int max_iter_;
    int step_size_;

    pointVec endTreePoints;

    int step;


};


}
