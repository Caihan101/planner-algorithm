#ifndef RRT_H
#define RRT_H

#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>


using namespace std;
using namespace Eigen;

namespace cvtRRT
{

enum NodeType{
    free = 0,
    obstacle
};

struct Node {
    vector<Node *> children;
    Node *parent;
    pair<double, double> position;
};

class RRT
{
public:
    RRT();
    void initialize(const vector<pair<int, int>>& obsPoints, int height, int width,
                    pair<double, double> startPoint, pair<double, double> targetPoint);
    
    
    vector<pair<double, double>> getPath();

    void setMaxIter(const int &maxIter){max_iter_ = maxIter;};
    void setStepSize(const int &stepSize){step_size_ = stepSize;};


private:

    inline  Node* getRandomNode();
    inline Node* nearest(pair<double, double> &point);
    inline double distance2D(pair<double, double> &p, pair<double, double> &q);
    inline void add(Node *qNearest, Node *qNew);
    inline pair<double, double> newConfig(Node *q, Node *qNearest);
    inline bool isThereObstacleBetween(pair<double, double> &p, pair<double, double> &q);
    inline bool reached();
    inline void deleteNodes(Node *root);

    template <typename T>
    inline bool isPointInObstacle(pair<T, T> &p);

    //与地图相关参数
    int height_;
    int width_;
    vector<vector<int>> map_;

    int max_iter_;
    int step_size_;

    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;


    pair<double, double> startPos, endPos;

    float end_dist_therehold;


};

}

#endif // RRT_H
