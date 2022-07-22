#include "rrt.h"

namespace cvtRRT
{

RRT::RRT()
{
    end_dist_therehold = 5;
    step_size_ = 3;
    max_iter_ = 10000;
}

/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize(const vector<pair<int, int>>& obsPoints, int height, int width,
                    pair<double, double> startPoint, pair<double, double> targetPoint)
{

    height_ = height;
    width_ = width;

    map_.resize(height_);
    for(size_t i = 0; i < map_.size(); i++){
        map_[i].resize(width_);
    }

    for(size_t i = 0; i < obsPoints.size(); i++){
        map_[obsPoints[i].first][obsPoints[i].second] = obstacle;
    }

    startPos = startPoint;
    endPos = targetPoint;

    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
}


vector<pair<double, double>> RRT::getPath()
{
    for(int i = 0; i < max_iter_; i++)
    {
        Node *q = this->getRandomNode();
        if (q) {
            Node *qNearest = this->nearest(q->position);
            if (this->distance2D(q->position, qNearest->position) > this->step_size_) {
                pair<double, double> newConfig = this->newConfig(q, qNearest);
                if (!isThereObstacleBetween(qNearest->position, newConfig)) {
                    Node *qNew = new Node;
                    qNew->position = newConfig;
                    this->add(qNearest, qNew);
                }
            }
        }
        if (this->reached()) {
            cout << "rrt reach" << endl;
            break;
        }
    }

    Node *q;
    if (this->reached()) {
        q = this->lastNode;
    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q = this->nearest(this->endPos);
    }
    // generate shortest path to destination.
    while (q != NULL) {
        this->path.push_back(q);
        q = q->parent;
    }

    vector<pair<double, double>> rrtPath;

    rrtPath.resize(path.size());

    for(size_t i =0; i < this->path.size(); i++){
        rrtPath[i].first = this->path.at(i)->position.first;
        rrtPath[i].second = this->path.at(i)->position.second;
    }

    return rrtPath;
}




/**
 * @brief Generate a random node in the field.
 * @return
 */
inline Node* RRT::getRandomNode()
{
    Node* ret;
    pair<double, double> point(drand48() * height_, drand48() * width_);
    if (point.first >= 0 && point.first <= height_ && point.second >= 0 && point.second <= width_) {
        ret = new Node;
        ret->position = point;
        return ret;
    }
    return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
inline double RRT::distance2D(pair<double, double> &p, pair<double, double> &q)
{
    pair<double, double> v = make_pair(p.first - q.first, p.second - q.second);
    return sqrt(powf(v.first, 2) + powf(v.second, 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
inline Node* RRT::nearest(pair<double, double> &point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        float dist = distance2D(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
inline pair<double, double> RRT::newConfig(Node *q, Node *qNearest)
{
    Vector2f to(q->position.first, q->position.second);
    Vector2f from(qNearest->position.first, qNearest->position.second);
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size_ * intermediate;
    return make_pair(ret.x(), ret.y());
}

template <typename T>
inline bool RRT::isPointInObstacle(pair<T, T> &p)
{
    if(map_[(int)p.first][(int)p.second] == obstacle
       || map_[(int)p.first + 1][(int)p.second] == obstacle
       || map_[(int)p.first][(int)p.second + 1] == obstacle
       || map_[(int)p.first - 1][(int)p.second] == obstacle
       || map_[(int)p.first][(int)p.second - 1] == obstacle){

        return true;
    }
    else{
        return false;
    }
}

/**
 * @brief check obstalce between two points.
 * @param q
 * @param p
 * @return
 */
inline bool RRT::isThereObstacleBetween(pair<double, double> &p, pair<double, double> &q)
{
    double dist = distance2D(p,q);
    if(dist < 1){
        if(isPointInObstacle(p) || isPointInObstacle(q)){
            return true;
        }
        else{
            return false;;
        }
    }
    else{
        int step_number = static_cast<int>(dist/1);
        //cout << "step number" << step_number << endl;
        double dx = (q.first - p.first) / step_number;
        double dy = (q.second - p.second) / step_number;
        //float theta = atan2(q.first - p.first, q.second - p.second);
        pair<double, double> p_tmp;
        for(int i = 0; i < step_number; i++){
            p_tmp.first = p.first + i * dx;
            p_tmp.second = p.second + i * dy;

            if(isPointInObstacle(p_tmp)){
                return true;
            }
        }
        return false;
    }
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
inline void RRT::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
inline bool RRT::reached()
{
    if (distance2D(lastNode->position, endPos) < end_dist_therehold)
        return true;
    return false;
}


/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
inline void RRT::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}


}

