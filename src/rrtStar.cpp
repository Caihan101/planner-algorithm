#include "rrtStar.h"


namespace cvtRRTStar {



RRTStar::RRTStar()                                     
{
    radius_ = 20;
    end_dist_therehold = 5.0;
    step_size_ = 5;
    max_iter_ = 100000000;
}

void RRTStar::initialize(const vector<pair<int, int>>& obsPoints, int height, int width,
                    pair<double, double> startPoint, pair<double, double> targetPoint){

    height_ = height;
    width_ = width;

    map_.resize(height_);
    for(size_t i = 0; i < map_.size(); i++){
        map_[i].resize(width_);
    }

    for(size_t i = 0; i < obsPoints.size(); i++){
        map_[obsPoints[i].first][obsPoints[i].second] = obstacle;
    }

    mapUpdate(obsPoints);


    start_point_ = startPoint;
    goal_point_ = targetPoint;

    root = new Node;
    root->parent = nullptr;
    root->position = start_point_;
    root->cost = 0.0;
    lastNode = root;
    nodes_.push_back(root);

    node_size = 0;

}

void RRTStar::mapUpdate(const vector<pair<int, int>>& obsPoints)
{
    vector<pair<int, int>> bound64;
    bound64.resize(81);
    int x,y;
    x = -4;
    y = -4;
    for(int i = 0; i < bound64.size(); i++){
        if(x == 0 && y ==0){
            x++;
            continue;
        }
        if(x > 4){
            x = -4;
            y++;
        }
        bound64[i] = {x, y};
        x++;
    }

    for(size_t i = 0; i < obsPoints.size(); i++){
        for (size_t j = 0; j < bound64.size(); j++)
        {
            int bound_x = obsPoints[i].first + bound64[j].first;
            int bound_y = obsPoints[i].second + bound64[j].second;
            if(bound_x > 0 && bound_x < height_ && bound_y > 0 && bound_y < width_)
            {
                map_[bound_x][bound_y] = obstacle;
            } 
        }
    }

}


vector<pair<double, double>> RRTStar::pathPlanning() 
{

    srand48(time(NULL));

    for(int i = 0; i < max_iter_; i++)
    {
        iter_count = i+1;

        Node *q = this->getRandomNode();
        if (q) {

            Node *qNearest = this->nearest(q->position);


            if (euclideanDistance(q->position, qNearest->position) > this->step_size_) {
                

                pair<double, double> newConfig = this->newConfig(q, qNearest);


                if (!isThereObstacleBetween(qNearest->position, newConfig)) {
                
                    Node *qNew = new Node;
                    qNew->position = newConfig;
                    this->add(qNearest, qNew);
                    node_size++;
                }
            }
        }
        if (this->reached()) {
            cout << "rrtStar reach" << endl;
            break;
        }
    }

    
    Node *tmp;
    if (this->reached()) {
        tmp = this->lastNode;
    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        cout << "fail" << endl;
        tmp = nearest(goal_point_);
    }
    // generate shortest path to destination.
    while (tmp != nullptr) {
        path.push_back(tmp);
        tmp = tmp->parent;
    }

    vector<pair<double, double>> rrtStarPath;

    rrtStarPath.resize(path.size());

    for(size_t i =0; i < this->path.size(); i++){
        rrtStarPath[i].first = this->path.at(i)->position.first;
        rrtStarPath[i].second = this->path.at(i)->position.second;
    }


    return rrtStarPath;
}

inline Node* RRTStar::getRandomNode()
{
    
    Node* ret;
    pair<double, double> point(drand48() * height_, drand48() * width_);

    if (point.first > 0 && point.first < height_ && point.second > 0 && point.second < width_) {
        ret = new Node;
        ret->position = point;
        return ret;
    }
    return nullptr;
}

inline pair<double, double> RRTStar::newConfig(Node *q, Node *qNearest)
{
    Vector2f to(q->position.first, q->position.second);
    Vector2f from(qNearest->position.first, qNearest->position.second);
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size_ * intermediate;
    return make_pair(ret.x(), ret.y());
}

inline void RRTStar::add(Node *qNearest, Node *qNew)
{

    double cost_new_node, cost_other_parent;
    cost_new_node = qNearest->cost + euclideanDistance(qNearest->position, qNew->position);

    //choose parent
    for (auto &node : nodes_) {

        if (node == qNew) continue;
        // distance between node and new_node
        double dist = euclideanDistance(node->position, qNew->position);

        cost_other_parent = node->cost + dist;

        if (dist < radius_ && cost_other_parent < cost_new_node && !isThereObstacleBetween(node->position, qNew->position))
        {
            qNew->parent = node;
            qNew->cost = cost_other_parent;
        }
        else{
            qNew->parent = qNearest;
            qNew->cost = cost_new_node;
        }

    }

    nodes_.push_back(qNew);
    lastNode = qNew;

    //rewire
    for(auto &node : nodes_){
        double dist = euclideanDistance(node->position, lastNode->position);
        if(node->parent != nullptr && dist < radius_){
            double cost_node = lastNode->cost + dist;
            if(cost_node < node->cost && !isThereObstacleBetween(node->position, lastNode->position)){
                node->parent = lastNode;
                node->cost = cost_node;
            }
        }
    }

   
}



inline Node* RRTStar::nearest(pair<double, double> &point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes_.size(); i++) {
        float dist = euclideanDistance(point, nodes_[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes_[i];
        }
    }
    return closest;
}



template <typename T>
inline bool RRTStar::isPointInObstacle(pair<T, T> &p)
{
    if(!isPointInMap((int)p.first, (int)p.second))
    //    || !isPointInMap((int)p.first + 1, (int)p.second)
    //    || !isPointInMap((int)p.first, (int)p.second + 1)
    //    || !isPointInMap((int)p.first - 1, (int)p.second)
    //    || !isPointInMap((int)p.first, (int)p.second - 1))
    {
        return true;
    }

    if(map_[(int)p.first][(int)p.second] == obstacle)
    //    || map_[(int)p.first + 1][(int)p.second] == obstacle
    //    || map_[(int)p.first][(int)p.second + 1] == obstacle
    //    || map_[(int)p.first - 1][(int)p.second] == obstacle
    //    || map_[(int)p.first][(int)p.second - 1] == obstacle)
    {

        return true;
    }
    else{
        return false;
    }
}

template <typename T>
inline bool RRTStar::isPointInMap(T x, T y)
{
    return x > 0 && x < height_ && y > 0 && y < width_;
}



/**
 * @brief check obstalce between two points.
 * @param q
 * @param p
 * @return
 */
inline bool RRTStar::isThereObstacleBetween(pair<double, double> &p, pair<double, double> &q)
{
    double dist = euclideanDistance(p,q);
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
        double dx = (q.first - p.first) / step_number;
        double dy = (q.second - p.second) / step_number;
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

inline bool RRTStar::reached()
{
    Node* tmp = nodes_.back();
    if (euclideanDistance(tmp->position, goal_point_) < end_dist_therehold && !isThereObstacleBetween(tmp->position, goal_point_))
        return true;
    return false;
}




}  // namespace rrt_star_global_planner
