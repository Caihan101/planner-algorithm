#include "biRRTStar.h"


namespace cvtBiRRTStar {

BiRRTStar::BiRRTStar()                                     
{
    endTreePoints.resize(0);
    radius_ = 10;
    end_dist_therehold = 5.0;
    step_size_ = 5;
    max_iter_ = 100000000;
    step = 0;
}

void BiRRTStar::initialize(const vector<pair<int, int>>& obsPoints, int height, int width,
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

    start_root = new Node;
    start_root->parent = nullptr;
    start_root->position = start_point_;
    start_root->cost = 0.0;
    start_lastNode = start_root;
    start_nodes_.push_back(start_root);

    end_root = new Node;
    end_root->parent = nullptr;
    end_root->position = goal_point_;
    end_root->cost = 0.0;
    end_lastNode = end_root;
    end_nodes_.push_back(end_root);

    node_size = 0;
    iter_count = 0;

}

void BiRRTStar::mapUpdate(const vector<pair<int, int>>& obsPoints)
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


vector<pair<double, double>> BiRRTStar::pathPlanning() 
{

    double start_goal_dis = hypot(start_point_.first - goal_point_.first, start_point_.second - goal_point_.second);
    int min_step = start_goal_dis/step_size_ - 1;

    srand48(time(NULL));
    for(int i = 0; i < max_iter_; i++)
    {

        Node *q = getRandomNode();
        if (q) {
            Node *qNearest = this->nearest(q->position, true);
            if (euclideanDistance(q->position, qNearest->position) > this->step_size_) {
                pair<double, double> newConfig = this->newConfig(q, qNearest);
                if (!isThereObstacleBetween(qNearest->position, newConfig)) {
                    Node *qNew = new Node;
                    qNew->position = newConfig;
                    this->start_add(qNearest, qNew);
                    node_size++;
                }
            }
        }


        Node *p = getRandomNode();
        if(p){
            Node *pNearest = this->nearest(p->position, false);
            if (euclideanDistance(p->position, pNearest->position) > this->step_size_) {
                pair<double, double> newConfig = this->newConfig(p, pNearest);
                if (!isThereObstacleBetween(pNearest->position, newConfig)) {
                    Node *pNew = new Node;
                    pNew->position = newConfig;
                    this->end_add(pNearest, pNew);
                    node_size++;
                }
            }
        }
        if( i > min_step){
            //cout << "i" << i << endl;
            if (this->reached()) {
                cout << "BiRRTStar reach" << endl;
                break;
            }
        }

        iter_count = i;

    }

    Node *start;
    Node *end;

    Node s_tmp, e_tmp;
    getNearestTwoNode(s_tmp, e_tmp);
    start = &s_tmp;
    end = &e_tmp;

    // generate shortest path to destination.
    while (start != NULL) {
        this->start_path.push_back(start);
        start = start->parent;
    }

    while (end != NULL) {
        this->end_path.push_back(end);
        end = end->parent;
    }

    vector<pair<double, double>> startPath;
    vector<pair<double, double>> endPath;

    startPath.resize(start_path.size());
    for(size_t i =0; i < this->start_path.size(); i++){
        startPath[i].first = this->start_path.at(i)->position.first;
        startPath[i].second = this->start_path.at(i)->position.second;
    }

    endPath.resize(end_path.size());
    for(size_t i =0; i < this->end_path.size(); i++){
        endPath[i].first = this->end_path.at(i)->position.first;
        endPath[i].second = this->end_path.at(i)->position.second;
    }
    reverse(startPath.begin(), startPath.end());
    startPath.insert(startPath.end(), endPath.begin(), endPath.end());
    return startPath;
}

inline Node* BiRRTStar::getRandomNode()
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

inline pair<double, double> BiRRTStar::newConfig(Node *q, Node *qNearest)
{
    Vector2f to(q->position.first, q->position.second);
    Vector2f from(qNearest->position.first, qNearest->position.second);
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size_ * intermediate;
    return make_pair(ret.x(), ret.y());
}


inline void BiRRTStar::start_add(Node *qNearest, Node *qNew)
{
    double cost_new_node, cost_other_parent;
    cost_new_node = qNearest->cost + euclideanDistance(qNearest->position, qNew->position);

    //choose parent
    for (auto &node : start_nodes_) {

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

    start_nodes_.push_back(qNew);
    start_lastNode = qNew;

    //rewire
    for(auto &node : start_nodes_){
        double dist = euclideanDistance(node->position, start_lastNode->position);
        if(node->parent != nullptr && dist < radius_){
            double cost_node = start_lastNode->cost + dist;
            if(cost_node < node->cost && !isThereObstacleBetween(node->position, start_lastNode->position)){
                node->parent = start_lastNode;
                node->cost = cost_node;
            }
        }
    }
}

inline void BiRRTStar::end_add(Node *qNearest, Node *qNew)
{
    double cost_new_node, cost_other_parent;
    cost_new_node = qNearest->cost + euclideanDistance(qNearest->position, qNew->position);

    //choose parent
    for (auto &node : end_nodes_) {

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

    end_nodes_.push_back(qNew);
    end_lastNode = qNew;

    point_t pt = {qNew->position.first, qNew->position.second};
    endTreePoints.push_back(pt);
    

    //rewire
    for(auto &node : end_nodes_){
        double dist = euclideanDistance(node->position, end_lastNode->position);
        if(node->parent != nullptr && dist < radius_){
            double cost_node = end_lastNode->cost + dist;
            if(cost_node < node->cost && !isThereObstacleBetween(node->position, end_lastNode->position)){
                node->parent = end_lastNode;
                node->cost = cost_node;
            }
        }
    }
}


inline Node* BiRRTStar::nearest(pair<double, double> &point, bool isStartTree)
{
    float minDist = 1e9;
    Node *closest = NULL;

    if(isStartTree){
        for(int i = 0; i < (int)start_nodes_.size(); i++) {
            float dist = euclideanDistance(point, start_nodes_[i]->position);
            if (dist < minDist) {
                minDist = dist;
                closest = start_nodes_[i];
            }
        }
    }
    else{
        for(int i = 0; i < (int)end_nodes_.size(); i++) {
            float dist = euclideanDistance(point, end_nodes_[i]->position);
            if (dist < minDist) {
                minDist = dist;
                closest = end_nodes_[i];
            }
        }
    }

    return closest;
}

template <typename T>
inline bool BiRRTStar::isPointInMap(T x, T y)
{
    return x > 0 && x < height_ && y > 0 && y < width_;
}


template <typename T>
inline bool BiRRTStar::isPointInObstacle(pair<T, T> &p)
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

/**
 * @brief check obstalce between two points.
 * @param q
 * @param p
 * @return
 */
inline bool BiRRTStar::isThereObstacleBetween(pair<double, double> &p, pair<double, double> &q)
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

inline bool BiRRTStar::reached()
{

    for(auto& node: end_nodes_){
        if(euclideanDistance(start_lastNode->position, node->position) < end_dist_therehold 
           && !isThereObstacleBetween(start_lastNode->position, node->position))
        {
            return true;
        }
    }

    for(auto& node: start_nodes_){
        if(euclideanDistance(end_lastNode->position, node->position) < end_dist_therehold 
           && !isThereObstacleBetween(end_lastNode->position, node->position))
        {
            return true;
        }
    }

    return false;

    // if(endTreePoints.size() < 2){
    //     return false;
    // }

    // Node qTmp;
    // Node pTmp;
    // getNearestTwoNode(pTmp, qTmp);

    // if(euclideanDistance(qTmp.position, pTmp.position) < end_dist_therehold && !isThereObstacleBetween(qTmp.position, pTmp.position)){
    //     return true;
    // }
    // else{
    //     return false;
    // }
}



inline void BiRRTStar::getNearestTwoNode(Node& p, Node& q)
{
    
    KDTree *endTree = new KDTree(endTreePoints);

    double min_dis = 100000;

    pair<double, double> qPos;

    for(auto& node:start_nodes_){
        point_t pt = {node->position.first, node->position.second};
        auto res = endTree->nearest_point(pt);
        double dis = sqrt(pow(pt[0] - res[0], 2) + pow(pt[1] - res[1], 2));
        if(dis < min_dis){
            min_dis = dis;
            p = *node;
            qPos = make_pair(res[0], res[1]);
        }
    }

    delete endTree;

    for(auto& node:end_nodes_)
    {
        if(node->position == qPos){
            q = *node;
        }
    }


}

}  
