#include "Astar.h"


namespace cvtAstar{

/**
 * @description: 初始化astar
 * @return {*}
 */



void Astar::InitAstar(const vector<pair<int, int>>& obsPoints, int height, int width)
{

    neighbor_ = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };

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

}

void Astar::mapUpdate(const vector<pair<int, int>>& obsPoints)
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

    updateObsPoints.clear();

    for(size_t i = 0; i < map_.size(); i++){
        for(size_t j = 0; j < map_[i].size(); j++){
            if(map_[i][j] == obstacle){
                updateObsPoints.push_back(make_pair(i, j));
            }
        }
    }

}


/**
 * @description: astar寻路
 */
vector<pair<double, double>> Astar::pathPlanning(pair<int, int> startPoint, pair<int, int> targetPoint){

    // Get variables
    startPoint_ = startPoint;
    targetPoint_ = targetPoint;
    vector<pair<double, double>> path;

    clock_t s_time, e_time;

    // Path Planning

    s_time = clock();
    Node* TailNode = FindPath();
    GetPath(TailNode, path);
    e_time = clock();

    cout << "astar time: " << (double)(e_time - s_time) / CLOCKS_PER_SEC << " s" << endl;

    releaseMemory();
    return path;
}

/**
 * @description: 寻路
 * @param {*}
 * @return {*}
 */
Node* Astar::FindPath()
{
    // Add startPoint to OpenList

    Node* startPointNode = new Node(startPoint_);
    OpenList.push(pair<int, pair<int, int>>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    map_[startPoint_.first][startPoint_.second] = inOpenList;

    int step = 0;
    
    while(!OpenList.empty())
    {
        // Find the node with least F value
       // cout << "start :" << step << endl;
        pair<int, int> CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        OpenDict.erase(index);

        int curX = CurPoint.first;
        int curY = CurPoint.second;

        map_[curX][curY] = inCloseList;


        // Determine whether arrive the target point
        if(curX == targetPoint_.first && curY == targetPoint_.second)
        {
            return CurNode; // Find a valid path
        }
       

        // Traversal the neighborhood
        for(int k = 0;k < neighbor_.size();k++)
        {   

            
            int y = curY + neighbor_[k][1];
            int x = curX + neighbor_[k][0];

        
            if(x < 0 || x >= height_ || y < 0 || y >= width_){
                continue;
            }


            if(map_[x][y] == free || map_[x][y] == inOpenList)
            {
    
                // Determine whether a diagonal line can pass
                int dist1 = abs(neighbor_[k][0]) + abs(neighbor_[k][1]);
                if(dist1 == 2 && map_[x][curY] == obstacle && map_[curX][y] == obstacle)
                    continue;

                
                // Calculate G, H, F value
                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                
                int dist2 = (x - targetPoint_.first) * (x - targetPoint_.first) + (y - targetPoint_.second) * (y - targetPoint_.second);
                H = round(10 * sqrt(dist2));

               // H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                F = G + H;
             
                // Update the G, H, F value of node
                if(map_[x][y] == free)
                {
                    Node* node = new Node();
                    node->point = make_pair(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(pair<int, pair<int, int>>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    map_[x][y] = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                     
                    int index = point2index(make_pair(x, y));
                    Node* node = OpenDict[index];

                    if(G < node->G)
                    {

                        
                        node->G = G;
                        node->F = F;
                        
                        node->parent = CurNode;
                        
                    }

                    
                }
                
            }

        }
        //cout << "end :" << step << endl;
        step++;
        
    }
    cout << "astar failure" << endl;
    return NULL; // Can not find a valid path
}


/**
 * @description: 反转路径
 * @param {Node*} TailNode
 * @return {*}
 */
void Astar::GetPath(Node* TailNode, vector<pair<double, double>>& path)
{
    PathList.clear();
    path.clear();

    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    // Save path to vector<Point>
    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        path.push_back(make_pair(PathList.back()->point.first, PathList.back()->point.second));
        PathList.pop_back();
    }

}

void Astar::releaseMemory()
{
    while(OpenList.size()) {
        pair<int, int> CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}

void Astar::getCloseAndOpenSize(){
    int openSize = 0;
    int CloseSize = 0;
    int freesize = 0;
    int obsszie = 0;
    for(size_t i = 0; i < map_.size(); i++){
        for(size_t j = 0; j < map_[i].size(); j++){
            if(map_[i][j] == inOpenList){
                openSize++;
            }
            else if(map_[i][j] == inCloseList){
                CloseSize++;
            }
            else{
                continue;
            }
        }
    }
    cout << "opensize: " << openSize << endl;
    cout << "closesize: " << CloseSize << endl;
}


} 