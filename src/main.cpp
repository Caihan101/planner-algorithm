#include <iostream>
#include <fstream>
#include <vector>

#include "rrt.h"
#include "rrtStar.h"
#include "biRRTStar.h"
#include "Astar.h"
#include "corridor.h"
#include "smoothosqpproblem.h"

#include "cavataFunc.hpp"
#include "TxtStream.hpp"

#define M_PI 3.1415926

using namespace std;

double getRandom(){

    return drand48();
}

int main()
{

    clock_t start_time;
    clock_t end_time;

    vector<pair<int, int>> obs_pts;
    fstream edge_file("../experiment/obstacle_edge.txt");
    ReadTxtData(edge_file, obs_pts);
    cutData(obs_pts);
    DataToTxt("./data/newEdge.txt", obs_pts);

    // //make_pair(10, 535), make_pair(287, 335)
    // //make_pair(304, 353), make_pair(33, 513)
    // //make_pair(304, 559), make_pair(35, 396)
    // //make_pair(365, 403), make_pair(295, 562)

    // cvtAstar::Astar A;
    // A.InitAstar(obs_pts, 400, 600);
    // vector<pair<double, double>> AstarPath = A.pathPlanning(make_pair(10, 535), make_pair(287, 335));
    // A.getCloseAndOpenSize();
    // DataToTxt("./data/astarPath.txt", AstarPath);

    cvtRRTStar::RRTStar RStar;
    RStar.initialize(obs_pts, 400, 600, make_pair(365, 403), make_pair(295, 562));
    vector<pair<double, double>>  rrtStarPath;

    start_time = clock();
    rrtStarPath = RStar.pathPlanning();
    end_time = clock();

    cout << "rrtStar time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << " s" << endl;
    cout << "rrtStar node size: " << RStar.node_size << endl;
    cout << "rrtStar iter count: " << RStar.iter_count << endl;

    double total_length = 0;

    for(size_t i = 0; i < rrtStarPath.size() - 1; i++){
        total_length += hypot(rrtStarPath[i+1].first - rrtStarPath[i].first, rrtStarPath[i+1].second - rrtStarPath[i].second);
    }
    cout << "coarse path length: " << total_length << endl;
 
    DataToTxt("./data/rrtStarPath.txt", rrtStarPath);

    cvtBiRRTStar::BiRRTStar BiRRTStar;
    BiRRTStar.initialize(obs_pts, 400, 600, make_pair(365, 403), make_pair(295, 562));
    vector<pair<double, double>> BiRRTStarPath;

    start_time = clock();
    BiRRTStarPath = BiRRTStar.pathPlanning();
    end_time = clock();
   
    cout << "bi rrtStar time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << " s" << endl;
    cout << "bi rrtStar node size: " << BiRRTStar.node_size << endl;
    cout << "bi rrtStar iter count: " << BiRRTStar.iter_count << endl;

    DataToTxt("./data/biRRTStarPath.txt", BiRRTStarPath);

    total_length = 0;

    for(size_t i = 0; i < BiRRTStarPath.size() - 1; i++){
        total_length += hypot(BiRRTStarPath[i+1].first - BiRRTStarPath[i].first, BiRRTStarPath[i+1].second - BiRRTStarPath[i].second);
    }

    cout << "coarse path length: " << total_length << endl;


    // pointVec obsTreePoints;
    // point_t pt;
    // for(size_t i = 0; i < obs_pts.size(); i++){
    //     pt = {(double)obs_pts[i].first, (double)obs_pts[i].second};
    //     obsTreePoints.push_back(pt);
    // }
    // KDTree obsTree(obsTreePoints);

    // corridor C;
    // C.initCorridor(obsTree, 0, 400, 0, 600);
    // start_time = clock();

    // C.update(rrtStarPath);

    // end_time = clock();

    // cout << "create corridor box time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << " s" << endl;

    // std::vector<std::vector<double>> boxes_corridor = C.boxes;

    // BoxDataToTxt("./data/boxData.txt", C.vis_boxes);

    // FemPosDeviationSqpOsqpInterface solver;

    // vector<pair<double,double>> opt_path;

    // solver.set_ref_points(rrtStarPath);
    // solver.set_boundsBox(C.boxes);

    // start_time = clock();

    // if(!solver.Solve()){
    //     cout << "opt fail" << endl;
    // }

    // end_time = clock();

    // cout << "opt time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << " s" << endl;

    // opt_path = solver.opt_xy();

    // vector<double> heading;
    // vector<double> curvate;
    // heading.resize(opt_path.size());
    // curvate.resize(opt_path.size());
    // double total_length = 0;
    
    // for(size_t i = 0; i < opt_path.size() - 1; i++){
    //     total_length += hypot(opt_path[i+1].first - opt_path[i].first, opt_path[i+1].second - opt_path[i].second);
    //     heading[i] = atan2(opt_path[i+1].second - opt_path[i].second, opt_path[i+1].first - opt_path[i].first);
    // }
    // heading[heading.size() - 1] = heading[heading.size() - 2];

    // cout << "total path: " << total_length << endl;

    // for(size_t i = 0; i < opt_path.size(); i++){
    //     double headTmp = 0;
    //     double lengthTmp = 0;
    //     if(i < opt_path.size() - 3 && i > 3){
    //         for(int n = i - 3; n < i + 3; n++){
    //             headTmp += heading[n+1] - heading[n];
    //             lengthTmp += hypot(opt_path[n+1].first - opt_path[n].first, opt_path[n+1].second - opt_path[n].second);
    //         }
    //         if (headTmp > 180) headTmp = headTmp - 360;
    //         if (headTmp < -180) headTmp = headTmp + 360;
    //         curvate[i] = headTmp * M_PI / 180 / lengthTmp;
    //     }
    //     else{
    //         curvate[i] = 0;
    //     }
    // }

    // double total_curvate = 0;

    // for(size_t i = 0; i < curvate.size() - 1; i++){
    //     total_curvate += curvate[i];
    // }
    
    // cout << "average curvature: " << total_curvate/curvate.size() << endl;

    // DataToTxt("./data/curvate.txt", curvate);
    // DataToTxt("./data/optPath.txt", opt_path);

    // cvtRRT::RRT R;
    // R.initialize(obs_pts, 400, 600, make_pair(365, 403), make_pair(295, 562));
    // vector<pair<double, double>> rrtPath = R.getPath();
    // DataToTxt("./data/rrtPath.txt", rrtPath);

    
    // cvtBiRRTStar::BiRRTStar BiRRTStar;
    // BiRRTStar.initialize(obs_pts, 400, 600, make_pair(365, 403), make_pair(295, 562));
    // vector<pair<double, double>> BiRRTStarPath;
    // BiRRTStarPath = BiRRTStar.pathPlanning();
    // DataToTxt("./data/biRRTStarPath.txt", BiRRTStarPath);

    return 0;
}