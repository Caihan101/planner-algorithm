#pragma once
#include <iostream>
#include <fstream>
#include <vector>

void ReadTxtData(std::fstream &file, std::vector<std::pair<int, int>> &pts)
{
    std::string s;

    int x,y;
    int i = 0;
    while(file >> s)
    {
        if(i == 2){
            i = 0;
        }

        if(i == 0){
            x = atoi(s.c_str());
        }
        else{
            y = atoi(s.c_str());
            pts.push_back(std::make_pair(x, y));
        }
        i++;
    }
}

void cutData(std::vector<std::pair<int, int>> &pts)
{
    int i = 0;
    while(i < pts.size() - 1)
    {
        if(pts[i] == pts[i+1]){
            pts.erase(pts.begin() + i + 1);
        }
        else{
            i++;
        }
    }
}

template <typename T1, typename T2>
void DataToTxt(std::string txtPath, std::vector<std::pair<T1, T2>> &pts)
{
    fstream file(txtPath);
    for(size_t i = 0; i < pts.size(); i++){
        file << pts[i].first << "," << pts[i].second << endl;
    }
}

void DataToTxt(std::string txtPath, std::vector<double> &pts)
{
    fstream file(txtPath);
    for(size_t i = 0; i < pts.size(); i++){
        file << pts[i] << endl;
    }
}


void BoxDataToTxt(std::string txtPath, std::vector<std::vector<double>> &boxes)
{
    fstream file(txtPath);
    for(size_t i = 0; i < boxes.size(); i++){
        file << boxes[i][0] << "," << boxes[i][1] << "," << boxes[i][2] << "," << boxes[i][3] << "," << endl;
    }
}