#pragma once
#include <vector>
#include <iostream>
#include <stdio.h>

#define TEST printf("test\n")

void debug(){
    printf("test\n");
}

namespace cavataFunc{

template <typename T>
void printVector1d(const std::vector<T> &v)
{
    for(size_t i = 0; i < v.size(); i++)
    {
        std::cout << v.at(i) << std::endl;
    }
}

template <typename T1, typename T2>
void printVectorPair(const std::vector<std::pair<T1, T2>> &v)
{
    for(size_t i = 0; i < v.size(); i++)
    {
        std::cout << v.at(i).first << "  "  << v.at(i).second <<std::endl;
    }
}

vector<int> unique_element_in_vector(vector<int> v) {
    vector<int>::iterator vector_iterator;
    sort(v.begin(), v.end());
    vector_iterator = unique(v.begin(), v.end());
    if (vector_iterator != v.end()) {
        v.erase(vector_iterator, v.end());
    }
    return v;
}

}