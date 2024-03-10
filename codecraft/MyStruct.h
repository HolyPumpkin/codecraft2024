#pragma once
#include <functional>
#include <vector>
#include <list>
#include <iostream>
using namespace std;

struct Point
{
    int x, y;    //点坐标
    int F, G, H;  //F = G + H(贪心准则中F = H,表示距离目标点距离)
    Point* parent; //指向父节点
    Point(int _x, int _y) : x(_x), y(_y), F(0), G(0), H(0), parent(nullptr) {}

    //为优先级队列重载
    bool operator < (const Point& b) const {
        return this->F < b.F;
    }

    //为集合重载
    bool operator == (const Point& c) const {
        return this->x == c.x && this->y == c.y;
    }

    //哈希函数
    struct Hasher {
        std::size_t operator()(const Point& d) const {
            std::hash<int> int_hash;
            return int_hash(d.x) ^ (int_hash(d.y) << 1);
        }
    };
};