#pragma once
#include <functional>
#include <vector>
#include <list>
#include <iostream>
using namespace std;

struct Point
{
    int x, y;    //������
    int F, G, H;  //F = G + H(̰��׼����F = H,��ʾ����Ŀ������)
    Point* parent; //ָ�򸸽ڵ�
    Point(int _x, int _y) : x(_x), y(_y), F(0), G(0), H(0), parent(nullptr) {}

    //Ϊ���ȼ���������
    bool operator < (const Point& b) const {
        return this->F < b.F;
    }

    //Ϊ��������
    bool operator == (const Point& c) const {
        return this->x == c.x && this->y == c.y;
    }

    //��ϣ����
    struct Hasher {
        std::size_t operator()(const Point& d) const {
            std::hash<int> int_hash;
            return int_hash(d.x) ^ (int_hash(d.y) << 1);
        }
    };
};