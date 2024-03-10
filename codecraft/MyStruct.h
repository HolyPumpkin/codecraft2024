#pragma once
#include <functional>
#include <iostream>
#include <vector>
#include <list>
#include <fstream>
#include <string>
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

struct Boat
{
	// 容量
	int capacity;

	// 位置（泊口ID、虚拟点-1）
	int pos;
	
	// 状态
	int status;
};

struct Berth
{
	// 左上角坐标
	int x, y;

	// 泊口运输到虚拟点的时间
	int transport_time;

	// 装载速度
	int loading_speed;
};

struct Robot
{
	// 坐标
	int x, y;

	// 是否携带物品 // 0：未携带；1：携带
	int is_carry;
	
	// 机器人状态 // 0：恢复；1：正常
	int status;

	// TODO 还需定义Good结构体
};