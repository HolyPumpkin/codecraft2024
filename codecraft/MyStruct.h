#pragma once
#include <functional>
#include <iostream>
#include <vector>
#include <list>
#include <map>
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

struct CollisionPoint
{
	// 冲突点坐标
	int x, y;

	// 存储冲突对象的id 和 对应指令的下标
	vector<pair<int, int>> data;

	CollisionPoint() {};

	CollisionPoint(int _x, int _y, vector<pair<int, int>> _data) : x(_x), y(_y), data(_data) {};
};

struct Command
{
	// 指令名称
	// 机器人指令
	// // 1：move；2：get；4：pull
	// 轮船指令
	// // 8：ship；16：go
	int key;

	// （机器人/轮船） ID
	int id;

	// 参数 2 
	// -1：表示无参数
	int param_2;
};

struct Boat
{
	// 容量
	int capacity;

	// 位置（泊口ID、虚拟点-1）
	int pos;

	// 状态
	// 0：运输中；1：装货或运输完成；2：泊外等待中
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

	Berth() {}

	Berth(int x, int y, int transport_time, int loading_speed) {
		this->x = x;
		this->y = y;
		this->transport_time = transport_time;
		this->loading_speed = loading_speed;
	}
};

struct Robot
{
	// 坐标
	int x, y;

	// 是否携带物品
	// 0：未携带；1：携带
	int is_carry;

	// 机器人状态
	// 0：恢复；1：正常
	int status;

	// 取物路径下标，送物路径下标
	int fetch_good_cur, send_good_cur;

	// 取物路径
	vector<pair<int, int>> fetch_good_path;

	// 送物路径
	vector<pair<int, int>> send_good_path;

	Robot() {}

	Robot(int x, int y) {
		this->x = x;
		this->y = y;
		this->is_carry = 0;
		this->status = 1;
	}
};

struct Good
{
	// 坐标
	int x, y;

	// time_to_live
	int ttl;

	// 价值
	int val;

	Good() {};

	// 构造函数（每个物品生命周期为20）
	Good(int _x, int _y, int _val) : x(_x), y(_y), val(_y), ttl(20) {};
};