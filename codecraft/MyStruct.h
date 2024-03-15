#pragma once
#include <functional>
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <fstream>
#include <string>
#include <algorithm>
using namespace std;

const int testMapSize = 210;

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
	// // -1：表示空指令
	// 机器人指令
	// // 1：move
	// // 2：get
	// // 4：pull
	// 轮船指令
	// // 8：ship
	// // 16：go 
	int key;

	// （机器人/轮船） ID
	int id;

	// 参数 2 
	// 1.若为机器人move指令，则表示移动方向，0表示右移一格，1表示左移一格，2表示上移一格，3表示下移一格
	// 2.若为机器人get、pull指令，则无此参数
	// 3.若为轮船ship指令，则表示泊位id，取值[0,9]
	// 4.若为轮船go指令，则无此参数
	// 
	// -1：表示无参数
	int param_2;


	Command(int _key, int _id, int _param_2) : key(_key), id(_id), param_2(_param_2) {};
};

struct Boat
{
	// 容量，最多能装的物品数
	int capacity;

	// 当前装载物品数量
	int cur_load;

	// 位置（泊口ID、虚拟点-1）
	int pos;

	// 官方输入状态
	// 0：表示移动(运输)中；1：表示正常运行状态(即装货状态或运输完成状态)；2：表示泊位外等待状态
	int status;

	// 是否正在装货
	bool is_loading;

	Boat()
	{
		this->capacity = 0;
		this->cur_load = 0;
		this->pos = 0;
		this->status = 2;
		this->is_loading = false;
	}

	Boat(int _capacity, int pos, int _pos) : capacity(_capacity), cur_load(0), pos(_pos), status(2), is_loading(false) {};
};

struct Berth
{
	// 左上角坐标
	int x, y;

	// 泊位的尺寸
	int r_size, c_size;

	// 右下角坐标
	int rdx, rdy;

	// 泊口运输到虚拟点的时间
	int transport_time;

	// 装载速度
	int loading_speed;

	// 当前货物数量
	int cur_goods_num;

	// 当前货物总价值
	int cur_goods_val;

	Berth() 
	{
		this->x = 0;
		this->y = 0;
		this->r_size = 4;
		this->r_size = 4;
		this->rdx = 0;
		this->rdy = 0;
		this->transport_time = 0;
		this->loading_speed = 0;
		this->cur_goods_num = 0;
		this->cur_goods_val = 0;
	}

	Berth(int x, int y, int transport_time, int loading_speed, int row, int col) {
		this->x = x;
		this->y = y;
		this->transport_time = transport_time;
		this->loading_speed = loading_speed;
		this->cur_goods_num = 0;
		this->cur_goods_val = 0;
		this->r_size = row;
		this->c_size = col;
		this->rdx = x + row;
		this->rdy = y + col;
	}
};

struct Robot
{
	// 坐标
	int x, y;

	// 是否携带物品
	// 0：未携带；1：携带
	int is_carry;

	// is_carry的孪生变量，用来判断is_carry是否改变
	// 即机器人是否拿到或放下货物，初值设为与is_carry互异
	// 如果和is_carry相同，则表示机器人拿到或放下货物
	int last_is_carry;

	// 是否被指派
	// 0：未指派；1：被指派
	int is_assigned;

	// 机器人状态
	// 0：恢复；1：正常
	int status;

	// 取物路径下标，送物路径下标
	int fetch_good_cur, send_good_cur;

	// 取物路径
	vector<pair<int, int>> fetch_good_path;

	// 送物路径
	vector<pair<int, int>> send_good_path;

	Robot() 
	{
		this->x = 0;
		this->y = 0;
		this->is_carry = 0;
		this->status = 1;
		this->fetch_good_cur = 0;
		this->send_good_cur = 0;
		this->is_assigned = 0;
		this->last_is_carry = 1;
	}

	Robot(int x, int y) {
		this->x = x;
		this->y = y;
		this->is_carry = 0;
		this->status = 1;
		this->fetch_good_cur = 0;
		this->send_good_cur = 0;
		this->is_assigned = 0;
		this->last_is_carry = 1;
	}
};

struct Good
{
	// 坐标
	int x, y;

	// time_to_live,初始为1000帧，每一帧自减1
	int ttl;

	// 价值
	int val;

	//是否已被指派给某机器人
	bool is_assigned;

	Good() 
	{
		this->x = 0;
		this->y = 0;
		this->ttl = 1000;
		this->val = 0;
		this->is_assigned = false;
	}

	// 构造函数（每个物品生命周期为1000帧）
	Good(int _x, int _y, int _val) : x(_x), y(_y), val(_val), ttl(1000), is_assigned(false) {};
};