#pragma once
#include <functional>
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <fstream>
#include <string>
#include <algorithm>
#include <queue>

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

// 以某个泊位为起点广搜得到的点
struct BerthPoint
{
	int dir;	//该点走向对应泊位的路径方向，0表示右移一格，1表示左移一格，2表示上移一格，3表示下移一格，-1表示不可达

	int dist;	//该点到对应泊位的距离，-1表示不可达

	BerthPoint(int _dir, int _dist)
	{
		this->dir = _dir;
		this->dist = _dist;
	}

	BerthPoint()
	{
		this->dir = -1;
		this->dist = -1;
	}
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

	// 上一帧的状态
	int last_status;

	// 开始装货的帧数
	int start_load_frame;

	// 结束装货的帧数
	int end_load_frame;

	// 装货需要的时间，目前设置为固定值1000，表示一个最大值
	int loading_time;

	// 是否正在装货
	bool is_loading;

	// 泊位id，表明这艘轮船要去的泊位
	int berth_id;

	Boat()
	{
		this->capacity = 0;
		this->cur_load = 0;
		this->pos = 0;
		this->status = 0;
		this->is_loading = false;
		this->start_load_frame = 0;
		this->end_load_frame = 0;
		this->loading_time = 0;
		this->last_status = 0;
		this->berth_id = 0;
	}

	Boat(int _capacity, int pos, int _pos) : capacity(_capacity), cur_load(0), pos(_pos), status(0), start_load_frame(0), end_load_frame(0), loading_time(0), is_loading(false), last_status(0), berth_id(0){};
};

struct Berth
{
	// 泊口的id
	int id;

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

	// 泊位当前是否被一艘轮船占用，当轮船去这个泊位时，就修改为1，当轮船离开时，修改为0
	// 0为未被占用，1为被占用
	int is_occupied;

	// 用来存储锁定当前泊口的机器人
	vector<int> rbt_seq;

	// 用来存储可达地图及其属性
	// reachable_map[x][y]表示的就是该点对于当前泊位而言的可达状态，如果可达，则其dir为走到泊位的方向，dist为距离
	vector<vector<BerthPoint>> reachable_map;

	Berth() 
	{
		this->id = 0;
		this->x = 0;
		this->y = 0;
		this->r_size = 4;
		this->c_size = 4;
		this->rdx = 0;
		this->rdy = 0;
		this->transport_time = 0;
		this->loading_speed = 0;
		this->cur_goods_num = 0;
		this->cur_goods_val = 0;
		this->is_occupied = 0;
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
		this->is_occupied = 0;
	}
};

struct Robot
{
	// 机器人id
	int id;

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
	// 0：因为正常取物送物未指派；
	// 1：被指派；
	// 2：因为碰撞而未指派
	int is_assigned;

	// 正在去拿的货物的消失时间
	int good_end_frame;

	// 机器人状态
	// 0：恢复；1：正常
	int status;

	// 机器人上一帧的状态
	// 0：恢复；1：正常
	int last_status;

	// 取物路径下标，送物路径下标
	int fetch_good_cur, send_good_cur;

	// 当前要去的泊位id。初始化为-1，代表还未设置泊口。
	int berth_id;

	// 机器人当前价值，即所携带货物的价值
	int robot_val;

	// 取物路径
	vector<pair<int, int>> fetch_good_path;

	// 送物路径
	vector<pair<int, int>> send_good_path;

	// 是否不可达某个泊位，1表示可达，0表示不可达
	vector<int> is_ungettable;

	// 当前机器人的可达地图，其中true表示可达，false表示不可达
	vector<vector<bool>> reachable_map;

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
		this->good_end_frame = -1;
		this->berth_id = -1;
		this->robot_val = 0;
		this->last_status = 1;	//初始是正常状态
		for (int i = 0; i < 10; ++i)
		{
			this->is_ungettable.push_back(1);
		}
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
		this->good_end_frame = -1;
		this->berth_id = -1;
		this->robot_val = 0;
		this->last_status = 1;	//初始是正常状态
		for (int i = 0; i < 10; ++i)
		{
			this->is_ungettable.push_back(1);
		}
	}
};

struct Good
{
	// 坐标
	int x, y;

	// time_to_live,1000帧
	int ttl;

	// 货物起始帧
	int start_frame;

	// 货物终止帧，消失的时间点
	int end_frame;

	// 价值
	int val;

	//是否已被指派给某机器人
	bool is_assigned;

	//是否对某个机器人不可达
	bool is_ungettable[10];

	Good() 
	{
		this->x = 0;
		this->y = 0;
		this->ttl = 1000;
		this->val = 0;
		this->is_assigned = false;
		this->start_frame = 0;
		this->end_frame = 0;
		for (int i = 0; i < 10; ++i)
		{
			this->is_ungettable[i] = false;
		}
	}

	// 构造函数（每个物品生命周期为1000帧）
	Good(int _x, int _y, int _val, int frame_id) : x(_x), y(_y), val(_val), ttl(1000), start_frame(frame_id), end_frame(frame_id + 1000), is_assigned(false) {
		for (int i = 0; i < 10; ++i)
		{
			this->is_ungettable[i] = false;
		}
	};

};

