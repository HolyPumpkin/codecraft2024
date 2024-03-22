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

// ��ĳ����λΪ�����ѵõ��ĵ�
struct BerthPoint
{
	int dir;	//�õ������Ӧ��λ��·������0��ʾ����һ��1��ʾ����һ��2��ʾ����һ��3��ʾ����һ��-1��ʾ���ɴ�

	int dist;	//�õ㵽��Ӧ��λ�ľ��룬-1��ʾ���ɴ�

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
	// ��ͻ������
	int x, y;

	// �洢��ͻ�����id �� ��Ӧָ����±�
	vector<pair<int, int>> data;

	CollisionPoint() {};

	CollisionPoint(int _x, int _y, vector<pair<int, int>> _data) : x(_x), y(_y), data(_data) {};
};

struct Command
{
	// ָ������
	// // -1����ʾ��ָ��
	// ������ָ��
	// // 1��move
	// // 2��get
	// // 4��pull
	// �ִ�ָ��
	// // 8��ship
	// // 16��go 
	int key;

	// ��������/�ִ��� ID
	int id;

	// ���� 2 
	// 1.��Ϊ������moveָ����ʾ�ƶ�����0��ʾ����һ��1��ʾ����һ��2��ʾ����һ��3��ʾ����һ��
	// 2.��Ϊ������get��pullָ����޴˲���
	// 3.��Ϊ�ִ�shipָ����ʾ��λid��ȡֵ[0,9]
	// 4.��Ϊ�ִ�goָ����޴˲���
	// 
	// -1����ʾ�޲���
	int param_2;


	Command(int _key, int _id, int _param_2) : key(_key), id(_id), param_2(_param_2) {};
};

struct Boat
{
	// �����������װ����Ʒ��
	int capacity;

	// ��ǰװ����Ʒ����
	int cur_load;

	// λ�ã�����ID�������-1��
	int pos;

	// �ٷ�����״̬
	// 0����ʾ�ƶ�(����)�У�1����ʾ��������״̬(��װ��״̬���������״̬)��2����ʾ��λ��ȴ�״̬
	int status;

	// ��һ֡��״̬
	int last_status;

	// ��ʼװ����֡��
	int start_load_frame;

	// ����װ����֡��
	int end_load_frame;

	// װ����Ҫ��ʱ�䣬Ŀǰ����Ϊ�̶�ֵ1000����ʾһ�����ֵ
	int loading_time;

	// �Ƿ�����װ��
	bool is_loading;

	// ��λid�����������ִ�Ҫȥ�Ĳ�λ
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
	// ���ڵ�id
	int id;

	// ���Ͻ�����
	int x, y;

	// ��λ�ĳߴ�
	int r_size, c_size;

	// ���½�����
	int rdx, rdy;

	// �������䵽������ʱ��
	int transport_time;

	// װ���ٶ�
	int loading_speed;

	// ��ǰ��������
	int cur_goods_num;

	// ��ǰ�����ܼ�ֵ
	int cur_goods_val;

	// ��λ��ǰ�Ƿ�һ���ִ�ռ�ã����ִ�ȥ�����λʱ�����޸�Ϊ1�����ִ��뿪ʱ���޸�Ϊ0
	// 0Ϊδ��ռ�ã�1Ϊ��ռ��
	int is_occupied;

	// �����洢������ǰ���ڵĻ�����
	vector<int> rbt_seq;

	// �����洢�ɴ��ͼ��������
	// reachable_map[x][y]��ʾ�ľ��Ǹõ���ڵ�ǰ��λ���ԵĿɴ�״̬������ɴ����dirΪ�ߵ���λ�ķ���distΪ����
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
	// ������id
	int id;

	// ����
	int x, y;

	// �Ƿ�Я����Ʒ
	// 0��δЯ����1��Я��
	int is_carry;

	// is_carry�����������������ж�is_carry�Ƿ�ı�
	// ���������Ƿ��õ�����»����ֵ��Ϊ��is_carry����
	// �����is_carry��ͬ�����ʾ�������õ�����»���
	int last_is_carry;

	// �Ƿ�ָ��
	// 0����Ϊ����ȡ������δָ�ɣ�
	// 1����ָ�ɣ�
	// 2����Ϊ��ײ��δָ��
	int is_assigned;

	// ����ȥ�õĻ������ʧʱ��
	int good_end_frame;

	// ������״̬
	// 0���ָ���1������
	int status;

	// ��������һ֡��״̬
	// 0���ָ���1������
	int last_status;

	// ȡ��·���±꣬����·���±�
	int fetch_good_cur, send_good_cur;

	// ��ǰҪȥ�Ĳ�λid����ʼ��Ϊ-1������δ���ò��ڡ�
	int berth_id;

	// �����˵�ǰ��ֵ������Я������ļ�ֵ
	int robot_val;

	// ȡ��·��
	vector<pair<int, int>> fetch_good_path;

	// ����·��
	vector<pair<int, int>> send_good_path;

	// �Ƿ񲻿ɴ�ĳ����λ��1��ʾ�ɴ0��ʾ���ɴ�
	vector<int> is_ungettable;

	// ��ǰ�����˵Ŀɴ��ͼ������true��ʾ�ɴfalse��ʾ���ɴ�
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
		this->last_status = 1;	//��ʼ������״̬
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
		this->last_status = 1;	//��ʼ������״̬
		for (int i = 0; i < 10; ++i)
		{
			this->is_ungettable.push_back(1);
		}
	}
};

struct Good
{
	// ����
	int x, y;

	// time_to_live,1000֡
	int ttl;

	// ������ʼ֡
	int start_frame;

	// ������ֹ֡����ʧ��ʱ���
	int end_frame;

	// ��ֵ
	int val;

	//�Ƿ��ѱ�ָ�ɸ�ĳ������
	bool is_assigned;

	//�Ƿ��ĳ�������˲��ɴ�
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

	// ���캯����ÿ����Ʒ��������Ϊ1000֡��
	Good(int _x, int _y, int _val, int frame_id) : x(_x), y(_y), val(_val), ttl(1000), start_frame(frame_id), end_frame(frame_id + 1000), is_assigned(false) {
		for (int i = 0; i < 10; ++i)
		{
			this->is_ungettable[i] = false;
		}
	};

};

