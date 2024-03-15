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

	// �Ƿ�����װ��
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
	// 0��δָ�ɣ�1����ָ��
	int is_assigned;

	// ������״̬
	// 0���ָ���1������
	int status;

	// ȡ��·���±꣬����·���±�
	int fetch_good_cur, send_good_cur;

	// ȡ��·��
	vector<pair<int, int>> fetch_good_path;

	// ����·��
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
	// ����
	int x, y;

	// time_to_live,��ʼΪ1000֡��ÿһ֡�Լ�1
	int ttl;

	// ��ֵ
	int val;

	//�Ƿ��ѱ�ָ�ɸ�ĳ������
	bool is_assigned;

	Good() 
	{
		this->x = 0;
		this->y = 0;
		this->ttl = 1000;
		this->val = 0;
		this->is_assigned = false;
	}

	// ���캯����ÿ����Ʒ��������Ϊ1000֡��
	Good(int _x, int _y, int _val) : x(_x), y(_y), val(_val), ttl(1000), is_assigned(false) {};
};