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
	// ������ָ��
	// // 1��move��2��get��4��pull
	// �ִ�ָ��
	// // 8��ship��16��go
	int key;

	// ��������/�ִ��� ID
	int id;

	// ���� 2 
	// -1����ʾ�޲���
	int param_2;
};

struct Boat
{
	// ����
	int capacity;

	// λ�ã�����ID�������-1��
	int pos;

	// ״̬
	// 0�������У�1��װ����������ɣ�2������ȴ���
	int status;
};

struct Berth
{
	// ���Ͻ�����
	int x, y;

	// �������䵽������ʱ��
	int transport_time;

	// װ���ٶ�
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
	// ����
	int x, y;

	// �Ƿ�Я����Ʒ
	// 0��δЯ����1��Я��
	int is_carry;

	// ������״̬
	// 0���ָ���1������
	int status;

	// ȡ��·���±꣬����·���±�
	int fetch_good_cur, send_good_cur;

	// ȡ��·��
	vector<pair<int, int>> fetch_good_path;

	// ����·��
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
	// ����
	int x, y;

	// time_to_live
	int ttl;

	// ��ֵ
	int val;

	Good() {};

	// ���캯����ÿ����Ʒ��������Ϊ20��
	Good(int _x, int _y, int _val) : x(_x), y(_y), val(_y), ttl(20) {};
};