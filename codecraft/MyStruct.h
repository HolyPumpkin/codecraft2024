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

struct Boat
{
	// ����
	int capacity;

	// λ�ã�����ID�������-1��
	int pos;
	
	// ״̬
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
};

struct Robot
{
	// ����
	int x, y;

	// �Ƿ�Я����Ʒ // 0��δЯ����1��Я��
	int is_carry;
	
	// ������״̬ // 0���ָ���1������
	int status;

	// TODO ���趨��Good�ṹ��
};