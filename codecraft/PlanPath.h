#pragma once
#include "MyStruct.h"


class PlanPath
{
public:
	/* ���캯�� */
	PlanPath() = default;
	explicit PlanPath(vector<vector<char>>& _maze, int _N, int _n, vector<Robot> robots); // ͨ����ά�ַ���ͼ��ʼ��

	/* ��Ա���� */
	int getDis(Point& start, Point& end);

	Point* getMinF(list<Point*>& l);

	bool isInlist(list<Point*>& l, Point& p);

	Point* getPoint(list<Point*>& l, Point& p);

	void printPath(vector<pair<int, int>>& path);

	/* ���ⲿ��¶�Ľӿڣ�·���滮���� */
	vector<pair<int, int>> pathplanning(Point& start, Point& end);

	

private:
	int N;	//�����ͼ��С
	int n;	//ʵ�ʵ�ͼ��С
	vector<vector<char>> maze;	//��ά�ַ���ͼ
	
};

