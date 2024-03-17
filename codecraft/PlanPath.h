#pragma once
#include "MyStruct.h"


class PlanPath
{
public:
	/* 构造函数 */
	PlanPath() = default;
	explicit PlanPath(vector<vector<char>>& _maze, int _N, int _n, vector<Robot> robots); // 通过二维字符地图初始化

	/* 成员函数 */
	int getDis(Point& start, Point& end);

	Point* getMinF(list<Point*>& l);

	bool isInlist(list<Point*>& l, Point& p);

	Point* getPoint(list<Point*>& l, Point& p);

	void printPath(vector<pair<int, int>>& path);

	/* 对外部暴露的接口，路径规划主体 */
	vector<pair<int, int>> pathplanning(Point& start, Point& end);

	

private:
	int N;	//构造地图大小
	int n;	//实际地图大小
	vector<vector<char>> maze;	//二维字符地图
	
};

