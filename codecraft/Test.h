#pragma once
#include "MyStruct.h"
#include "PlanPath.h"
#include "IOProcessor.h"


class Test
{
public:
	/* 默认构造函数 */
	Test() = default;

	/* 路径规划模块测试函数 */
	void displayPath(vector<vector<char>> map, Point& start, Point& end, int N, int n);

};

