#pragma once
#include "MyStruct.h"
#include "PlanPath.h"
#include "IOProcessor.h"


class Test
{
public:
	/* Ĭ�Ϲ��캯�� */
	Test() = default;

	/* ·���滮ģ����Ժ��� */
	void displayPath(vector<vector<char>> map, Point& start, Point& end, int N, int n);

};

