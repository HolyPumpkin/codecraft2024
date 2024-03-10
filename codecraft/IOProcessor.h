#pragma once
#include "MyStruct.h"


class IOProcessor
{
private:
	/* data */
public:
	IOProcessor() {};

	// 读取初始化数据
	void InitData(vector<vector<char>>& map, vector<Berth>& berth, vector<Boat>& boat);

	~IOProcessor() {};
};

