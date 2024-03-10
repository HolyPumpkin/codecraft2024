#pragma once
#include "MyStruct.h"


class IOProcessor
{
private:
	/* data */
public:
	IOProcessor() {};

	// 读取初始化数据
	void InitData(vector<vector<char>>& map, vector<Berth>& berths, vector<Boat>& boats);

	// 读取每一帧的输入
	int InputFrameData(int& frame_id, int& money, vector<Good>& goods, vector<Robot>& robots, vector<Boat>& boats);

	~IOProcessor() {};
};

