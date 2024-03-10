#pragma once
#include "MyStruct.h"


class IOProcessor
{
private:
	/* data */
public:
	IOProcessor() {};

	// ��ȡ��ʼ������
	void InitData(vector<vector<char>>& map, vector<Berth>& berths, vector<Boat>& boats);

	// ��ȡÿһ֡������
	int InputFrameData(int& frame_id, int& money, vector<Good>& goods, vector<Robot>& robots, vector<Boat>& boats);

	~IOProcessor() {};
};

