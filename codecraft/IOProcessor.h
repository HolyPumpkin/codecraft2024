#pragma once
#include "MyStruct.h"


class IOProcessor
{
private:
	/* data */
public:
	IOProcessor() {};

	// ��ȡ��ʼ������
	void InitData(vector<vector<char>>& map, vector<Berth>& berth, vector<Boat>& boat);

	~IOProcessor() {};
};

