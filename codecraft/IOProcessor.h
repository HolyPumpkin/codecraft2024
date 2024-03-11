#pragma once
#include "MyStruct.h"


class IOProcessor
{
private:

	// ��ͼ��С�������������ִ�����������������
	int map_size, berth_size, boat_size, robot_size;

public:

	IOProcessor() :map_size(20), berth_size(10), boat_size(5), robot_size(10) {};

	IOProcessor(int map_size, int berth_size, int boat_size, int robot_size);

	// ���ļ���ȡ��ͼ
	int ReadMapFromFile(vector<vector<char>>& map);

	// ��ȡ��ʼ������
	void InitData(vector<vector<char>>& map, vector<Berth>& berths, vector<Boat>& boats);

	// ��ȡÿһ֡������
	int InputFrameData(int& frame_id, int& money, vector<Good>& goods, vector<Robot>& robots, vector<Boat>& boats);

	~IOProcessor() {};
};

