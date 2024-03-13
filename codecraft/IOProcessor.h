#pragma once
#include "MyStruct.h"

class IOProcessor
{
private:

	// ��ͼ��С�������������ִ�����������������
	int map_size, berth_size, boat_size, robot_size;

	// ָ����ַ���ӳ��
	map<int, string> key_to_name =
	{
		{1, "move"},
		{2, "get"},
		{4, "pull"},
		{8, "ship"},
		{16, "go"}
	};

public:

	IOProcessor() :map_size(200), berth_size(10), boat_size(5), robot_size(10) {};

	IOProcessor(int map_size, int berth_size, int boat_size, int robot_size);

	// ���ļ���ȡ��ͼ
	int ReadMapFromFile(string file_path, vector<vector<char>>& map);

	// �����ͼ������̨
	void OutputMap(vector<vector<char>>& map);

	// ��ȡ��ʼ������
	void InitData(vector<vector<char>>& map, vector<Berth>& berths, vector<Boat>& boats);

	// ��ȡÿһ֡������
	int InputFrameData(int& frame_id, int& money, vector<Good>& goods, vector<Robot>& robots, vector<Boat>& boats);

	// �����������ÿһ֡��ָ��
	void OutputCommand(vector<vector<Command>>& robot_cmd, vector<vector<Command>>& boat_cmd);

	~IOProcessor() {};
};

