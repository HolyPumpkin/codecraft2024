#pragma once
#include "MyStruct.h"

class IOProcessor
{
private:

	// 地图大小，泊口数量，轮船数量，机器人数量
	int map_size, berth_size, boat_size, robot_size;

	// 指令的字符串映射
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

	// 从文件读取地图
	int ReadMapFromFile(string file_path, vector<vector<char>>& map);

	// 输出地图到控制台
	void OutputMap(vector<vector<char>>& map);

	// 读取初始化数据
	void InitData(vector<vector<char>>& map, vector<Berth>& berths, vector<Boat>& boats);

	// 读取每一帧的输入
	int InputFrameData(int& frame_id, int& money, vector<Good>& goods, vector<Robot>& robots, vector<Boat>& boats);

	// 向判题器输出每一帧的指令
	void OutputCommand(vector<vector<Command>>& robot_cmd, vector<vector<Command>>& boat_cmd);

	~IOProcessor() {};
};

