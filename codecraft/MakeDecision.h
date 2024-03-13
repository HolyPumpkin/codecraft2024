#pragma once
#include "MyStruct.h"
#include "PlanPath.h"
/*
 * Change Logs:
 * Date           Author       Notes
 * 2024-3-13     Haoyu Nan     the first version
 */

class MakeDecision
{
public:
	/* 构造函数 */
	MakeDecision() = default;

	explicit MakeDecision(vector<vector<char>>& _maze, int _N, int _n);	// 通过二维字符地图初始化

	/* 决策函数 */

	//指派某个机器人去拿货物
	int assignRobotGet(Robot& bot, vector<Good>& goods);

	//指派某个机器人去送货物
	int assighRobotSend(Robot& bot, vector<Berth>& berths);

	//生成某个机器人该帧的指令
	vector<Command> makeRobotCmd(Robot& bot, int bot_id);

	//生成某个轮船该帧的指令
	vector<Command> makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths);

	//生成空指令
	vector<Command> makeNullCmd(int robot_id);

private:
	int N;	//构造地图大小
	int n;	//实际地图大小
	vector<vector<char>> maze;	//二维字符地图
};

