#pragma once
#include "MyStruct.h"
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

	/* 决策函数 */
	//生成某个机器人该帧的指令
	vector<Command> makeRobotCmd(Robot& bot, int bot_id);

	//生成某个轮船该帧的指令
	vector<Command> makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths);

};

