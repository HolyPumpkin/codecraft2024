#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// 产生冲突的点集


public:
	
	// 通过机器人下一步位置，检测产生冲突的点
	// >> 机器人下一步操作的指令集
	// >> 机器人集合
	void DetectRobotInNextStep(vector<Command>& robot_commands, vector<Robot>& robots);

	// 解除机器人的碰撞
	void ClearRobotCollision();

	// 通过轮船下一步位置，检测产生冲突的点
	// >> boat_commands, boats
	//void DetectBoatInNextStep(vector<Command>& commands, vector<Boat>& boats);
};

