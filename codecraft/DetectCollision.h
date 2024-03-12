#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// 产生冲突的点集
	vector<CollisionPoint> collision_points;

public:
	
	// 通过机器人下一步位置，检测产生冲突的点
	// >> 机器人下一步操作的指令集
	// >> 机器人集合
	void DetectRobotInNextStep(vector<Command>& robot_commands, vector<Robot>& robots);

	// 解除机器人的碰撞
	// >> 机器人下一步操作的指令集
	// >> 机器人集合，以便于后面决策
	void ClearRobotCollision(vector<Command>& robot_commands, vector<Robot>& robots);

	// 机器人路径回退
	void RetreatRobotPath(Robot& robot, Command& robot_command);

	// 通过轮船下一步位置，检测产生冲突的点
	// >> boat_commands, boats
	//void DetectBoatInNextStep(vector<Command>& commands, vector<Boat>& boats);
};

