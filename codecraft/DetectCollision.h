#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// 产生冲突的点集
	vector<CollisionPoint> collision_points;

public:
	
	DetectCollision() = default;

	// 通过机器人下一步位置，检测产生冲突的点
	void DetectRobotInNextStep(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// 解除机器人的碰撞
	int ClearRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// 机器人路径回退
	void RetreatRobotPath(Robot& robot, Command& robot_command);

	// 通过轮船下一步位置，检测产生冲突的点
	// >> boat_commands, boats
	//void DetectBoatInNextStep(vector<Command>& commands, vector<Boat>& boats);
};

