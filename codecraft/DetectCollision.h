#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// 产生冲突的点集
	vector<CollisionPoint> collision_points;

	//========================================[ v 2.0 ]===============================================//

	// 机器人移向同一个可行点而产生的碰撞点
	map<pair<int, int>, vector<pair<int, int>> > interval_points;

	// 机器人移向另一个机器人而产生的碰撞点
	// [x, y] = [move_rbt_id, second_rbt_id]
	map<pair<int, int>, pair<int, int>> adjacent_points;

public:
	
	DetectCollision() = default;

	// 通过机器人下一步位置，检测产生冲突的点
	void DetectRobotInNextStep(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// 解除机器人的碰撞
	int ClearRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	//========================================[ v 2.0 ]===============================================//

	// 处理机器人碰撞问题
	vector<vector<Command>> HandleRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// 处理间隔碰撞点
	void HandleIntervalPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// 处理邻近碰撞点
	void HandleAdjacentPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// 计算两类碰撞点
	void CalculateCollisionPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// 初始化各类点数据
	void InitPointsData();

	//========================================[ v 2.0 ]===============================================//

	// 机器人路径回退
	void RetreatRobotPath(Robot& robot, Command& robot_command);

	// 通过轮船下一步位置，检测产生冲突的点
	// >> boat_commands, boats
	//void DetectBoatInNextStep(vector<Command>& commands, vector<Boat>& boats);
};

