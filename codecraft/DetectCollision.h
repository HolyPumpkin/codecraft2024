#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// ������ͻ�ĵ㼯
	vector<CollisionPoint> collision_points;

public:
	
	DetectCollision() = default;

	// ͨ����������һ��λ�ã���������ͻ�ĵ�
	void DetectRobotInNextStep(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// ��������˵���ײ
	int ClearRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// ������·������
	void RetreatRobotPath(Robot& robot, Command& robot_command);

	// ͨ���ִ���һ��λ�ã���������ͻ�ĵ�
	// >> boat_commands, boats
	//void DetectBoatInNextStep(vector<Command>& commands, vector<Boat>& boats);
};

