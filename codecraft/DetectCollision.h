#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// ������ͻ�ĵ㼯
	vector<CollisionPoint> collision_points;

	//========================================[ v 2.0 ]===============================================//

	// ����������ͬһ�����е����������ײ��
	map<pair<int, int>, vector<int>> interval_points;

	// ������������һ�������˶���������ײ��
	map<pair<int, int>, vector<int>> adjacent_points;

public:
	
	DetectCollision() = default;

	// ͨ����������һ��λ�ã���������ͻ�ĵ�
	void DetectRobotInNextStep(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// ��������˵���ײ
	int ClearRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	//========================================[ v 2.0 ]===============================================//

	// �����������ײ����
	vector<vector<Command>> HandleRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	// ����������ײ��
	void CalculateCollisionPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands);

	//========================================[ v 2.0 ]===============================================//

	// ������·������
	void RetreatRobotPath(Robot& robot, Command& robot_command);

	// ͨ���ִ���һ��λ�ã���������ͻ�ĵ�
	// >> boat_commands, boats
	//void DetectBoatInNextStep(vector<Command>& commands, vector<Boat>& boats);
};

