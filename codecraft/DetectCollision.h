#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// ������ͻ�ĵ㼯
	vector<CollisionPoint> collision_points;

public:
	
	// ͨ����������һ��λ�ã���������ͻ�ĵ�
	// >> ��������һ��������ָ�
	// >> �����˼���
	void DetectRobotInNextStep(vector<Command>& robot_commands, vector<Robot>& robots);

	// ��������˵���ײ
	// >> ��������һ��������ָ�
	// >> �����˼��ϣ��Ա��ں������
	void ClearRobotCollision(vector<Command>& robot_commands, vector<Robot>& robots);

	// ������·������
	void RetreatRobotPath(vector<Command>& robot_commands, vector<Robot>& robots);

	// ͨ���ִ���һ��λ�ã���������ͻ�ĵ�
	// >> boat_commands, boats
	//void DetectBoatInNextStep(vector<Command>& commands, vector<Boat>& boats);
};

