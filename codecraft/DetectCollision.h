#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// ������ͻ�ĵ㼯


public:
	
	// ͨ����������һ��λ�ã���������ͻ�ĵ�
	// >> ��������һ��������ָ�
	// >> �����˼���
	void DetectRobotInNextStep(vector<Command>& robot_commands, vector<Robot>& robots);

	// ��������˵���ײ
	void ClearRobotCollision();

	// ͨ���ִ���һ��λ�ã���������ͻ�ĵ�
	// >> boat_commands, boats
	//void DetectBoatInNextStep(vector<Command>& commands, vector<Boat>& boats);
};

