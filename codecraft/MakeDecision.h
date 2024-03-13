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
	/* ���캯�� */
	MakeDecision() = default;

	/* ���ߺ��� */
	//����ĳ�������˸�֡��ָ��
	vector<Command> makeRobotCmd(Robot& bot, int bot_id);

	//����ĳ���ִ���֡��ָ��
	vector<Command> makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths);

};

