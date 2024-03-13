#pragma once
#include "MyStruct.h"
#include "PlanPath.h"
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

	explicit MakeDecision(vector<vector<char>>& _maze, int _N, int _n);	// ͨ����ά�ַ���ͼ��ʼ��

	/* ���ߺ��� */

	//ָ��ĳ��������ȥ�û���
	int assignRobotGet(Robot& bot, vector<Good>& goods);

	//ָ��ĳ��������ȥ�ͻ���
	int assighRobotSend(Robot& bot, vector<Berth>& berths);

	//����ĳ�������˸�֡��ָ��
	vector<Command> makeRobotCmd(Robot& bot, int bot_id);

	//����ĳ���ִ���֡��ָ��
	vector<Command> makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths);

	//���ɿ�ָ��
	vector<Command> makeNullCmd(int robot_id);

private:
	int N;	//�����ͼ��С
	int n;	//ʵ�ʵ�ͼ��С
	vector<vector<char>> maze;	//��ά�ַ���ͼ
};

