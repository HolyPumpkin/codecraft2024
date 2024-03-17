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
	int assignRobotGet(Robot& bot, list<Good>& goods, int cur_frame_id);

	//ָ��ĳ��������ȥ�ͻ���
	int assighRobotSend(Robot& bot, vector<Berth>& berths);

	//����ĳ�������˸�֡��ָ��
	vector<Command> makeRobotCmd(Robot& bot, int bot_id);

	//����ĳ���ִ���֡��ָ��
	vector<Command> makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths, int frame_id);

	//���ɿ�ָ��
	vector<Command> makeNullCmd(int robot_id);

	//����������Ϣ�жϻ������߼�ָ���Ƿ���Ҫ�޸��Լ�·���Ƿ���Ҫ�ع滮
	void robotInputCheck(vector<Robot>& robots, list<Good>& goods, int cur_frame_id);

	//�ж��ִ���״̬
	int boatStatusCheck(Boat& boat);

	//����������Ϣ�ж��ִ��ڲ������Ƿ���Ҫ�޸�
	void boatInputCheck(vector<Boat>& boats, int frame_id);

	//�ִ�״̬Ǩ��
	void boatStatusTrans(vector<Boat>& boats);

	// ��������������
	void robotReboot(Robot& robot);

	// �жϻ������Ƿ���Ҫ�����������Ҫ������
	void rebootRobots(vector<Robot>& robots);

	//������״̬Ǩ��
	void robotStatusTrans(vector<Robot>& robots);

private:
	int N;	//�����ͼ��С
	int n;	//ʵ�ʵ�ͼ��С
	vector<vector<char>> maze;	//��ά�ַ���ͼ
	vector<Robot> robots;	//���������飬��Ҫ��֤ÿһ֡����Ķ������µ�
};

