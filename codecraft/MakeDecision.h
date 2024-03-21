#pragma once
#include "MyStruct.h"
#include "PlanPath.h"
#include "limits.h"
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
	int assignRobotGet(Robot& bot, int robot_id, list<Good>& goods, int cur_frame_id);

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

	// ������״̬Ǩ��
	void robotStatusTrans(vector<Robot>& robots);

	// �ж��Ƿ���Ӧ����ʧ�Ļ���
	void vanishGoods(list<Good>& goods, int frame_id);

	// �����˽ӿں�������װÿһ֡�Ի����˵Ĳ�����ָ�ɡ�����ָ���
	void robotsOperate(vector<Robot>& robots, int robot_num, vector<vector<Command>>& robot_cmd, list<Good>& goods, vector<Berth>& berths, int frame_id);

	// �ִ��ӿں�������װÿһ֡���ִ��Ĳ���������ָ���
	void boatsOperate(vector<Boat>& boats, vector<vector<Command>>& boat_cmd, vector<Berth>& berths, int boat_num, int frame_id);

	// �ִ����һ�裬����ȥ������ָ��
	Command lastDance(int boat_id);

	// ѡ����ò�λ��Ŀǰ���ݻ����˿ɴ���ѡȡ
	void chooseBerths(vector<Berth>& berths, vector<Robot>& robots, int frame_id);

	// ����ÿ�������˵Ŀɴ�����
	void calRobotReachableMap(vector<Robot>& robots, int frame_id);

private:
	int N;	//�����ͼ��С
	int n;	//ʵ�ʵ�ͼ��С
	vector<vector<char>> maze;	//��ά�ַ���ͼ
	vector<Robot> own_robots;	//���������飬��Ҫ��֤ÿһ֡����Ķ������µ�
};

