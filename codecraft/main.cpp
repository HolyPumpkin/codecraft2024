#define _CRT_SECURE_NO_WARNINGS
#include "MyStruct.h"
#include "IOProcessor.h"
#include "MakeDecision.h"
#include "DetectCollision.h"
#include "Test.h"


/***********************************************************************/
const int n = 200;			// ʵ�ʵ�ͼ��С
const int N = 210;			// �����ͼ��С

const int robot_num = 10;	// ����������
const int berth_num = 10;	// ��������
const int boat_num = 5;		// �ִ�����

int money, frame_id;		// Ǯ����֡��

/***********************************************************************/
vector<vector<char>> ch_map(N, vector<char>(N));	// ��ͼ
vector<Robot> robots(robot_num);				// ������
vector<Berth> berths(berth_num);				// ����
vector<Boat> boats(boat_num);					// �ִ�
list<Good> goods;								// ����

// ������һ֡��ָ����ִ�һ֡��ָ�
// ��i�У���i�������˵�ָ�
// 
// �洢��ʽ�����Ϊ�ڽӱ����£�
// [robot1]: [rbt_cmd1, rbt_cmd2, ...]
// [robot2]: [rbt_cmd1, rbt_cmd2, ...]
// ...
vector<vector<Command>> robot_cmd(robot_num);
vector<vector<Command>> boat_cmd(boat_num);

//int gds[N][N];

/***********************************************************************/
int main()
{
	// �ܳ�ʼ��
	IOProcessor iop(n, berth_num, boat_num, robot_num);

	iop.InitData(ch_map, berths, boats, robots);

	MakeDecision mkd(ch_map, N, n);
	
	DetectCollision dtc;
	for (int zhen = 0; zhen < 15000; ++zhen)
	{
		// �ִ�״̬Ǩ�ƣ���֤��ǰ֡��ǰһ֡��״̬׼ȷ
		mkd.boatStatusTrans(boats);

		// ������״̬Ǩ�ƣ���֤��ǰ֡��ǰһ֡��״̬׼ȷ
		mkd.robotStatusTrans(robots);

		// ��ȡÿһ֡��Ϣ
		iop.InputFrameData(frame_id, money, goods, robots, boats);
		
		/* ֻ�ڵ�һ֡���еĳ�ʼ������ */
		mkd.chooseBerths(berths, robots, frame_id);	//��λѡ��

		mkd.calRobotReachableMap(robots, frame_id);	//�����˿ɴ��������

		mkd.calBerthReachableMap(berths, frame_id);	//��λ�ɴ��������
		/* ֻ�ڵ�һ֡���еĳ�ʼ������ */

		// ÿһ֡��ʼ�ж�����Ӧ����ʧ�Ļ���
		mkd.vanishGoods(goods, frame_id);

		// ÿһ֡��ʼʱ��������״̬
		mkd.robotInputCheck(robots, goods, frame_id);

		// ÿһ֡��ʼʱ����ִ�״̬
		mkd.boatInputCheck(boats, frame_id);

		// ÿһ֡���Ի���������������б�Ҫ��
		mkd.rebootRobots(robots);

		// �����˲�����ָ�����robot_cmd
		mkd.robotsOperate(robots, robot_num, robot_cmd, goods, berths, frame_id);

		//��������ײ���
		vector<vector<Command>> modify_robot_cmd = dtc.HandleRobotCollision(robots, robot_cmd);

		//�ִ�������ָ�����boat_cmd
		mkd.boatsOperate(boats, boat_cmd, berths, boat_num, frame_id);

		//���ָ��
		iop.OutputCommand(modify_robot_cmd, boat_cmd);
		
	}
	return 0;
}