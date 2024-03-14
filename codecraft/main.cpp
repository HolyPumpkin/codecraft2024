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
vector<Good> goods;								// ����

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
	/*IOProcessor iop;
	iop.ReadMapFromFile("../maps/map3.txt", ch_map);
	iop.OutputMap(ch_map);*/

	/*Test test;
	Point s = Point(30, 122);
	Point e = Point(66, 56);
	test.displayPath(ch_map, s, e, N, n);*/

	IOProcessor iop(n, berth_num, boat_num, robot_num);
	
	MakeDecision mkd(ch_map, N, n);
	
	DetectCollision dtc;

	// �ܳ�ʼ��
	iop.InitData(ch_map, berths, boats);
	for (int frame_id = 0; frame_id < 15000; ++frame_id)
	{
		iop.InputFrameData(frame_id, money, goods, robots, boats);

		/* todo���˴�Ӧ���ж���һָ֡���Ƿ�����ִ�У�����ִ�н���޸Ľṹ������ */
		/* �����ṹ�������ָ���vector */
		
		//�����˲���
		for (int rbt_idx = 0; rbt_idx < robot_num; ++rbt_idx)
		{
			//�����ǰ������δ��ָ��
			if (0 == robots[rbt_idx].is_assigned)
			{
				int assign_success;
				//���δЯ��
				if (0 == robots[rbt_idx].is_carry)
				{
					// ��ָ�ɻ�����ȥ�û���
					assign_success = mkd.assignRobotGet(robots[rbt_idx], goods);
					
					// ���ָ��ʧ�ܣ������ָ���ʾ�����κβ���
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
						continue;
					}
					// ָ�ɳɹ�
					else if(0 == assign_success){
						robots[rbt_idx].is_assigned = 1;
					}
				}
				//�����Я��
				else if (1 == robots[rbt_idx].is_carry)
				{
					// ��ָ�ɻ�����ȥ�ͻ���
					assign_success = mkd.assighRobotSend(robots[rbt_idx], berths);
					// ���ָ��ʧ�ܣ������ָ���ʾ�����κβ���
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
						continue;
					}
					// ָ�ɳɹ�
					else if (0 == assign_success) {
						robots[rbt_idx].is_assigned = 1;
					}
				}
				//������ִ��󣬾Ͳ����κβ���
				else
				{
					robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
					continue;
				}
			}

			// ����ָ��
			robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
		}

		//��������ײ��⣨ѭ����
		while (-1 == dtc.ClearRobotCollision(robots, robot_cmd))
		{
			;	//ֱ������û����ײ
		}

		//�ִ��Ĳ���
		for (int boat_idx = 0; boat_idx < boat_num; ++boat_idx)
		{
			boat_cmd[boat_idx] = mkd.makeBoatCmd(boats[boat_idx], boat_idx, berths);
		}

		//���ָ��
		iop.OutputCommand(robot_cmd, boat_cmd);
	}





	return 0;
}