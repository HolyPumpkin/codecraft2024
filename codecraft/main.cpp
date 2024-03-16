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
	/*IOProcessor iop;
	iop.ReadMapFromFile("../maps/map3.txt", ch_map);
	iop.OutputMap(ch_map);*/

	/*Test test;
	Point s = Point(30, 122);
	Point e = Point(66, 56);
	test.displayPath(ch_map, s, e, N, n);*/

	IOProcessor iop(n, berth_num, boat_num, robot_num);

	iop.InitData(ch_map, berths, boats);

	MakeDecision mkd(ch_map, N, n);
	
	DetectCollision dtc;

	// �ܳ�ʼ��
	
	for (int zhen = 0; zhen < 15000; ++zhen)
	{
		mkd.boatStatusTrans(boats);

		iop.InputFrameData(frame_id, money, goods, robots, boats);

		/* todo���˴�Ӧ���ж���һָ֡���Ƿ�����ִ�У�����ִ�н���޸Ľṹ������ */
		/* �����ṹ�������ָ���vector */

		// ÿһ֡��ʼ�ж�����Ӧ����ʧ�Ļ���
		for (auto it = goods.begin(); it != goods.end();)
		{
			if (frame_id >= it->end_frame)
			{
				it = goods.erase(it);
			}
			else
			{
				++it;
			}
		}

		// ÿһ֡��ʼʱ��������״̬
		mkd.robotInputCheck(robots, goods, frame_id);

		// ÿһ֡��ʼʱ����ִ�״̬
		mkd.boatInputCheck(boats, frame_id);

		// �����˲���
		for (int rbt_idx = 0; rbt_idx < robot_num; ++rbt_idx)
		{
			/* �ع����� */
			// is_carry�����ı䣬��ʾ�õ����߷����˻����ʱҪ�޸�is_assigned��ֵΪ0����ʾҪ����ָ��
			if (robots[rbt_idx].is_carry == robots[rbt_idx].last_is_carry)
			{
				// ���is_carry��1���0����ʾ�ڲ�λ�����˻��Ҫ���Ķ�Ӧ��λ����Ϣ
				if (0 == robots[rbt_idx].is_carry)
				{
					berths[robots[rbt_idx].berth_id].cur_goods_num++;
					berths[robots[rbt_idx].berth_id].cur_goods_val += robots[rbt_idx].robot_val;
				}

				robots[rbt_idx].is_assigned = 0;
				// ��֤����������is_carryʼ�ղ�ͬ
				robots[rbt_idx].last_is_carry = robots[rbt_idx].last_is_carry ^ 1;
			}

			// �����ǰ������δЯ������
			if (0 == robots[rbt_idx].is_carry)
			{
				// �����ǰ�������Ѿ���ָ��
				if (1 == robots[rbt_idx].is_assigned)
				{
					// �жϻ���Ĵ��ʱ��,�����ʱ�����Ѿ���ʧ
					if (frame_id >= robots[rbt_idx].good_end_frame)
					{
						robots[rbt_idx].is_assigned = 0;
						// ��ָ�ɻ�����ȥ�û���
						int assign_success = mkd.assignRobotGet(robots[rbt_idx], goods, frame_id);

						// ���ָ��ʧ�ܣ������ָ���ʾ�����κβ���
						if (-1 == assign_success)
						{
							robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
							continue;
						}
						// ָ�ɳɹ�
						else if (0 == assign_success) {
							robots[rbt_idx].is_assigned = 1;	// �޸��ڲ�����
						}
					}
					// ֱ������ָ��
					robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
				}
				// �����ǰ������δ��ָ��
				else if (0 == robots[rbt_idx].is_assigned)
				{
					// ��ָ�ɻ�����ȥ�û���
					int assign_success = mkd.assignRobotGet(robots[rbt_idx], goods, frame_id);

					// ���ָ��ʧ�ܣ������ָ���ʾ�����κβ���
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
						continue;
					}
					// ָ�ɳɹ�
					else if (0 == assign_success) {
						robots[rbt_idx].is_assigned = 1;	// �޸��ڲ�����
					}
					// ����ָ��
					robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
				}
			}
			// �����ǰ������Я������
			else if (1 == robots[rbt_idx].is_carry)
			{
				// �����ǰ�������Ѿ���ָ��
				if (1 == robots[rbt_idx].is_assigned)
				{
					// ֱ������ָ��
					robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
				}
				// �����ǰ������δ��ָ��
				else if (0 == robots[rbt_idx].is_assigned)
				{
					// ��ָ�ɻ�����ȥ�û���
					int assign_success = mkd.assighRobotSend(robots[rbt_idx], berths);

					// ���ָ��ʧ�ܣ������ָ���ʾ�����κβ���
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
						continue;
					}
					// ָ�ɳɹ�
					else if (0 == assign_success) {
						robots[rbt_idx].is_assigned = 1;	// �޸��ڲ�����
					}
					// ����ָ��
					robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
				}
			}
		}

			/* �ع����� */

		//��������ײ��⣨ѭ����
		//while (-1 == dtc.ClearRobotCollision(robots, robot_cmd))
		//{
		//	;	//ֱ������û����ײ
		//}

		dtc.ClearRobotCollision(robots, robot_cmd);

		//�ִ��Ĳ���
		for (int boat_idx = 0; boat_idx < boat_num; ++boat_idx)
		{
			boat_cmd[boat_idx] = mkd.makeBoatCmd(boats[boat_idx], boat_idx, berths, frame_id);
		}

		//���ָ��
		iop.OutputCommand(robot_cmd, boat_cmd);
	}





	return 0;
}