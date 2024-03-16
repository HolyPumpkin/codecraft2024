#define _CRT_SECURE_NO_WARNINGS
#include "MyStruct.h"
#include "IOProcessor.h"
#include "MakeDecision.h"
#include "DetectCollision.h"
#include "Test.h"


/***********************************************************************/
const int n = 200;			// 实际地图大小
const int N = 210;			// 构造地图大小

const int robot_num = 10;	// 机器人数量
const int berth_num = 10;	// 泊口数量
const int boat_num = 5;		// 轮船数量

int money, frame_id;		// 钱数，帧数

/***********************************************************************/
vector<vector<char>> ch_map(N, vector<char>(N));	// 地图
vector<Robot> robots(robot_num);				// 机器人
vector<Berth> berths(berth_num);				// 泊口
vector<Boat> boats(boat_num);					// 轮船
list<Good> goods;								// 货物

// 机器人一帧的指令集、轮船一帧的指令集
// 第i行：第i个机器人的指令集
// 
// 存储格式可理解为邻接表，如下：
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

	// 总初始化
	
	for (int zhen = 0; zhen < 15000; ++zhen)
	{
		mkd.boatStatusTrans(boats);

		iop.InputFrameData(frame_id, money, goods, robots, boats);

		/* todo：此处应该判断上一帧指令是否正常执行，根据执行结果修改结构体属性 */
		/* 依赖结构体中添加指令的vector */

		// 每一帧开始判断有无应该消失的货物
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

		// 每一帧开始时检查机器人状态
		mkd.robotInputCheck(robots, goods, frame_id);

		// 每一帧开始时检查轮船状态
		mkd.boatInputCheck(boats, frame_id);

		// 机器人操作
		for (int rbt_idx = 0; rbt_idx < robot_num; ++rbt_idx)
		{
			/* 重构部分 */
			// is_carry发生改变，表示拿到或者放下了货物，此时要修改is_assigned的值为0，表示要重新指派
			if (robots[rbt_idx].is_carry == robots[rbt_idx].last_is_carry)
			{
				// 如果is_carry从1变成0，表示在泊位放下了货物，要更改对应泊位的信息
				if (0 == robots[rbt_idx].is_carry)
				{
					berths[robots[rbt_idx].berth_id].cur_goods_num++;
					berths[robots[rbt_idx].berth_id].cur_goods_val += robots[rbt_idx].robot_val;
				}

				robots[rbt_idx].is_assigned = 0;
				// 保证孪生变量与is_carry始终不同
				robots[rbt_idx].last_is_carry = robots[rbt_idx].last_is_carry ^ 1;
			}

			// 如果当前机器人未携带东西
			if (0 == robots[rbt_idx].is_carry)
			{
				// 如果当前机器人已经被指派
				if (1 == robots[rbt_idx].is_assigned)
				{
					// 判断货物的存活时间,如果此时货物已经消失
					if (frame_id >= robots[rbt_idx].good_end_frame)
					{
						robots[rbt_idx].is_assigned = 0;
						// 先指派机器人去拿货物
						int assign_success = mkd.assignRobotGet(robots[rbt_idx], goods, frame_id);

						// 如果指派失败，插入空指令，表示不做任何操作
						if (-1 == assign_success)
						{
							robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
							continue;
						}
						// 指派成功
						else if (0 == assign_success) {
							robots[rbt_idx].is_assigned = 1;	// 修改内部变量
						}
					}
					// 直接生成指令
					robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
				}
				// 如果当前机器人未被指派
				else if (0 == robots[rbt_idx].is_assigned)
				{
					// 先指派机器人去拿货物
					int assign_success = mkd.assignRobotGet(robots[rbt_idx], goods, frame_id);

					// 如果指派失败，插入空指令，表示不做任何操作
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
						continue;
					}
					// 指派成功
					else if (0 == assign_success) {
						robots[rbt_idx].is_assigned = 1;	// 修改内部变量
					}
					// 生成指令
					robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
				}
			}
			// 如果当前机器人携带东西
			else if (1 == robots[rbt_idx].is_carry)
			{
				// 如果当前机器人已经被指派
				if (1 == robots[rbt_idx].is_assigned)
				{
					// 直接生成指令
					robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
				}
				// 如果当前机器人未被指派
				else if (0 == robots[rbt_idx].is_assigned)
				{
					// 先指派机器人去拿货物
					int assign_success = mkd.assighRobotSend(robots[rbt_idx], berths);

					// 如果指派失败，插入空指令，表示不做任何操作
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
						continue;
					}
					// 指派成功
					else if (0 == assign_success) {
						robots[rbt_idx].is_assigned = 1;	// 修改内部变量
					}
					// 生成指令
					robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
				}
			}
		}

			/* 重构部分 */

		//机器人碰撞检测（循环）
		//while (-1 == dtc.ClearRobotCollision(robots, robot_cmd))
		//{
		//	;	//直到本次没有碰撞
		//}

		dtc.ClearRobotCollision(robots, robot_cmd);

		//轮船的操作
		for (int boat_idx = 0; boat_idx < boat_num; ++boat_idx)
		{
			boat_cmd[boat_idx] = mkd.makeBoatCmd(boats[boat_idx], boat_idx, berths, frame_id);
		}

		//输出指令
		iop.OutputCommand(robot_cmd, boat_cmd);
	}





	return 0;
}