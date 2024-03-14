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
vector<Good> goods;								// 货物

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
	
	MakeDecision mkd(ch_map, N, n);
	
	DetectCollision dtc;

	// 总初始化
	iop.InitData(ch_map, berths, boats);
	for (int frame_id = 0; frame_id < 15000; ++frame_id)
	{
		iop.InputFrameData(frame_id, money, goods, robots, boats);

		/* todo：此处应该判断上一帧指令是否正常执行，根据执行结果修改结构体属性 */
		/* 依赖结构体中添加指令的vector */
		
		//机器人操作
		for (int rbt_idx = 0; rbt_idx < robot_num; ++rbt_idx)
		{
			//如果当前机器人未被指派
			if (0 == robots[rbt_idx].is_assigned)
			{
				int assign_success;
				//如果未携带
				if (0 == robots[rbt_idx].is_carry)
				{
					// 先指派机器人去拿货物
					assign_success = mkd.assignRobotGet(robots[rbt_idx], goods);
					
					// 如果指派失败，插入空指令，表示不做任何操作
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
						continue;
					}
					// 指派成功
					else if(0 == assign_success){
						robots[rbt_idx].is_assigned = 1;
					}
				}
				//如果已携带
				else if (1 == robots[rbt_idx].is_carry)
				{
					// 先指派机器人去送货物
					assign_success = mkd.assighRobotSend(robots[rbt_idx], berths);
					// 如果指派失败，插入空指令，表示不做任何操作
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
						continue;
					}
					// 指派成功
					else if (0 == assign_success) {
						robots[rbt_idx].is_assigned = 1;
					}
				}
				//如果出现错误，就不做任何操作
				else
				{
					robot_cmd[rbt_idx] = mkd.makeNullCmd(rbt_idx);
					continue;
				}
			}

			// 生成指令
			robot_cmd[rbt_idx] = mkd.makeRobotCmd(robots[rbt_idx], rbt_idx);
		}

		//todo:机器人碰撞检测（循环）
			
		//todo:轮船的操作

		//输出指令
	}





	return 0;
}