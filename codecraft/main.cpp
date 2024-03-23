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
	// 总初始化
	IOProcessor iop(n, berth_num, boat_num, robot_num);

	iop.InitData(ch_map, berths, boats, robots);

	MakeDecision mkd(ch_map, N, n);
	
	DetectCollision dtc;
	for (int zhen = 0; zhen < 15000; ++zhen)
	{
		// 轮船状态迁移，保证当前帧和前一帧的状态准确
		mkd.boatStatusTrans(boats);

		// 机器人状态迁移，保证当前帧和前一帧的状态准确
		mkd.robotStatusTrans(robots);

		// 读取每一帧信息
		iop.InputFrameData(frame_id, money, goods, robots, boats);
		
		/* 只在第一帧进行的初始化操作 */
		mkd.chooseBerths(berths, robots, frame_id);	//泊位选择

		mkd.calRobotReachableMap(robots, frame_id);	//机器人可达区域计算

		mkd.calBerthReachableMap(berths, frame_id);	//泊位可达区域计算
		/* 只在第一帧进行的初始化操作 */

		// 每一帧开始判断有无应该消失的货物
		mkd.vanishGoods(goods, frame_id);

		// 每一帧开始时检查机器人状态
		mkd.robotInputCheck(robots, goods, frame_id);

		// 每一帧开始时检查轮船状态
		mkd.boatInputCheck(boats, frame_id);

		// 每一帧来对机器人重启（如果有必要）
		mkd.rebootRobots(robots);

		// 机器人操作，指令存在robot_cmd
		mkd.robotsOperate(robots, robot_num, robot_cmd, goods, berths, frame_id);

		//机器人碰撞检测
		vector<vector<Command>> modify_robot_cmd = dtc.HandleRobotCollision(robots, robot_cmd);

		//轮船操作，指令存在boat_cmd
		mkd.boatsOperate(boats, boat_cmd, berths, boat_num, frame_id);

		//输出指令
		iop.OutputCommand(modify_robot_cmd, boat_cmd);
		
	}
	return 0;
}