#define _CRT_SECURE_NO_WARNINGS
#include "MyStruct.h"
#include "IOProcessor.h"
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
	IOProcessor iop;
	iop.ReadMapFromFile("../maps/map3.txt", ch_map);
	iop.OutputMap(ch_map);

	Test test;
	Point s = Point(30, 122);
	Point e = Point(66, 56);
	test.displayPath(ch_map, s, e, N, n);


	return 0;
}