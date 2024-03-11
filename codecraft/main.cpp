#define _CRT_SECURE_NO_WARNINGS
#include "MyStruct.h"
#include "IOProcessor.h"


/***********************************************************************/
const int n = 200;	// 实际地图大小
const int N = 210;	// 构造地图大小

const int robot_num = 10;	// 机器人数量
const int berth_num = 10;	// 泊口数量
const int boat_num = 5;		// 轮船数量

int money, frame_id;		// 钱数，帧数

/***********************************************************************/
vector<vector<char>> map(N, vector<char>(N));
vector<Robot> robots(robot_num);
vector<Berth> berths(berth_num);
vector<Boat> boats(boat_num);

//int gds[N][N];


int main()
{
	IOProcessor iop;
	iop.ReadMapFromFile("../maps/map3.txt", map);
	iop.OutputMap(map);



	return 0;
}