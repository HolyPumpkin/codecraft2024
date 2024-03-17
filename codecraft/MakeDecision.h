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
	/* 构造函数 */
	MakeDecision() = default;

	explicit MakeDecision(vector<vector<char>>& _maze, int _N, int _n);	// 通过二维字符地图初始化

	/* 决策函数 */

	//指派某个机器人去拿货物
	int assignRobotGet(Robot& bot, list<Good>& goods, int cur_frame_id);

	//指派某个机器人去送货物
	int assighRobotSend(Robot& bot, vector<Berth>& berths);

	//生成某个机器人该帧的指令
	vector<Command> makeRobotCmd(Robot& bot, int bot_id);

	//生成某个轮船该帧的指令
	vector<Command> makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths, int frame_id);

	//生成空指令
	vector<Command> makeNullCmd(int robot_id);

	//根据已有信息判断机器人逻辑指针是否需要修改以及路径是否需要重规划
	void robotInputCheck(vector<Robot>& robots, list<Good>& goods, int cur_frame_id);

	//判断轮船的状态
	int boatStatusCheck(Boat& boat);

	//根据已有信息判断轮船内部变量是否需要修改
	void boatInputCheck(vector<Boat>& boats, int frame_id);

	//轮船状态迁移
	void boatStatusTrans(vector<Boat>& boats);

	// 单个机器人重启
	void robotReboot(Robot& robot);

	// 判断机器人是否需要重启，如果需要则重启
	void rebootRobots(vector<Robot>& robots);

	//机器人状态迁移
	void robotStatusTrans(vector<Robot>& robots);

private:
	int N;	//构造地图大小
	int n;	//实际地图大小
	vector<vector<char>> maze;	//二维字符地图
	vector<Robot> robots;	//机器人数组，需要保证每一帧这里的都是最新的
};

