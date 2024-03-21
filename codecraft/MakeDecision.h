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
	/* 构造函数 */
	MakeDecision() = default;

	explicit MakeDecision(vector<vector<char>>& _maze, int _N, int _n);	// 通过二维字符地图初始化

	/* 决策函数 */

	//指派某个机器人去拿货物
	int assignRobotGet(Robot& bot, int robot_id, list<Good>& goods, int cur_frame_id);

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

	// 机器人状态迁移
	void robotStatusTrans(vector<Robot>& robots);

	// 判断是否有应该消失的货物
	void vanishGoods(list<Good>& goods, int frame_id);

	// 机器人接口函数，封装每一帧对机器人的操作，指派、生成指令等
	void robotsOperate(vector<Robot>& robots, int robot_num, vector<vector<Command>>& robot_cmd, list<Good>& goods, vector<Berth>& berths, int frame_id);

	// 轮船接口函数，封装每一帧对轮船的操作，生成指令等
	void boatsOperate(vector<Boat>& boats, vector<vector<Command>>& boat_cmd, vector<Berth>& berths, int boat_num, int frame_id);

	// 轮船最后一舞，生成去虚拟点的指令
	Command lastDance(int boat_id);

	// 选择可用泊位，目前根据机器人可达性选取
	void chooseBerths(vector<Berth>& berths, vector<Robot>& robots, int frame_id);

	// 计算每个机器人的可达区域
	void calRobotReachableMap(vector<Robot>& robots, int frame_id);

private:
	int N;	//构造地图大小
	int n;	//实际地图大小
	vector<vector<char>> maze;	//二维字符地图
	vector<Robot> own_robots;	//机器人数组，需要保证每一帧这里的都是最新的
};

