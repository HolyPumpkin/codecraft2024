#include "DetectCollision.h"

/**
 * DetectRobotInNextStep - 通过机器人下一步位置，检测产生冲突的点
 * @param robots : 机器人集合
 * @param robot_commands : 机器人指令集集合
 * 
 * @return : null
 * @note : none
 */
void DetectCollision::DetectRobotInNextStep(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// 0: right; 1: left; 2: up; 3: down
	int dx[4] = { 0, 0, -1, 1 }, dy[4] = { 1, -1, 0, 0 };

	// 用哈希统计move point
	// >> <nx, ny>
	// >> [{robot_id, cmd_idx}, ...]
	map<pair<int, int>, vector<pair<int, int>>> move_points;

	// 取出指令中有move操作的机器人ID
	for (int rbt_idx = 0; rbt_idx < robots.size(); rbt_idx++)
	{
		for (int cmd_idx = 0; cmd_idx < robot_commands[rbt_idx].size(); cmd_idx++)
		{
			Command robot_cmd = robot_commands[rbt_idx][cmd_idx];
			// rbt_idx 和 robot_cmd.id 应当是相等的
			int key = robot_cmd.key, id = robot_cmd.id, param_2 = robot_cmd.param_2;

			// move <==> key == 1
			if (key == 1)
			{
				// 取出机器人当前坐标
				int x = robots[id].x, y = robots[id].y;

				// 得到机器人move后的坐标
				int nx = x + dx[param_2], ny = y + dy[param_2];

				// 用哈希统计move点。
				// 将下一步移动同一点的（机器人，指令下标）压入一个vector
				move_points[{nx, ny}].push_back({ id, cmd_idx });
			}
		}
	}

	// 遍历统计好的move point，提取出collision point
	for (auto& point : move_points)
	{
		int nx = point.first.first, ny = point.first.second;
		vector<pair<int, int>> data = point.second;
		if (data.size() > 1)
		{
			this->collision_points.push_back({ nx, ny, data });
		}
	}
}

/**
 * ClearRobotCollision - 解除机器人的碰撞
 * @param robots : 机器人集合
 * @param robot_commands : 机器人指令集集合
 *
 * @return : 整型，-1表示本次存在碰撞；0表示本次已不存在碰撞
 * @note : none
 */
int DetectCollision::ClearRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// 保证每次检测之前，碰撞点集为空
	this->collision_points.clear();

	// 调用碰撞检测函数，从而生成碰撞点集
	this->DetectRobotInNextStep(robots, robot_commands);

	// 如果存在碰撞点，进一步进行碰撞解除
	if (this->collision_points.size() > 0)
	{
		// TODO 对每个冲突点进行处理
		// NOTE 如果robots下标和 robot_commands下标对应，则可以优化
		for (int i = 0; i < this->collision_points.size(); i++)
		{
			CollisionPoint cp = this->collision_points[i];

			// 取出冲突产生的对象的数据
			vector<pair<int, int>> cp_data = cp.data;

			if (cp_data.size() == 2)
			{
				// 一进一退
				// 让robot[0]后退
				for (int j = 0; j < 1; j++)
				{
					int rbt_idx = cp_data[j].first, cmd_idx = cp_data[j].second;
					this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
				}
			}
			else if (cp_data.size() == 3)
			{
				// 一进两退
				// 让robot[0], robot[1]后退
				for (int j = 0; j < 2; j++)
				{
					int rbt_idx = cp_data[j].first, cmd_idx = cp_data[j].second;
					this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
				}
			}
			else if (cp_data.size() == 4)
			{
				// 一进三退
				// 让robot[0], robot[1], robot[2]后退
				for (int j = 0; j < 3; j++)
				{
					int rbt_idx = cp_data[j].first, cmd_idx = cp_data[j].second;
					this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
				}
			}
		}
		return -1;
	}
	return 0;
}

/**
 * RetreatRobotPath - 机器人路径回退
 * @param robot : 一个机器人
 * @param robot_command : 一条机器人指令
 *
 * @return : null
 * @note : none
 */
void DetectCollision::RetreatRobotPath(Robot& robot, Command& robot_command)
{
	// 获得机器人的路径和当前下标
	int cur;
	vector<pair<int, int>> path;
	if (robot.is_carry == 0)	// 机器人在取物
	{
		cur = robot.fetch_good_cur;
		path = robot.fetch_good_path;
	}
	else if (robot.is_carry == 0) // 机器人在送物
	{
		cur = robot.send_good_cur;
		path = robot.send_good_path;
	}

	// 定义机器人前一步下标
	int back_cur = cur - 1;

	if (back_cur == -1)	// 如果回退到了原点，则让机器人原地不动
	{
		return;
	}

	// 计算回退方向
	int nx = path[cur].first, ny = path[cur].second;			// 当前坐标
	int bx = path[back_cur].first, by = path[back_cur].second;	// 上一步坐标
	int dx = bx - nx, dy = by - ny;								// 计算得出方向坐标

	int param_2 = 0;	// move 的第2个参数
	if (dx == 0)
	{
		if (dy == 1) param_2 = 0;	// right
		if (dy == -1) param_2 = 1;	// left
	}
	else if (dy == 0)
	{
		if (dx == -1) param_2 = 2;	// up
		if (dx == 1) param_2 = 3;	// down
	}

	// 修改指令为回退指令
	robot_command.param_2 = param_2;
}
