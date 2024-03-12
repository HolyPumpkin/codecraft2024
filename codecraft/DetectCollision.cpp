#include "DetectCollision.h"

void DetectCollision::DetectRobotInNextStep(vector<Command>& robot_commands, vector<Robot>& robots)
{
	// 0: right; 1: left; 2: up; 3: down
	int dx[4] = { 0, 0, -1, 1 }, dy[4] = { 1, -1, 0, 0 };

	// 用哈希统计move point
	// >> <nx, ny>
	// >> [{robot_id, cmd_idx}, ...]
	map<pair<int, int>, vector<pair<int, int>>> move_points;

	// 取出指令中有move操作的机器人ID
	// vector<pair<int, int>> move_robots;
	for (int cmd_idx = 0; cmd_idx < robot_commands.size(); cmd_idx++)
	{
		Command robot_cmd = robot_commands[cmd_idx];
		int key = robot_cmd.key, id = robot_cmd.id, param_2 = robot_cmd.param_2;

		// 'move', key == 1
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

	// 遍历统计好的move point，提取出collision point
	for (auto point : move_points)
	{
		int nx = point.first.first, ny = point.first.second;
		vector<pair<int, int>> data = point.second;
		if (data.size() > 1)
		{
			this->collision_points.push_back({nx, ny, data});
		}
	}
}

void DetectCollision::ClearRobotCollision(vector<Command>& robot_commands, vector<Robot>& robots)
{
	// 调用碰撞检测函数，从而生成碰撞点集
	this->DetectRobotInNextStep(robot_commands, robots);

	// 如果存在碰撞点，进一步进行碰撞解除
	if (this->collision_points.size() > 0)
	{
		// TODO 对每个冲突点进行处理
		for (int i = 0; i < this->collision_points.size(); i++)
		{
			CollisionPoint cp = this->collision_points[i];

			// 取出冲突产生的对象的数据
			vector<pair<int, int>> cp_data = cp.data;

			if (cp_data.size() == 2)
			{
				// 一进一退
				// 让robot[0]后退
				
			}
			else if (cp_data.size() == 3)
			{
				// 一进两退
				// 让robot[0], robot[1]后退
			}
			else if (cp_data.size() == 4)
			{
				// 一进三退
				// 让robot[0], robot[1], robot[2]后退
				for (int j = 0; j < 3; j++)
				{
					int robot_id = cp_data[j].first, cmd_idx = cp_data[j].second;
					int dir = robot_commands[cmd_idx].param_2;
					//robot_commands[cmd_idx].param_2 = (dir + 2) % 4;
				}
			}
		}
	}
}
