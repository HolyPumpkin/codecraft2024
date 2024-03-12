#include "DetectCollision.h"

void DetectCollision::DetectRobotInNextStep(vector<Command>& robot_commands, vector<Robot>& robots)
{
	// 0: right; 1: left; 2: up; 3: down
	int dx[4] = {0, 0, -1, 1}, dy[4] = {1, -1, 0, 0};

	// 统计冲突点
	

	// 取出指令中有move操作的机器人ID
	vector<pair<int, int>> move_robots;
	for (auto& robot_cmd : robot_commands)
	{
		// move的key = 1
		if (robot_cmd.key == 1)
		{
			// 取出机器人当前坐标
			int id = robot_cmd.id, param_2 = robot_cmd.param_2;
			int x = robots[id].x, y = robots[id].y;

			// 得到机器人move后的坐标
			int nx = x + dx[param_2], ny = y + dy[param_2];

			// 用哈希统计冲突点

		}
	}
}
