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

			// 取出机器人当前坐标
			int x = robots[id].x, y = robots[id].y;

			// move <==> key == 1
			if (key == 1)
			{
				// 得到机器人move后的坐标
				int nx = x + dx[param_2], ny = y + dy[param_2];

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
		// 可能产生冲突的点

		// v 1.0
		if (data.size() > 1)
		{
			this->collision_points.push_back({ nx, ny, data });
		}

		//// v2.0
		//this->collision_points.push_back({ nx, ny, data });
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
 * @brief 处理机器人碰撞问题
 * @param robots
 * @param robot_commands
 * @return 处理后的指令集
 */
vector<vector<Command>> DetectCollision::HandleRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// 初始化各类点数据
	this->InitPointsData();

	// 计算碰撞点
	this->CalculateCollisionPoints(robots, robot_commands);

	// 需要拷贝原数据，防止修改原数据
	vector<Robot>& new_robots = robots;
	vector<vector<Command>> new_robot_commands = robot_commands;

	// 先处理间隔碰撞点
	this->HandleIntervalPoints(new_robots, new_robot_commands);

	// 再处理邻近碰撞点
	this->HandleAdjacentPoints(new_robots, new_robot_commands);


	return new_robot_commands;
}

/**
 * @brief 处理间隔碰撞点
 * @param robots
 * @param robot_commands
 */
void DetectCollision::HandleIntervalPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	for (auto& mp : this->interval_points)
	{
		// 获取产生当前间隔冲突点的机器人id集合
		vector<pair<int, int>> rbt_set = mp.second;

		// 情况处理
		if (rbt_set.size() == 2)		// 一进一退，让robot[0]后退
		{
			for (int i = 0; i < 1; i++)
			{
				int rbt_idx = rbt_set[i].first, cmd_idx = rbt_set[i].second; // 机器人下标和机器人指令下标相对应
				this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
			}
		}
		else if (rbt_set.size() == 3)	// 一进二退，让robot[0]、robot[1]后退
		{
			for (int i = 0; i < 2; i++)
			{
				int rbt_idx = rbt_set[i].first, cmd_idx = rbt_set[i].second; // 机器人下标和机器人指令下标相对应
				this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
			}
		}
		else if (rbt_set.size() == 4)	// 一进三退，让robot[0]、robot[1]、robot[2]后退
		{
			for (int i = 0; i < 3; i++)
			{
				int rbt_idx = rbt_set[i].first, cmd_idx = rbt_set[i].second; // 机器人下标和机器人指令下标相对应
				this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
			}
		}
		else
		{
			// printf("interval_points has problem!");
		}
	}
}

/**
 * @brief 处理邻近碰撞点
 * @param robots
 * @param robot_commands
 */
void DetectCollision::HandleAdjacentPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// 遍历处理邻近碰撞点
	for (auto& mp : this->adjacent_points)
	{
		// 获取产生当前邻近冲突点的两个机器人id
		pair<int, int> pii = mp.second;
		int first_id = pii.first, second_id = pii.second;

		// 获得两个机器人的指令
		// NOTE 需要考虑下标越界的隐患
		int first_cmd_idx = -1, second_cmd_idx = -1;
		for (int i = 0; i < robot_commands[first_id].size(); i++)
		{
			first_cmd_idx = i;
			if (robot_commands[first_id][i].key == 1)
			{
				break;
			}
		}
		for (int i = 0; i < robot_commands[second_id].size(); i++)
		{
			second_cmd_idx = i;
			if (robot_commands[second_id][i].key == 1)
			{
				break;
			}
		}
		// 使用引用，方便直接修改原来的robot_commands
		Command& first_cmd = robot_commands[first_id][first_cmd_idx];
		Command& second_cmd = robot_commands[second_id][second_cmd_idx];

		int first_cmd_key = first_cmd.key, first_cmd_param_2 = first_cmd.param_2;
		int second_cmd_key = second_cmd.key, second_cmd_param_2 = second_cmd.param_2;

		// 情况处理
		// move & move
			// (up | down) & (left | right)：
			// (->  ->) 同向：要保持前面一个机器人指令先执行
		if (first_cmd_key == 1 && second_cmd_key == 1)
		{
			// (left & right) | (up & down) 对冲：一个回退一个前进，回退要在前进之前执行
			if ((first_cmd_param_2 == 0 && second_cmd_param_2 == 1) ||
				(first_cmd_param_2 == 1 && second_cmd_param_2 == 0) ||
				(first_cmd_param_2 == 2 && second_cmd_param_2 == 3) ||
				(first_cmd_param_2 == 3 && second_cmd_param_2 == 2))
			{
				// 对冲情况mp会记录两次，只需修改一次就可以
				// 因此只考虑（右、左）和（上、下），不处理（左、右）和（下、上）
				// 修改后会变为（左、左）和（下、下），调整执行顺序会在下次循环交给另一方，在下面else if分支执行
				// 设置一个回退优先级（序号大的回退），防止map的自动排序问题。
				if (first_id > second_id)
				{
					this->RetreatRobotPath(robots[first_id], first_cmd);
				}
				else
				{
					this->RetreatRobotPath(robots[second_id], second_cmd);
				}
			}
			// (right | left | up | down) & others
			else if (first_cmd_param_2 == 0 || first_cmd_param_2 == 1 ||
				first_cmd_param_2 == 2 || first_cmd_param_2 == 3)
			{
				// 如果first_rbt的指令将在second_rbt之前执行，则应交换指令输出顺序
				if (first_id < second_id)
				{
					// 找到second_rbt的指令集
					int second_itr = 0;
					for (second_itr = 0; second_itr < robot_commands.size(); second_itr++)
					{
						if (robot_commands[second_itr][0].id == second_id)
						{
							break;
						}
					}
					// 将second_rbt的指令集暂存
					vector<Command> tmp_cmd_set = robot_commands[second_itr];
					// 删除second_rbt的指令集
					robot_commands.erase(robot_commands.begin() + second_itr);
					// 找到first_rbt的指令集
					int first_itr = 0;
					for (first_itr = 0; first_itr < robot_commands.size(); first_itr++)
					{
						if (robot_commands[first_itr][0].id == first_id)
						{
							break;
						}
					}
					// 将second_rbt的指令集插入到first_rbt之前
					robot_commands.insert(robot_commands.begin() + first_itr, tmp_cmd_set);
				}
			}
		}
		// move & stay
		else if (first_cmd_key == 1 && second_cmd_key != 1)
		{
			// 让move的回退，且stay的机器人指令要在move之前执行
			this->RetreatRobotPath(robots[first_id], first_cmd);
			if (first_id < second_id)
			{
				// 找到second_rbt的指令集
				int second_itr = 0;
				for (second_itr = 0; second_itr < robot_commands.size(); second_itr++)
				{
					if (robot_commands[second_itr][0].id == second_id)
					{
						break;
					}
				}
				// 将second_rbt的指令集暂存
				vector<Command> tmp_cmd_set = robot_commands[second_itr];
				// 删除second_rbt的指令集
				robot_commands.erase(robot_commands.begin() + second_itr);
				// 找到first_rbt的指令集
				int first_itr = 0;
				for (first_itr = 0; first_itr < robot_commands.size(); first_itr++)
				{
					if (robot_commands[first_itr][0].id == first_id)
					{
						break;
					}
				}
				// 将second_rbt的指令集插入到first_rbt之前
				robot_commands.insert(robot_commands.begin() + first_itr, tmp_cmd_set);
			}
		}
	}
}

/**
 * @brief 计算两类碰撞点
 * @param robots
 * @param robot_commands
 */
void DetectCollision::CalculateCollisionPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// 0: right; 1: left; 2: up; 3: down
	int dx[4] = { 0, 0, -1, 1 }, dy[4] = { 1, -1, 0, 0 };

	// 记录每个机器人当前的坐标
	// [x, y] = rbt_idx
	map<pair<int, int>, int> now_points;
	for (int rbt_idx = 0; rbt_idx < robots.size(); rbt_idx++)
	{
		// 机器人现在的坐标
		int x = robots[rbt_idx].x, y = robots[rbt_idx].y;
		now_points[{x, y}] = rbt_idx;
	}

	// 计算移动后会碰撞另一个robot的robot
	for (int rbt_idx = 0; rbt_idx < robots.size(); rbt_idx++)
	{
		// 机器人现在的坐标
		int x = robots[rbt_idx].x, y = robots[rbt_idx].y;
		// 机器人执行的指令
		vector<Command> rbt_cmd = robot_commands[rbt_idx];

		// 计算机器人执行指令后的坐标
		for (int cmd_idx = 0; cmd_idx < rbt_cmd.size(); cmd_idx++)
		{
			int key = rbt_cmd[cmd_idx].key, param_2 = rbt_cmd[cmd_idx].param_2;

			// move
			if (key == 1)
			{
				// 机器人移动后的坐标
				int nx = x + dx[param_2], ny = y + dy[param_2];

				// 如果移动后的点存在机器人，则为邻近碰撞点
				// TODO 对冲情况会加入两次
				if (now_points.count({ nx, ny }) != 0)
				{
					this->adjacent_points[{nx, ny}] = { rbt_idx, now_points[{nx, ny}] };
				}
				// 如果移动后的点不存在机器人，则考虑间隔碰撞点情况
				else
				{
					// 加入可能产生间隔碰撞点的情况
					// 后续筛选机器人个数 > 1的间隔碰撞点
					this->interval_points[{nx, ny}].push_back({ rbt_idx, cmd_idx });
				}

			}
		}
	}

	// 筛选机器人个数 > 1的点，从而得到最终间隔碰撞点
	map<pair<int, int>, vector<pair<int, int>> >::iterator iter;
	for (iter = this->interval_points.begin(); iter != this->interval_points.end();)
	{
		if ((*iter).second.size() < 2) // 擦除冲突机器人 < 2的点
		{
			this->interval_points.erase(iter++);
		}
		else
		{
			iter++;
		}
	}

}

/**
 * @brief 初始化各类点数据
 */
void DetectCollision::InitPointsData()
{
	this->interval_points.clear();
	this->adjacent_points.clear();
}

/**
 * RetreatRobotPath - 机器人路径回退。回退应该取消当前指令，然后增加反向指令。只适用于move机器人。
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
	else if (robot.is_carry == 1) // 机器人在送物
	{
		cur = robot.send_good_cur;
		path = robot.send_good_path;
	}

	// 定义机器人前一步下标
	int back_cur = cur - 1;

	if (back_cur == -1)
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
	robot_command.key = 1;
	robot_command.param_2 = param_2;
}
