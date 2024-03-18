#include "DetectCollision.h"

/**
 * DetectRobotInNextStep - ͨ����������һ��λ�ã���������ͻ�ĵ�
 * @param robots : �����˼���
 * @param robot_commands : ������ָ�����
 *
 * @return : null
 * @note : none
 */
void DetectCollision::DetectRobotInNextStep(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// 0: right; 1: left; 2: up; 3: down
	int dx[4] = { 0, 0, -1, 1 }, dy[4] = { 1, -1, 0, 0 };

	// �ù�ϣͳ��move point
	// >> <nx, ny>
	// >> [{robot_id, cmd_idx}, ...]
	map<pair<int, int>, vector<pair<int, int>>> move_points;

	// ȡ��ָ������move�����Ļ�����ID
	for (int rbt_idx = 0; rbt_idx < robots.size(); rbt_idx++)
	{
		for (int cmd_idx = 0; cmd_idx < robot_commands[rbt_idx].size(); cmd_idx++)
		{
			Command robot_cmd = robot_commands[rbt_idx][cmd_idx];
			// rbt_idx �� robot_cmd.id Ӧ������ȵ�
			int key = robot_cmd.key, id = robot_cmd.id, param_2 = robot_cmd.param_2;

			// ȡ�������˵�ǰ����
			int x = robots[id].x, y = robots[id].y;

			// move <==> key == 1
			if (key == 1)
			{
				// �õ�������move�������
				int nx = x + dx[param_2], ny = y + dy[param_2];

				// ����һ���ƶ�ͬһ��ģ������ˣ�ָ���±꣩ѹ��һ��vector
				move_points[{nx, ny}].push_back({ id, cmd_idx });
			}
		}
	}

	// ����ͳ�ƺõ�move point����ȡ��collision point
	for (auto& point : move_points)
	{
		int nx = point.first.first, ny = point.first.second;
		vector<pair<int, int>> data = point.second;
		// ���ܲ�����ͻ�ĵ�

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
 * ClearRobotCollision - ��������˵���ײ
 * @param robots : �����˼���
 * @param robot_commands : ������ָ�����
 *
 * @return : ���ͣ�-1��ʾ���δ�����ײ��0��ʾ�����Ѳ�������ײ
 * @note : none
 */
int DetectCollision::ClearRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// ��֤ÿ�μ��֮ǰ����ײ�㼯Ϊ��
	this->collision_points.clear();

	// ������ײ��⺯�����Ӷ�������ײ�㼯
	this->DetectRobotInNextStep(robots, robot_commands);

	// ���������ײ�㣬��һ��������ײ���
	if (this->collision_points.size() > 0)
	{
		// TODO ��ÿ����ͻ����д���
		// NOTE ���robots�±�� robot_commands�±��Ӧ��������Ż�
		for (int i = 0; i < this->collision_points.size(); i++)
		{
			CollisionPoint cp = this->collision_points[i];

			// ȡ����ͻ�����Ķ��������
			vector<pair<int, int>> cp_data = cp.data;

			//if (cp_data.size() == 1)
			//{
			//	// ���Է�����Ӧ����һ��Ч��
			//	// ɨ������ڳ�ͻ��rbt_idx��������л����ˣ���ֹ�Գ���������Ƿ��ڳ�ͻ����
			//	bool is_conflict = false;
			//	int rbt_idx = cp_data[0].first, cmd_idx = cp_data[0].second;

			//	for (int j = rbt_idx + 1; j < robots.size(); j++)
			//	{
			//		int tx = robots[j].x, ty = robots[j].y;
			//		if (cp.x == tx && cp.y == ty)
			//		{
			//			is_conflict = true;
			//			break;
			//		}
			//	}

			//	// is_conflict == true���л������ڳ�ͻ���ϣ������
			//	if (is_conflict)
			//	{
			//		this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
			//	}
			//	// is_conflict == false��û�У�����ͣһ֡
			//	else
			//	{
			//		/*robot_commands[rbt_idx][cmd_idx].key = -1;
			//		robot_commands[rbt_idx][cmd_idx].param_2 = -1;*/
			//	}
			//}
			//else if (cp_data.size() == 2)

			if (cp_data.size() == 2)
			{
				// һ��һ��
				// ��robot[0]����
				for (int j = 0; j < 1; j++)
				{
					int rbt_idx = cp_data[j].first, cmd_idx = cp_data[j].second;
					this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
				}
			}
			else if (cp_data.size() == 3)
			{
				// һ������
				// ��robot[0], robot[1]����
				for (int j = 0; j < 2; j++)
				{
					int rbt_idx = cp_data[j].first, cmd_idx = cp_data[j].second;
					this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
				}
			}
			else if (cp_data.size() == 4)
			{
				// һ������
				// ��robot[0], robot[1], robot[2]����
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
 * @brief �����������ײ����
 * @param robots
 * @param robot_commands
 * @return ������ָ�
 */
vector<vector<Command>> DetectCollision::HandleRobotCollision(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// ��ʼ�����������
	this->InitPointsData();

	// ��������ָ��
	vector<vector<Command>> new_robot_commands = robot_commands;

	// �ȴ�������ײ��


	// �ٴ����ڽ���ײ��



	return new_robot_commands;
}

/**
 * @brief ��������ײ��
 * @param robots
 * @param robot_commands
 */
void DetectCollision::HandleIntervalPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	for (auto& mp : this->interval_points)
	{
		// ��ȡ������ǰ�����ͻ��Ļ�����id����
		vector<int> rbt_set = mp.second;

		// �������
		if (rbt_set.size() == 2)		// һ��һ�ˣ���robot[0]����
		{
			for (int i = 0; i < 1; i++)
			{
				int rbt_idx = rbt_set[i];
				int cmd_idx = rbt_idx;	// �������±�ͻ�����ָ���±����Ӧ
				this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
			}
		}
		else if (rbt_set.size() == 3)	// һ�����ˣ���robot[0]��robot[1]����
		{
			for (int i = 0; i < 2; i++)
			{
				int rbt_idx = rbt_set[i];
				int cmd_idx = rbt_idx;	// �������±�ͻ�����ָ���±����Ӧ
				this->RetreatRobotPath(robots[rbt_idx], robot_commands[rbt_idx][cmd_idx]);
			}
		}
		else if (rbt_set.size() == 4)	// һ�����ˣ���robot[0]��robot[1]��robot[2]����
		{
			for (int i = 0; i < 3; i++)
			{
				int rbt_idx = rbt_set[i];
				int cmd_idx = rbt_idx;	// �������±�ͻ�����ָ���±����Ӧ
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
 * @brief �����ڽ���ײ��
 * @param robots
 * @param robot_commands
 */
void DetectCollision::HandleAdjacentPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	for (auto& mp : this->adjacent_points)
	{
		// ��ȡ������ǰ�ڽ���ͻ�������������id
		pair<int, int> pii = mp.second;
		int first_id = pii.first, second_id = pii.second;

		// ������������˵�ָ��
		Command first_cmd = robot_commands[first_id][0];
		Command second_cmd = robot_commands[second_id][0];

		// �������
		
		// move & move
			// (up | down) & (left | right)��
			// (->  ->) ͬ��Ҫ����ǰ��һ��������ָ����ִ��
			// (->  <-) �Գ壺һ������һ��ǰ��������Ҫ��ǰ��֮ǰִ��

		// move & stay
	}
}

/**
 * @brief ����������ײ��
 * @param robots
 * @param robot_commands
 */
void DetectCollision::CalculateCollisionPoints(vector<Robot>& robots, vector<vector<Command>>& robot_commands)
{
	// 0: right; 1: left; 2: up; 3: down
	int dx[4] = { 0, 0, -1, 1 }, dy[4] = { 1, -1, 0, 0 };

	// ��¼ÿ�������˵�ǰ������
	// [x, y] = rbt_idx
	map<pair<int, int>, int> now_points;
	for (int rbt_idx = 0; rbt_idx < robots.size(); rbt_idx++)
	{
		// ���������ڵ�����
		int x = robots[rbt_idx].x, y = robots[rbt_idx].y;
		now_points[{x, y}] = rbt_idx;
	}

	// �����ƶ������ײ��һ��robot��robot
	for (int rbt_idx = 0; rbt_idx < robots.size(); rbt_idx++)
	{
		// ���������ڵ�����
		int x = robots[rbt_idx].x, y = robots[rbt_idx].y;
		// ������ִ�е�ָ��
		vector<Command> rbt_cmd = robot_commands[rbt_idx];

		// ���������ִ��ָ��������
		for (int cmd_idx = 0; cmd_idx < rbt_cmd.size(); cmd_idx++)
		{
			int key = rbt_cmd[cmd_idx].key, param_2 = rbt_cmd[cmd_idx].param_2;

			// move
			if (key == 1)
			{
				for (int i = 0; i < 4; i++)
				{
					// �������ƶ��������
					int nx = x + dx[i], ny = y + dy[i];

					// ����ƶ���ĵ���ڻ����ˣ���Ϊ�ڽ���ײ��
					// TODO �Գ�������������
					if (now_points.count({ nx, ny }) == 1)
					{
						this->adjacent_points[{nx, ny}] = { rbt_idx, now_points[{nx, ny}] };
					}
					// ����ƶ���ĵ㲻���ڻ����ˣ����Ǽ����ײ�����
					else
					{
						// ������ܲ��������ײ������
						// ����ɸѡ�����˸��� > 1�ļ����ײ��
						this->interval_points[{nx, ny}].push_back(rbt_idx);
					}
				}
			}
		}

		// ɸѡ�����˸��� > 1�ĵ㣬�Ӷ��õ����ռ����ײ��
		map<pair<int, int>, vector<int>>::iterator iter;
		for (iter = this->interval_points.begin(); iter != this->interval_points.end();)
		{
			if (iter->second.size() < 2) // ������ͻ������ < 2�ĵ�
			{
				this->interval_points.erase(iter++);
			}
		}
	}
}

/**
 * RetreatRobotPath - ������·�����ˡ�����Ӧ��ȡ����ǰָ�Ȼ�����ӷ���ָ�
 * @param robot : һ��������
 * @param robot_command : һ��������ָ��
 *
 * @return : null
 * @note : none
 */
void DetectCollision::RetreatRobotPath(Robot& robot, Command& robot_command)
{
	// ��û����˵�·���͵�ǰ�±�
	int cur;
	vector<pair<int, int>> path;
	if (robot.is_carry == 0)	// ��������ȡ��
	{
		cur = robot.fetch_good_cur;
		path = robot.fetch_good_path;
	}
	else if (robot.is_carry == 1) // ������������
	{
		cur = robot.send_good_cur;
		path = robot.send_good_path;
	}

	// ���������ǰһ���±�
	int back_cur = cur - 1;

	if (back_cur == -1)
	{
		return;
	}

	// ������˷���
	int nx = path[cur].first, ny = path[cur].second;			// ��ǰ����
	int bx = path[back_cur].first, by = path[back_cur].second;	// ��һ������
	int dx = bx - nx, dy = by - ny;								// ����ó���������

	int param_2 = 0;	// move �ĵ�2������
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

	// �޸�ָ��Ϊ����ָ��
	robot_command.key = 1;
	robot_command.param_2 = param_2;
}
