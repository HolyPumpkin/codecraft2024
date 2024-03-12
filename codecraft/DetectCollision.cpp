#include "DetectCollision.h"

void DetectCollision::DetectRobotInNextStep(vector<Command>& robot_commands, vector<Robot>& robots)
{
	// 0: right; 1: left; 2: up; 3: down
	int dx[4] = { 0, 0, -1, 1 }, dy[4] = { 1, -1, 0, 0 };

	// �ù�ϣͳ��move point
	// >> <nx, ny>
	// >> [{robot_id, cmd_idx}, ...]
	map<pair<int, int>, vector<pair<int, int>>> move_points;

	// ȡ��ָ������move�����Ļ�����ID
	// vector<pair<int, int>> move_robots;
	for (int cmd_idx = 0; cmd_idx < robot_commands.size(); cmd_idx++)
	{
		Command robot_cmd = robot_commands[cmd_idx];
		int key = robot_cmd.key, id = robot_cmd.id, param_2 = robot_cmd.param_2;

		// 'move', key == 1
		if (key == 1)
		{
			// ȡ�������˵�ǰ����
			int x = robots[id].x, y = robots[id].y;

			// �õ�������move�������
			int nx = x + dx[param_2], ny = y + dy[param_2];

			// �ù�ϣͳ��move�㡣
			// ����һ���ƶ�ͬһ��ģ������ˣ�ָ���±꣩ѹ��һ��vector
			move_points[{nx, ny}].push_back({ id, cmd_idx });
		}
	}

	// ����ͳ�ƺõ�move point����ȡ��collision point
	for (auto &point : move_points)
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
	//��֤ÿ�μ��֮ǰ����ײ�㼯Ϊ��
	this->collision_points.clear();
	// ������ײ��⺯�����Ӷ�������ײ�㼯
	this->DetectRobotInNextStep(robot_commands, robots);

	// ���������ײ�㣬��һ��������ײ���
	if (this->collision_points.size() > 0)
	{
		// TODO ��ÿ����ͻ����д���
		// NOTE ���robots�±��robot_commands�±��Ӧ��������Ż�
		for (int i = 0; i < this->collision_points.size(); i++)
		{
			CollisionPoint cp = this->collision_points[i];

			// ȡ����ͻ�����Ķ��������
			vector<pair<int, int>> cp_data = cp.data;

			if (cp_data.size() == 2)
			{
				// һ��һ��
				// ��robot[0]����
				for (int j = 0; j < 1; j++)
				{
					int robot_id = cp_data[j].first, cmd_idx = cp_data[j].second;
					this->RetreatRobotPath(robots[robot_id], robot_commands[cmd_idx]);
				}
			}
			else if (cp_data.size() == 3)
			{
				// һ������
				// ��robot[0], robot[1]����
				for (int j = 0; j < 2; j++)
				{
					int robot_id = cp_data[j].first, cmd_idx = cp_data[j].second;
					this->RetreatRobotPath(robots[robot_id], robot_commands[cmd_idx]);
				}
			}
			else if (cp_data.size() == 4)
			{
				// һ������
				// ��robot[0], robot[1], robot[2]����
				for (int j = 0; j < 3; j++)
				{
					int robot_id = cp_data[j].first, cmd_idx = cp_data[j].second;
					this->RetreatRobotPath(robots[robot_id], robot_commands[cmd_idx]);
				}
			}
		}
	}
}

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
	else if (robot.is_carry == 0) // ������������
	{
		cur = robot.send_good_cur;
		path = robot.send_good_path;
	}
	
	// ���������ǰһ���±�
	int back_cur = cur - 1;

	if (back_cur == -1)	// ������˵���ԭ�㣬���û�����ԭ�ز���
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
	robot_command.param_2 = param_2;
}
