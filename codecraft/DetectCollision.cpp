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

			// move <==> key == 1
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
	}

	// ����ͳ�ƺõ�move point����ȡ��collision point
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
 * RetreatRobotPath - ������·������
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
