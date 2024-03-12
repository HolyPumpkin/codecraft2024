#include "DetectCollision.h"

void DetectCollision::DetectRobotInNextStep(vector<Command>& robot_commands, vector<Robot>& robots)
{
	// 0: right; 1: left; 2: up; 3: down
	int dx[4] = {0, 0, -1, 1}, dy[4] = {1, -1, 0, 0};
	
	// ȡ��ָ������move�����Ļ�����ID
	vector<pair<int, int>> move_robots;
	for (auto& robot_cmd : robot_commands)
	{
		// move��key == 1
		int key = robot_cmd.key, id = robot_cmd.id, param_2 = robot_cmd.param_2;
		if (key == 1)
		{
			// ȡ�������˵�ǰ����
			int x = robots[id].x, y = robots[id].y;

			// �õ�������move�������
			int nx = x + dx[param_2], ny = y + dy[param_2];

			// TODO �ù�ϣͳ�Ƴ�ͻ�㣬��Ҫ���ָ���±꣬��������޸�ָ��Ĳ���

		}
	}


}
