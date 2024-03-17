/*
 * Change Logs:
 * Date           Author       Notes
 * 2024-3-13     Haoyu Nan     the first version
 */

#include "MakeDecision.h"


 /**
  * MakeDecision - �������������ʵ��
  * @_maze: ��ά�ַ���ͼ
  * @_N: �����ͼ��С
  * @_n: ʵ�ʵ�ͼ��С
  *
  * �������������������MakeDecision��ʵ��
  *
  * ����ֵ:
  * ��
  *
  * ע������/Լ��������
  * ��
  */
MakeDecision::MakeDecision(vector<vector<char>>& _maze, int _N, int _n)
{
	this->maze = _maze;
	this->N = _N;
	this->n = _n;
}

/**
   * assignRobotGet - ��������ָ��ĳ��������ȥȡĳ������
   * @bot: �����˽ṹ��
   * @goods: ����ṹ�弯��
   *
   * ��������������������ȡ��·�������ڻ����˽ṹ���ڲ�
   *
   * ����ֵ:
   * ����ֵ��0��ʾָ�ɳɹ���-1��ʾָ��ʧ�ܣ������޻��
   *
   * ע������/Լ��������
   * ָ��ʧ�ܵ���������ж��֣���������
   */
int MakeDecision::assignRobotGet(Robot& bot, list<Good>& goods, int cur_frame_id)
{
	//�޻���
	if (goods.empty())
	{
		return -1;
	}
	//��ǰ�����ǣ�����һ���ɴ�����ȥ����������
	for (auto& i : goods)
	{
		//�Ѿ���ָ�ɸ�һ��������
		if (i.is_assigned)
		{
			continue;
		}
		//���ɴ�������ڻ����˵�֮ǰ����ʧ
		if ((abs(i.x - bot.x) + abs(i.y - bot.y)) >= i.end_frame - cur_frame_id)
		{
			continue;
		}
		//�ҵ�һ���ɴ���û��ָ�ɵĻ���
		PlanPath planpath(this->maze, this->N, this->n, this->robots);
		Point s = Point(bot.x, bot.y);
		Point e = Point(i.x, i.y);
		// ÿ�ι滮·����ʱ��Ҫ���α�����
		bot.fetch_good_path = planpath.pathplanning(s, e);
		bot.fetch_good_cur = 0;
		// ·���滮ʧ�ܣ���������·
		if (bot.fetch_good_path.empty())
		{
			return -1;
		}
		// ��ָ�ɳɹ�Ҫ�޸��ڲ���������¼�û������ʱ�䣬�����˵�ǰ��ֵ
		i.is_assigned = true;
		bot.good_end_frame = i.end_frame;
		bot.robot_val = i.val;
		// ����·���滮
		/*cout << " robot (" << bot.x << "," << bot.y << ")'s fetch_path: " << endl;
		planpath.printPath(bot.fetch_good_path);*/

		return 0;
	}
	//���δ��ѭ����return���ʾû�к��ʵ�goodָ�ɣ�ʧ��
	return -1;
}

/**
   * assignRobotSend - ��������ָ��ĳ��������ȥ��ĳ������
   * @bot: �����˽ṹ��
   * @berths: ��λ�ṹ�弯��
   *
   * ������������������������·�������ڻ����˽ṹ���ڲ�
   *
   * ����ֵ:
   * ����ֵ��0��ʾָ�ɳɹ���-1��ʾָ��ʧ��
   *
   * ע������/Լ��������
   * ָ��ʧ�ܵ���������ж��֣���������
   */
int MakeDecision::assighRobotSend(Robot& bot, vector<Berth>& berths)
{
	//�޲�λ
	if (berths.empty())
	{
		return -1;
	}
	//��λ�������ɶ�������ˣ�ֱ����һ�������
	int min_dis = INT_MAX;
	int min_id = 0;	//Ĭ��ȥ0
	int temp = 0;
	for (int i = 0; i < berths.size(); ++i)
	{
		temp = abs(berths[i].x - bot.x) + abs(berths[i].y - bot.y);
		if (temp < min_dis)
		{
			min_dis = temp;
			min_id = i;
		}
	}
	//�滮����·������������˽ṹ��
	PlanPath planpath(this->maze, this->N, this->n, this->robots);
	Point s = Point(bot.x, bot.y);
	Point e = Point(berths[min_id].x, berths[min_id].y);
	// ÿ�ι滮·����ʱ��Ҫ���α�����
	bot.send_good_path = planpath.pathplanning(s, e);
	bot.send_good_cur = 0;
	bot.berth_id = min_id;
	return 0;
}



/**
  * makeRobotCmd - ������������ĳ�������˵�ǰӦ���е�ָ��
  * @bot: �����˽ṹ��
  * @bot_id: ������id
  *
  * �����������������ɶ�Ӧ�����˵�ǰָ��
  *
  * ����ֵ:
  * ��ָ��ṹ��ΪԪ�ص�vector
  *
  * ע������/Լ��������
  * ��Щʱ����ܻ����ɶ���ָ�
  * ��������˲���Ҳ������key==-1�Ŀ�ָ��
  */
vector<Command> MakeDecision::makeRobotCmd(Robot& bot, int bot_id)
{
	int path_size = 0;	//·������

	vector<Command> res;	//���
	//ȡ�������
	if (0 == bot.is_carry)
	{
		path_size = bot.fetch_good_path.size();
		//���ȡ��·�����߼�ָ�벻�Ϸ�������ʱ����
		if (bot.fetch_good_cur < 0 || bot.fetch_good_cur >= path_size)
		{
			res.push_back(Command(-1, bot_id, -1));
		}
		//�����ǰ��Ŀ��㣬��Ӧ��ȡ�����Ϊ������Ϊ������ײ��ɵı����ߵ�Ŀ���
		else if (bot.fetch_good_cur == path_size - 1)
		{
			res.push_back(Command(2, bot_id, -1));	//ȡ��
		}
		//�����·���У�������·����
		else
		{
			int dir = -1;	
			//X��������1��
			if (bot.fetch_good_path[bot.fetch_good_cur + 1].first - bot.fetch_good_path[bot.fetch_good_cur].first > 0)
			{
				dir = 3;	//����һ��
			}
			//X��������1��
			else if (bot.fetch_good_path[bot.fetch_good_cur + 1].first - bot.fetch_good_path[bot.fetch_good_cur].first < 0)
			{
				dir = 2;	//����һ��
			}
			//Y��������1��
			else if (bot.fetch_good_path[bot.fetch_good_cur + 1].second - bot.fetch_good_path[bot.fetch_good_cur].second > 0)
			{
				dir = 0;	//����һ��
			}
			//Y��������1��
			else if (bot.fetch_good_path[bot.fetch_good_cur + 1].second - bot.fetch_good_path[bot.fetch_good_cur].second < 0)
			{
				dir = 1;	//����һ��
			}
			//����δ֪�����������·����һ����������Ǿ��û����˲���
			if (-1 == dir || dir > 3)
			{
				res.push_back(Command(-1, bot_id, -1));	//��ָ��
			}
			//move��Ӧ�ķ���
			else
			{
				res.push_back(Command(1, bot_id, dir));
				//�����ǰ����һ���͵���Ŀ��㣬������ٲ���һ��getָ��
				if (bot.fetch_good_cur == path_size - 2)
				{
					res.push_back(Command(2, bot_id, -1));	//ȡ��
				}
			}
		}
	}
	//���������
	else
	{
		path_size = bot.send_good_path.size();
		//�������·�����߼�ָ�벻�Ϸ�������ʱ����
		if (bot.send_good_cur < 0 || bot.send_good_cur >= path_size)
		{
			res.push_back(Command(-1, bot_id, -1));
		}
		//�����ǰ��·����ͷ����Ӧ������
		else if (bot.send_good_cur == path_size - 1)
		{
			res.push_back(Command(4, bot_id, -1));	//����
		}
		//�����·���У�������·����
		else
		{
			int dir = -1;
			//X��������1��
			if (bot.send_good_path[bot.send_good_cur + 1].first - bot.send_good_path[bot.send_good_cur].first > 0)
			{
				dir = 3;	//����һ��
			}
			//X��������1��
			else if (bot.send_good_path[bot.send_good_cur + 1].first - bot.send_good_path[bot.send_good_cur].first < 0)
			{
				dir = 2;	//����һ��
			}
			//Y��������1��
			else if (bot.send_good_path[bot.send_good_cur + 1].second - bot.send_good_path[bot.send_good_cur].second > 0)
			{
				dir = 0;	//����һ��
			}
			//Y��������1��
			else if (bot.send_good_path[bot.send_good_cur + 1].second - bot.send_good_path[bot.send_good_cur].second < 0)
			{
				dir = 1;	//����һ��
			}
			//����δ֪�����������·����һ����������Ǿ��û����˲���
			if (-1 == dir || dir > 3)
			{
				res.push_back(Command(-1, bot_id, -1));	//��ָ��
			}
			//move��Ӧ�ķ���
			else
			{
				res.push_back(Command(1, bot_id, dir));
				//�����ǰ����һ���͵���Ŀ��㣬������ٲ���һ��pullָ��
				if (bot.send_good_cur == path_size - 2)
				{
					res.push_back(Command(4, bot_id, -1));	//����
				}
			}
		}
	}
	return res;
}

/**
 * makeBoatCmd - ������������ĳ���ִ���ǰӦ���е�ָ��
 * @bot: �ִ��ṹ��
 * @bot_id: �ִ�id
 *
 * �����������������ɶ�Ӧ�ִ���ǰָ��
 *
 * ����ֵ:
 * ��ָ��ṹ��ΪԪ�ص�vector
 *
 * ע������/Լ��������
 * ��Щʱ����ܻ����ɶ���ָ�
 * �����ִ�����Ҳ������key==-1�Ŀ�ָ��
 */
vector<Command> MakeDecision::makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths, int frame_id)
{
	vector<Command> res;	//����ֵ
	if (frame_id == 1)
	{
		res.push_back(Command(8, boat_id, boat_id));	//shipָ��
		//����ȥ�����λ����ʱҪ�Ѳ�λ��״̬��Ϊ��ռ�ã����������ִ��ظ�����
		berths[boat_id].is_occupied = 1;
		return res;
	}

	//�õ��ִ���ǰ״̬
	int cur_status = this->boatStatusCheck(boat);
	
	//����������У�������װ���У��ͱ��ֲ���
	if (0 == boat.status || boat.is_loading)
	{
		res.push_back(Command(-1, boat_id, -1));	//��ָ��
		return res;
	}
	//״̬δ�ı�
	if (cur_status == -1)
	{
		if (boat.status == 1)
		{
			//�������װ��������ʱ�䣬ȥ�����
			if (frame_id >= boat.end_load_frame)
			{
				res.push_back(Command(16, boat_id, -1));	//goָ��
				//�뿪��λȥ����㣬��Ҫ�ı䲴λ״̬Ϊδ��ռ��
				if (boat.pos != -1)
				{
					berths[boat.pos].is_occupied = 0;
				}
				boat.is_loading = false;
			}
		}
	}
	//boat��status��0���1
	if (1 == cur_status)
	{
		//�������ж�����
		if (-1 == boat.pos)
		{
			boat.cur_load = 0;
			//�ҵ���λ�л����ֵ����
			int max_val = berths[0].cur_goods_val;
			int max_val_id = 0;
			for (int i = 1; i < berths.size(); ++i)
			{
				if (berths[i].cur_goods_val > max_val && berths[i].is_occupied == 0)
				{
					// TODO �����˷Ż����ʱ��Ҫ�ı��Ӧ��λ�Ļ����ֵ�ͻ�����
					max_val = berths[i].cur_goods_val;
					max_val_id = i;
				}
			}
			res.push_back(Command(8, boat_id, max_val_id));	//shipָ��
			//����ȥ�����λ����ʱҪ�Ѳ�λ��״̬��Ϊ��ռ�ã����������ִ��ظ�����
			berths[max_val_id].is_occupied = 1;


		}
		//����λװ��
		else
		{
			boat.is_loading = true;
			res.push_back(Command(-1, boat_id, -1));	//��ָ��
		}
	}
	//boat��status��1���0
	else if (0 == cur_status)
	{
		boat.is_loading = false;
		//����ڲ�λ
		if (boat.pos != -1)
		{
			//ȥ�����
			res.push_back(Command(16, boat_id, -1));	//goָ��
			//�뿪��λȥ����㣬��Ҫ�ı䲴λ״̬Ϊδ��ռ��
			if (boat.pos != -1)
			{
				berths[boat.pos].is_occupied = 0;
			}
		}
		else
		{
			//�ҵ���λ�л����ֵ����
			int max_val = berths[0].cur_goods_val;
			int max_val_id = 0;
			for (int i = 0; i < berths.size(); ++i)
			{
				if (berths[i].cur_goods_val > max_val && berths[i].is_occupied == 0)
				{
					// TODO �����˷Ż����ʱ��Ҫ�ı��Ӧ��λ�Ļ����ֵ�ͻ�����
					max_val = berths[i].cur_goods_val;
					max_val_id = i;
				}
			}
			res.push_back(Command(8, boat_id, max_val_id));	//shipָ��
			//����ȥ�����λ����ʱҪ�Ѳ�λ��״̬��Ϊ��ռ�ã����������ִ��ظ�����
			berths[boat_id].is_occupied = 1;
		}
	}
	//�������ֱ��ȥ�����
	else
	{
		boat.is_loading = false;
		res.push_back(Command(16, boat_id, -1));	//goָ��
	}
	return res;
}

/**
 * @brief ���ɻ����˿�ָ��
 * @param robot_id 
 * @return vector<Command>
*/
vector<Command> MakeDecision::makeNullCmd(int robot_id)
{
	vector<Command> res;
	res.push_back(Command(-1, robot_id, -1));
	return res;
}

/**
 * @brief ���ÿһ֡�Ļ��������룬�����߼�ָ��ֱ���˴����п��ܱ��޸ģ�
 *        �������ж�ʱ�߼�ָ��ָ�����ʵ�ǻ�������һ֡��λ�á�
 * 
 * @note  �˺�����Ҫ�Ǳ�֤ÿһ֡����ָ��֮ǰ���α��λ������ȷ��
 * 
 * @param robots 
*/
void MakeDecision::robotInputCheck(vector<Robot>& robots, list<Good>& goods, int cur_frame_id)
{
	this->robots = robots;	//ÿ���ȸ����ڲ�������Ϊ����
	for (int rbt_idx = 0; rbt_idx < robots.size(); ++rbt_idx)
	{
		//�����ǰ��������δЯ����Ʒ��״̬
		if (0 == robots[rbt_idx].is_carry && !robots[rbt_idx].fetch_good_path.empty())
		{
			//�α겻�Ϸ�
			/*if (robots[rbt_idx].fetch_good_cur > robots[rbt_idx].fetch_good_path.size() - 1
				|| robots[rbt_idx].fetch_good_cur < 0)
			{
				continue;
			}*/
			// ȡ�������˵�ǰ��λ�ú���һ֡��λ��
			std::pair<int, int> cur_pos(robots[rbt_idx].x, robots[rbt_idx].y);
			std::pair<int, int> pre_pos = robots[rbt_idx].fetch_good_path[robots[rbt_idx].fetch_good_cur];

			//�������ȡ��·���߼�ָ����ָ��λ�ã����������һ֡û��
			//֤���û����ˣ�1.��ײ��2.get��3.pull��4.��ָ��
			//���Դ�ʱ���޸��α�
			if (cur_pos == pre_pos)
			{
				continue;
			}
			//������겻��ȣ�˵�������˿϶����ˣ�Ҫô����·�����ˣ�Ҫôƫ��·��
			else if (cur_pos != pre_pos)
			{
				//�ж��Ƿ�����·����
				//�����һ�����յ㣬����������������·���send·��
				if (robots[rbt_idx].fetch_good_cur == robots[rbt_idx].fetch_good_path.size() - 1)
				{
					continue;
				}
				//�����һ������㣬ֻ�ж��Ƿ��ں�һ��
				else if (robots[rbt_idx].fetch_good_cur == 0)
				{
					//��·���У����޸��߼�ָ��
					if (robots[rbt_idx].fetch_good_path[robots[rbt_idx].fetch_good_cur + 1] == cur_pos)
					{
						robots[rbt_idx].fetch_good_cur = robots[rbt_idx].fetch_good_cur + 1;
					}

					//û����·���У���Ҫ���¹滮·��
					else
					{
						//�޸�ָ�ɱ������������¹滮
						robots[rbt_idx].is_assigned = 0;
					}
				}
				//�����·���У����������յ㣬��ֹ�±�Խ�磩
				else if (robots[rbt_idx].fetch_good_cur < robots[rbt_idx].fetch_good_path.size() - 1
			     && robots[rbt_idx].fetch_good_cur > 0)
				{
					//����·���ߣ�ǰһ�����ߺ�һ���������߼�ָ���޸�
					if (robots[rbt_idx].fetch_good_path[robots[rbt_idx].fetch_good_cur - 1] == cur_pos)
					{
						robots[rbt_idx].fetch_good_cur = robots[rbt_idx].fetch_good_cur - 1;
					}
					else if (robots[rbt_idx].fetch_good_path[robots[rbt_idx].fetch_good_cur + 1] == cur_pos)
					{
						robots[rbt_idx].fetch_good_cur = robots[rbt_idx].fetch_good_cur + 1;
					}

					//û����·���У���Ҫ���¹滮·��
					else
					{
						//�޸�ָ�ɱ������������¹滮
						robots[rbt_idx].is_assigned = 0;
					}
				}

			}

		}
		//�����ǰ��������Я����Ʒ��״̬
		else if (1 == robots[rbt_idx].is_carry && !robots[rbt_idx].send_good_path.empty())
		{
			// ȡ�������˵�ǰ��λ�ú���һ֡��λ��
			std::pair<int, int> cur_pos(robots[rbt_idx].x, robots[rbt_idx].y);
			std::pair<int, int> pre_pos = robots[rbt_idx].send_good_path[robots[rbt_idx].send_good_cur];

			//�������ȡ��·���߼�ָ����ָ��λ�ã����������һ֡û��
			//֤���û����ˣ�1.��ײ��2.get��3.pull��4.��ָ��
			//���Դ�ʱ���޸��α�
			if (cur_pos == pre_pos)
			{
				continue;
			}
			//������겻��ȣ�˵�������˿϶����ˣ�Ҫô����·�����ˣ�Ҫôƫ��·��
			else if (cur_pos != pre_pos)
			{
				//�ж��Ƿ�����·����
				//�����һ�����յ㣬����������������·���send·��
				if (robots[rbt_idx].send_good_cur == robots[rbt_idx].send_good_path.size() - 1)
				{
					continue;
				}
				//�����һ������㣬ֻ�ж��Ƿ��ں�һ��
				else if (robots[rbt_idx].send_good_cur == 0)
				{
					//��·���У����޸��߼�ָ��
					if (robots[rbt_idx].send_good_path[robots[rbt_idx].send_good_cur + 1] == cur_pos)
					{
						robots[rbt_idx].send_good_cur = robots[rbt_idx].send_good_cur + 1;
					}

					//û����·���У���Ҫ���¹滮·��
					else
					{
						//�޸�ָ�ɱ������������¹滮
						robots[rbt_idx].is_assigned = 0;
					}
				}
				//�����·���У����������յ㣬��ֹ�±�Խ�磩
				else if (robots[rbt_idx].send_good_cur < robots[rbt_idx].send_good_path.size() - 1
					&& robots[rbt_idx].send_good_cur > 0)
				{
					//����·���ߣ�ǰһ�����ߺ�һ���������߼�ָ���޸�
					if (robots[rbt_idx].send_good_path[robots[rbt_idx].send_good_cur - 1] == cur_pos)
					{
						robots[rbt_idx].send_good_cur = robots[rbt_idx].send_good_cur - 1;
					}
					else if (robots[rbt_idx].send_good_path[robots[rbt_idx].send_good_cur + 1] == cur_pos)
					{
						robots[rbt_idx].send_good_cur = robots[rbt_idx].send_good_cur + 1;
					}

					//û����·���У���Ҫ���¹滮·��
					else
					{
						//�޸�ָ�ɱ������������¹滮
						robots[rbt_idx].is_assigned = 0;
					}
				}

			}
		}
	}
}

/**
 * @brief ����ÿһ֡�������Ϣ�ж��ִ���״̬
 * @param boat 
 * @return ����ֵ����ͬ��ֵ�ֱ��Ӧ���¼���״̬
 * -1��boat��statusû�仯
 * 0��boat��status��1���0
 * 1��boat��status��0���1
 * 2��boat��status��2���1
 * 3��boat��status��2���0
 * 4��boat��status��0���2
 * 
*/
int MakeDecision::boatStatusCheck(Boat& boat)
{
	if (boat.last_status == boat.status)
	{
		return -1;
	}
	else if (boat.last_status == 1 && boat.status == 0)
	{
		return 0;
	}
	else if (boat.last_status == 0 && boat.status == 1)
	{
		return 1;
	}
	else if (boat.last_status == 2 && boat.status == 1)
	{
		return 2;
	}
	else if (boat.last_status == 2 && boat.status == 0)
	{
		return 3;
	}
	else if (boat.last_status == 0 && boat.status == 2)
	{
		return 4;
	}
	return -1;
}

/**
 * @brief ����ÿһ֡����������ִ���Ϣ
 * @param boats 
*/
void MakeDecision::boatInputCheck(vector<Boat>& boats, int frame_id)
{
	for (int i = 0; i < boats.size(); ++i)
	{
		// �����ǰ֡���ڵ��ڽ���װ����֡�����޸�is_loading����ʾװ�����
		if (frame_id >= boats[i].end_load_frame)
		{
			boats[i].is_loading = false;
		}
		// ��������Ҫ����Ʒ���
		if (boats[i].pos == -1)
		{
			boats[i].cur_load = 0;
		}
		// �õ��ִ���ǰ��״̬
		int boat_status = this->boatStatusCheck(boats[i]);
		if (boat_status == 1 || boat_status == 2)
		{
			boats[i].start_load_frame = frame_id;
			boats[i].end_load_frame = frame_id + boats[i].loading_time;
		}

	}
}

/**
 * @brief Ǩ���ִ���״̬��Ϣ
 * @param boats 
*/
void MakeDecision::boatStatusTrans(vector<Boat>& boats)
{
	for (auto& i : boats)
	{	
		i.last_status = i.status;
	}
}

/**
 * @brief ����������������
 * @param robot 
 */
void MakeDecision::robotReboot(Robot& robot)
{
	// ȡ��ָ��״̬
	robot.is_assigned = 0;

	// ���·����Ϣ
	robot.fetch_good_cur = 0;
	robot.send_good_cur = 0;
	robot.fetch_good_path.clear();
	robot.send_good_path.clear();

	if (robot.is_carry == 0)		// ���������δЯ����Ʒ
	{
		robot.last_is_carry = 1;
		robot.good_end_frame = -1;
		robot.robot_val = 0;
	}
	else if (robot.is_carry == 1)	// ���������Я����Ʒ
	{
		robot.last_is_carry = 0;
		robot.berth_id = 0;
	}
}

void MakeDecision::rebootRobots(vector<Robot>& robots)
{
	for (auto& bot : robots)
	{
		if (bot.last_status == bot.status)
		{
			continue;
		}
		else if (bot.status == 1)
		{
			continue;
		}
		else if (bot.status == 0)
		{
			robotReboot(bot);
		}
	}
}

void MakeDecision::robotStatusTrans(vector<Robot>& robots)
{
	for (auto& i : robots)
	{
		i.last_status = i.status;
	}
}

