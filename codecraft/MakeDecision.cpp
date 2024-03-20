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
int MakeDecision::assignRobotGet(Robot& bot, int robot_id, list<Good>& goods, int cur_frame_id)
{
	//�޻���
	if (goods.empty())
	{
		return -1;
	}

	// ȡ��һ��������Ϊ��׼
	Good* min_dist_gd;
	min_dist_gd = &goods.front();
	int min_dist = abs(min_dist_gd->x - bot.x) + abs(min_dist_gd->y - bot.y);

	// �������л��ȡ�����پ�����С��
	for (auto& gd : goods)
	{
		if (gd.is_assigned)
		{
			continue;
		}
		if (gd.is_ungettable[robot_id])
		{
			continue;
		}
		int dist = abs(gd.x - bot.x) + abs(gd.y - bot.y);
		if (dist >= (gd.end_frame - cur_frame_id))
		{
			continue;
		}
		if (min_dist > dist)
		{
			min_dist_gd = &gd;
			min_dist = dist;
		}

		//�����ʼѡ����С��������Ѿ���ָ�ɣ����л�
		if (min_dist_gd->is_assigned)
		{
			min_dist_gd = &gd;
			min_dist = dist;
		}
	}
	//�������ҵ����Ѿ���ָ�ɣ��򷵻�-1
	if (min_dist_gd->is_assigned)
	{
		return -1;
	}
	// �ҵ�һ���ɴ���û��ָ�ɵĻ���
	PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	Point s = Point(bot.x, bot.y);
	Point e = Point(min_dist_gd->x, min_dist_gd->y);

	// ÿ�ι滮·����ʱ��Ҫ���α�����
	bot.fetch_good_path = planpath.pathplanning(s, e);
	bot.fetch_good_cur = 0;

	// ·���滮ʧ�ܣ���������·����ʱ��Ҫ�����������Ϊ�Ըû����˲��ɴ�
	if (bot.fetch_good_path.empty())
	{
		min_dist_gd->is_ungettable[robot_id] = true;
		return -1;
	}

	// ��ָ�ɳɹ�Ҫ�޸��ڲ���������¼�û������ʱ�䣬�����˵�ǰ��ֵ
	min_dist_gd->is_assigned = true;
	bot.good_end_frame = min_dist_gd->end_frame;
	bot.robot_val = min_dist_gd->val;

	return 0;
	
	//// ԭ�����ǣ�����һ���ɴ�����ȥ����������
	//for (auto& i : goods)
	//{
	//	//�Ѿ���ָ�ɸ�һ��������
	//	if (i.is_assigned)
	//	{
	//		continue;
	//	}
	//	//���ɴ�������ڻ����˵�֮ǰ����ʧ
	//	if ((abs(i.x - bot.x) + abs(i.y - bot.y)) >= i.end_frame - cur_frame_id)
	//	{
	//		continue;
	//	}
	//	//�ҵ�һ���ɴ���û��ָ�ɵĻ���
	//	PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	//	Point s = Point(bot.x, bot.y);
	//	Point e = Point(i.x, i.y);
	//	// ÿ�ι滮·����ʱ��Ҫ���α�����
	//	bot.fetch_good_path = planpath.pathplanning(s, e);
	//	bot.fetch_good_cur = 0;
	//	// ·���滮ʧ�ܣ���������·����ʱ��Ҫ���������ȥ�������������Ч����
	//	if (bot.fetch_good_path.empty())
	//	{
	//		i.is_assigned = true;
	//		return -1;
	//	}
	//	// ��ָ�ɳɹ�Ҫ�޸��ڲ���������¼�û������ʱ�䣬�����˵�ǰ��ֵ
	//	i.is_assigned = true;
	//	bot.good_end_frame = i.end_frame;
	//	bot.robot_val = i.val;
	//	// ����·���滮
	//	/*cout << " robot (" << bot.x << "," << bot.y << ")'s fetch_path: " << endl;
	//	planpath.printPath(bot.fetch_good_path);*/

	//	return 0;
	//}
	////���δ��ѭ����return���ʾû�к��ʵ�goodָ�ɣ�ʧ��
	//return -1;
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
	//// �޲�λ
	//if (berths.empty())
	//{
	//	return -1;
	//}

	//// ����������Ѿ��������ĳ�����ڣ���ֱ��·���滮
	//if (bot.berth_id != -1)
	//{
	//	// ��λ��4*4�ģ�ȥ����һ���㶼�У����Ծ�ȥ�����һ����
	//	int target_x = berths[bot.berth_id].x + rand() % 4;
	//	int target_y = berths[bot.berth_id].y + rand() % 4;
	//	PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	//	Point s = Point(bot.x, bot.y);
	//	Point e = Point(target_x, target_y);

	//	// ÿ�ι滮·����ʱ��Ҫ���α�����
	//	bot.send_good_path = planpath.pathplanning(s, e);
	//	bot.send_good_cur = 0;

	//	// ·���滮����
	//	if (bot.send_good_path.empty())
	//	{
	//		return -1;
	//	}

	//	return 0;
	//}

	//// ���������δ�������ĳ�����ڣ���ʼ������

	//// ��������
	//for (auto& bth : berths)
	//{
	//	// �����ǰ�������������˸��� < 2�����п��пɳ��Է���
	//	if (bth.rbt_seq.size() < 2)
	//	{
	//		// ��λ��4*4�ģ�ȥ����һ���㶼�У����Ծ�ȥ�����һ����
	//		int target_x = berths[bot.berth_id].x + rand() % 4;
	//		int target_y = berths[bot.berth_id].y + rand() % 4;
	//		PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	//		Point s = Point(bot.x, bot.y);
	//		Point e = Point(target_x, target_y);

	//		// ÿ�ι滮·����ʱ��Ҫ���α�����
	//		bot.send_good_path = planpath.pathplanning(s, e);
	//		bot.send_good_cur = 0;

	//		// ·���滮ʧ�ܣ������ǲ��ڲ��ɴ�
	//		if (bot.send_good_path.empty())
	//		{
	//			
	//		}
	//		// ·���滮�ɹ���֤�����ڿɴ�
	//		else
	//		{
	//			// �޸Ļ����˲���
	//			bot.berth_id = bth.id;
	//			// ���ڼ���û�����ID
	//			bth.rbt_seq.push_back(bot.id);
	//		}
	//	}
	//	else
	//	{

	//	}
	//}








	 // ԭ�汾=================================================================
	 
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
		//��ǰ��λ���ɴ�
		if (bot.is_ungettable[berths[i].id] == 0)
		{
			continue;
		}
		temp = abs(berths[i].x - bot.x) + abs(berths[i].y - bot.y);
		if (temp < min_dis)
		{
			min_dis = temp;
			min_id = i;
		}
		// ��ǰ�������������λ���ɴ��Ҫ�ı䲴λ
		if (bot.is_ungettable[berths[min_id].id] == 0)
		{
			min_id = i;
			min_dis = temp;
		}
	}
	//�滮����·������������˽ṹ��
	//��λ��4*4�ģ�ȥ����һ���㶼�У����Ծ�ȥ�����һ����
	int target_x = berths[min_id].x + rand() % 4;
	int target_y = berths[min_id].y + rand() % 4;
	PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	Point s = Point(bot.x, bot.y);
	Point e = Point(target_x, target_y);
	// ÿ�ι滮·����ʱ��Ҫ���α�����
	bot.send_good_path = planpath.pathplanning(s, e);
	//����滮��·�����ɴҪ����һ�£�����������´λ�ȥ�����λ
	if (bot.send_good_path.empty())
	{
		bot.is_ungettable[berths[min_id].id] = 0;
		return -1;
	}
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
	//��ʼ���ִ�λ��
	if (frame_id == 1)
	{
		res.push_back(Command(8, boat_id, boat_id));	//shipָ��
		//����ȥ�����λ����ʱҪ�Ѳ�λ��״̬��Ϊ��ռ�ã����������ִ��ظ�����
		berths[boat_id].is_occupied = 1;
		return res;
	}
	//���ʣ��ʱ�䲻���ˣ������ִ��������һ�裡
	if(boat.pos != -1)
	{
		//��10֡��Ϊ����������֤������ܵ������
		if (15000 - frame_id < berths[boat.pos].transport_time + 10)
		{
			res.push_back(this->lastDance(boat_id));
			return res;
		}
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

					//�ִ��뿪Ҫ���²�λ��������
					int temp_value = berths[boat.pos].cur_goods_num - berths[boat.pos].loading_speed * boat.loading_time;
					berths[boat.pos].cur_goods_num = (temp_value > 0) ? temp_value : 0;
				}
				boat.is_loading = false;
			}
		}
	}
	//boat��status��0���1
	if (1 == cur_status)
	{
		//�������ж����ɣ�ȥ��λ
		if (-1 == boat.pos)
		{
			boat.cur_load = 0;
			//�ҵ���λ�л�����������
			int max_val = berths[0].cur_goods_num;
			int max_val_id = 0;
			for (int i = 1; i < berths.size(); ++i)
			{
				if (berths[i].cur_goods_num > max_val && berths[i].is_occupied == 0)
				{
					// �����˷Ż����ʱ��Ҫ�ı��Ӧ��λ�Ļ����ֵ�ͻ�����
					max_val = berths[i].cur_goods_num;
					max_val_id = i;
				}
			}
			res.push_back(Command(8, boat_id, max_val_id));	//shipָ��

			//����ȥ�����λ����ʱҪ�Ѳ�λ��״̬��Ϊ��ռ�ã����������ִ��ظ�����
			berths[max_val_id].is_occupied = 1;

			//��̬�ı䵱ǰ�ִ���װ��ʱ�䣬����Ҫȥ�Ĳ�λ�����������ִ�������װ���ٶ��ж�
			if (berths[max_val_id].cur_goods_num >= boat.capacity)
			{
				boat.loading_time = boat.capacity / berths[max_val_id].loading_speed;
			}
			else if (berths[max_val_id].cur_goods_num < boat.capacity)
			{
				boat.loading_time = berths[max_val_id].cur_goods_num / berths[max_val_id].loading_speed;
			}

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

				//�ִ��뿪Ҫ���²�λ��������
				int temp_value = berths[boat.pos].cur_goods_num - berths[boat.pos].loading_speed * boat.loading_time;
				berths[boat.pos].cur_goods_num = (temp_value > 0) ? temp_value : 0;
			}
		}
		else
		{
			//�ҵ���λ�л�����������
			int max_val = berths[0].cur_goods_num;
			int max_val_id = 0;
			for (int i = 1; i < berths.size(); ++i)
			{
				if (berths[i].cur_goods_num > max_val && berths[i].is_occupied == 0)
				{
					// �����˷Ż����ʱ��Ҫ�ı��Ӧ��λ�Ļ����ֵ�ͻ�����
					max_val = berths[i].cur_goods_num;
					max_val_id = i;
				}
			}
			res.push_back(Command(8, boat_id, max_val_id));	//shipָ��
			//����ȥ�����λ����ʱҪ�Ѳ�λ��״̬��Ϊ��ռ�ã����������ִ��ظ�����
			berths[boat_id].is_occupied = 1;

			//��̬�ı䵱ǰ�ִ���װ��ʱ�䣬����Ҫȥ�Ĳ�λ�����������ִ�������װ���ٶ��ж�
			if (berths[max_val_id].cur_goods_num >= boat.capacity)
			{
				boat.loading_time = boat.capacity / berths[max_val_id].loading_speed;
			}
			else if (berths[max_val_id].cur_goods_num < boat.capacity)
			{
				boat.loading_time = berths[max_val_id].cur_goods_num / berths[max_val_id].loading_speed;
			}

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
	this->own_robots = robots;	//ÿ���ȸ����ڲ�������Ϊ����
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

/**
 * @brief ��������Ӧ�ñ������Ļ����ˣ�����һЩ״̬��
 * @param robots 
*/
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

/**
 * @brief ������״̬Ǩ��
 * @param robots 
*/
void MakeDecision::robotStatusTrans(vector<Robot>& robots)
{
	for (auto& i : robots)
	{
		i.last_status = i.status;
	}
}

/**
 * @brief ���ݵ�ǰ֡����Ӧ����ʧ�Ļ�����д���
 * @param goods 
 * @param frame_id 
*/
void MakeDecision::vanishGoods(list<Good>& goods, int frame_id)
{
	for (auto it = goods.begin(); it != goods.end();)
	{
		if (frame_id >= it->end_frame)
		{
			it = goods.erase(it);
		}
		else
		{
			++it;
		}
	}
}

/**
 * @brief ��װÿһ֡�Ի����˵Ĳ�����ָ�ɡ�����ָ���
 * @param robots 
 * @param goods 
 * @param berths 
 * @param frame_id 
*/
void MakeDecision::robotsOperate(vector<Robot>& robots, int robot_num, vector<vector<Command>>& robot_cmd, list<Good>& goods, vector<Berth>& berths, int frame_id)
{
	for (int rbt_idx = 0; rbt_idx < robot_num; ++rbt_idx)
	{
		/* �ع����� */
		// is_carry�����ı䣬��ʾ�õ����߷����˻����ʱҪ�޸�is_assigned��ֵΪ0����ʾҪ����ָ��
		if (robots[rbt_idx].is_carry == robots[rbt_idx].last_is_carry)
		{
			// ���is_carry��1���0����ʾ�ڲ�λ�����˻��Ҫ���Ķ�Ӧ��λ����Ϣ
			if (0 == robots[rbt_idx].is_carry)
			{
				berths[robots[rbt_idx].berth_id].cur_goods_num++;
				berths[robots[rbt_idx].berth_id].cur_goods_val += robots[rbt_idx].robot_val;
			}

			robots[rbt_idx].is_assigned = 0;
			// ��֤����������is_carryʼ�ղ�ͬ
			robots[rbt_idx].last_is_carry = robots[rbt_idx].last_is_carry ^ 1;
		}

		// �����ǰ������δЯ������
		if (0 == robots[rbt_idx].is_carry)
		{
			// �����ǰ�������Ѿ���ָ��
			if (1 == robots[rbt_idx].is_assigned)
			{
				// �жϻ���Ĵ��ʱ��,�����ʱ�����Ѿ���ʧ
				if (frame_id >= robots[rbt_idx].good_end_frame)
				{
					robots[rbt_idx].is_assigned = 0;
					// ��ָ�ɻ�����ȥ�û���
					int assign_success = this->assignRobotGet(robots[rbt_idx], rbt_idx, goods, frame_id);

					// ���ָ��ʧ�ܣ������ָ���ʾ�����κβ���
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
						continue;
					}
					// ָ�ɳɹ�
					else if (0 == assign_success) {
						robots[rbt_idx].is_assigned = 1;	// �޸��ڲ�����
					}
				}
				// ֱ������ָ��
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
			// �����ǰ������δ��ָ��
			else if (0 == robots[rbt_idx].is_assigned)
			{
				// ��ָ�ɻ�����ȥ�û���
				int assign_success = this->assignRobotGet(robots[rbt_idx], rbt_idx, goods, frame_id);

				// ���ָ��ʧ�ܣ������ָ���ʾ�����κβ���
				if (-1 == assign_success)
				{
					robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
					continue;
				}
				// ָ�ɳɹ�
				else if (0 == assign_success) {
					robots[rbt_idx].is_assigned = 1;	// �޸��ڲ�����
				}
				// ����ָ��
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
		}
		// �����ǰ������Я������
		else if (1 == robots[rbt_idx].is_carry)
		{
			// �����ǰ�������Ѿ���ָ��
			if (1 == robots[rbt_idx].is_assigned)
			{
				// ֱ������ָ��
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
			// �����ǰ������δ��ָ��
			else if (0 == robots[rbt_idx].is_assigned)
			{
				// ��ָ�ɻ�����ȥ�û���
				int assign_success = this->assighRobotSend(robots[rbt_idx], berths);

				// ���ָ��ʧ�ܣ������ָ���ʾ�����κβ���
				if (-1 == assign_success)
				{
					robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
					continue;
				}
				// ָ�ɳɹ�
				else if (0 == assign_success) {
					robots[rbt_idx].is_assigned = 1;	// �޸��ڲ�����
				}
				// ����ָ��
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
		}
	}
}

/**
 * @brief ��װÿһ֡���ִ��Ĳ���������ָ���
 * @param boats 
 * @param frame_id 
*/
void MakeDecision::boatsOperate(vector<Boat>& boats, vector<vector<Command>>& boat_cmd, vector<Berth>& berths, int boat_num, int frame_id)
{
	for (int boat_idx = 0; boat_idx < boat_num; ++boat_idx)
	{
		boat_cmd[boat_idx] = this->makeBoatCmd(boats[boat_idx], boat_idx, berths, frame_id);
	}
}

/**
 * @brief �ִ������һ���ͻ�
 * @param boat 
 * @return ָ������
*/
Command MakeDecision::lastDance(int boat_id)
{
	return Command(16, boat_id, -1);	//goָ��
}

