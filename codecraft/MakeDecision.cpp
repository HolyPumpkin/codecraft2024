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
int MakeDecision::assignRobotGet(Robot& bot, vector<Good>& goods)
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
		if ((abs(i.x - bot.x) + abs(i.y - bot.y)) >= i.ttl)
		{
			continue;
		}
		//�ҵ�һ���ɴ���û��ָ�ɵĻ���
		PlanPath planpath(this->maze, this->N, this->n);
		Point s = Point(bot.x, bot.y);
		Point e = Point(i.x, i.y);
		bot.fetch_good_path = planpath.pathplanning(s, e);
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
	PlanPath planpath(this->maze, this->N, this->n);
	Point s = Point(bot.x, bot.y);
	Point e = Point(berths[min_id].x, berths[min_id].y);
	bot.send_good_path = planpath.pathplanning(s, e);

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
		//���ȡ��·�����߼�ָ�벻�Ϸ�������ʱ����
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
				if (bot.fetch_good_cur == path_size - 2)
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
vector<Command> MakeDecision::makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths)
{
	vector<Command> res;	//����ֵ
	//����������У�������װ���У��ͱ��ֲ���
	if (0 == boat.status || boat.is_loading)
	{
		res.push_back(Command(-1, boat_id, -1));	//��ָ��
		return res;
	}
	//�ڲ�λװ����ɣ�goȥ�����
	else if (1 == boat.status && boat.pos != -1)
	{
		res.push_back(Command(16, boat_id, -1));	//goָ��
	}
	/* �����������Ҫָ��һ����λȻ���ȥ */
	//��δ֪������û�в�λ�����Ͳ���
	if (0 == berths.size())
	{
		res.push_back(Command(-1, boat_id, -1));
		return res;
	}
	//�ҵ���λ�л����ֵ����
	int max_val = berths[0].cur_goods_val;
	int max_val_id = 0;
	for (int i = 0; i < berths.size();++i)
	{
		if (berths[i].cur_goods_val > max_val)
		{
			max_val = berths[i].cur_goods_val;
			max_val_id = i;
		}
	}
	res.push_back(Command(8, boat_id, max_val_id));	//shipָ��

	return res;
}
