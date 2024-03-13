/*
 * Change Logs:
 * Date           Author       Notes
 * 2024-3-13     Haoyu Nan     the first version
 */

#include "MakeDecision.h"




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
	//����������У��ͱ��ֲ���
	if (0 == boat.status)
	{
		res.push_back(Command(-1, boat_id, -1));
		return res;
	}
	//װ�����������
	else if (1 == boat.status)
	{

	}


	//��δ֪������û�в�λ�����Ͳ���
	if (0 == berths.size())
	{
		res.push_back(Command(-1, boat_id, -1));
		return res;
	}
	//�ҵ���λ������
	Berth temp = berths[0];
	for (auto& i : berths)
	{
		if (i.cur_goods_val > temp.cur_goods_val)
		{
			temp = i;
		}
	}

	return res;
}
