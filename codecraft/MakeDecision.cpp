/*
 * Change Logs:
 * Date           Author       Notes
 * 2024-3-13     Haoyu Nan     the first version
 */

#include "MakeDecision.h"




 /**
  * makeRobotCmd - 根据输入生成某个机器人当前应进行的指令
  * @bot: 机器人结构体
  * @bot_id: 机器人id
  *
  * 接收两个参数，生成对应机器人当前指令
  *
  * 返回值:
  * 以指令结构体为元素的vector
  *
  * 注意事项/约束条件：
  * 有些时候可能会生成多条指令；
  * 就算机器人不动也会生成key==-1的空指令
  */
vector<Command> MakeDecision::makeRobotCmd(Robot& bot, int bot_id)
{
	int path_size = 0;	//路径长度

	vector<Command> res;	//输出
	//取物机器人
	if (0 == bot.is_carry)
	{
		path_size = bot.fetch_good_path.size();
		//如果取物路径的逻辑指针不合法，则暂时不动
		if (bot.fetch_good_cur < 0 || bot.fetch_good_cur >= path_size)
		{
			res.push_back(Command(-1, bot_id, -1));
		}
		//如果当前在目标点，则应该取物，这条为了在因为避免碰撞造成的被动走到目标点
		else if (bot.fetch_good_cur == path_size - 1)
		{
			res.push_back(Command(2, bot_id, -1));	//取物
		}
		//如果在路径中，则沿着路径走
		else
		{
			int dir = -1;	
			//X正方向走1格
			if (bot.fetch_good_path[bot.fetch_good_cur + 1].first - bot.fetch_good_path[bot.fetch_good_cur].first > 0)
			{
				dir = 3;	//下移一格
			}
			//X负方向走1格
			else if (bot.fetch_good_path[bot.fetch_good_cur + 1].first - bot.fetch_good_path[bot.fetch_good_cur].first < 0)
			{
				dir = 2;	//上移一格
			}
			//Y正方向走1格
			else if (bot.fetch_good_path[bot.fetch_good_cur + 1].second - bot.fetch_good_path[bot.fetch_good_cur].second > 0)
			{
				dir = 0;	//右移一格
			}
			//Y负方向走1格
			else if (bot.fetch_good_path[bot.fetch_good_cur + 1].second - bot.fetch_good_path[bot.fetch_good_cur].second < 0)
			{
				dir = 1;	//左移一格
			}
			//出现未知情况，即两个路径点一样等情况，那就让机器人不动
			if (-1 == dir || dir > 3)
			{
				res.push_back(Command(-1, bot_id, -1));	//空指令
			}
			//move对应的方向
			else
			{
				res.push_back(Command(1, bot_id, dir));
				//如果当前再走一步就到了目标点，则可以再产生一条get指令
				if (bot.fetch_good_cur == path_size - 2)
				{
					res.push_back(Command(2, bot_id, -1));	//取物
				}
			}
		}
	}
	//送物机器人
	else
	{
		path_size = bot.send_good_path.size();
		//如果取物路径的逻辑指针不合法，则暂时不动
		if (bot.send_good_cur < 0 || bot.send_good_cur >= path_size)
		{
			res.push_back(Command(-1, bot_id, -1));
		}
		//如果当前在路径尽头，则应该送物
		else if (bot.send_good_cur == path_size - 1)
		{
			res.push_back(Command(4, bot_id, -1));	//送物
		}
		//如果在路径中，则沿着路径走
		else
		{
			int dir = -1;
			//X正方向走1格
			if (bot.send_good_path[bot.send_good_cur + 1].first - bot.send_good_path[bot.send_good_cur].first > 0)
			{
				dir = 3;	//下移一格
			}
			//X负方向走1格
			else if (bot.send_good_path[bot.send_good_cur + 1].first - bot.send_good_path[bot.send_good_cur].first < 0)
			{
				dir = 2;	//上移一格
			}
			//Y正方向走1格
			else if (bot.send_good_path[bot.send_good_cur + 1].second - bot.send_good_path[bot.send_good_cur].second > 0)
			{
				dir = 0;	//右移一格
			}
			//Y负方向走1格
			else if (bot.send_good_path[bot.send_good_cur + 1].second - bot.send_good_path[bot.send_good_cur].second < 0)
			{
				dir = 1;	//左移一格
			}
			//出现未知情况，即两个路径点一样等情况，那就让机器人不动
			if (-1 == dir || dir > 3)
			{
				res.push_back(Command(-1, bot_id, -1));	//空指令
			}
			//move对应的方向
			else
			{
				res.push_back(Command(1, bot_id, dir));
				//如果当前再走一步就到了目标点，则可以再产生一条pull指令
				if (bot.fetch_good_cur == path_size - 2)
				{
					res.push_back(Command(4, bot_id, -1));	//送物
				}
			}
		}
	}
	return res;
}

/**
 * makeBoatCmd - 根据输入生成某个轮船当前应进行的指令
 * @bot: 轮船结构体
 * @bot_id: 轮船id
 *
 * 接收两个参数，生成对应轮船当前指令
 *
 * 返回值:
 * 以指令结构体为元素的vector
 *
 * 注意事项/约束条件：
 * 有些时候可能会生成多条指令；
 * 就算轮船不动也会生成key==-1的空指令
 */
vector<Command> MakeDecision::makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths)
{
	vector<Command> res;	//返回值
	//如果在运输中，就保持不动
	if (0 == boat.status)
	{
		res.push_back(Command(-1, boat_id, -1));
		return res;
	}
	//装货或运输完成
	else if (1 == boat.status)
	{

	}


	//有未知错误导致没有泊位，船就不动
	if (0 == berths.size())
	{
		res.push_back(Command(-1, boat_id, -1));
		return res;
	}
	//找到泊位中最大的
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
