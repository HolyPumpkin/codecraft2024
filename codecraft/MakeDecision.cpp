/*
 * Change Logs:
 * Date           Author       Notes
 * 2024-3-13     Haoyu Nan     the first version
 */

#include "MakeDecision.h"


 /**
  * MakeDecision - 根据输入决策类实例
  * @_maze: 二维字符地图
  * @_N: 构造地图大小
  * @_n: 实际地图大小
  *
  * 接收三个参数，构造出MakeDecision类实例
  *
  * 返回值:
  * 无
  *
  * 注意事项/约束条件：
  * 无
  */
MakeDecision::MakeDecision(vector<vector<char>>& _maze, int _N, int _n)
{
	this->maze = _maze;
	this->N = _N;
	this->n = _n;
}

/**
   * assignRobotGet - 根据输入指派某个机器人去取某个货物
   * @bot: 机器人结构体
   * @goods: 货物结构体集合
   *
   * 接收两个参数，会生成取物路径，存在机器人结构体内部
   *
   * 返回值:
   * 整型值，0表示指派成功，-1表示指派失败（可能无货物）
   *
   * 注意事项/约束条件：
   * 指派失败的情况可能有多种，后续完善
   */
int MakeDecision::assignRobotGet(Robot& bot, vector<Good>& goods)
{
	//无货物
	if (goods.empty())
	{
		return -1;
	}
	//当前策略是，发现一个可达货物就去，不看距离
	for (auto& i : goods)
	{
		//已经被指派给一个机器人
		if (i.is_assigned)
		{
			continue;
		}
		//不可达，即货物在机器人到之前就消失
		if ((abs(i.x - bot.x) + abs(i.y - bot.y)) >= i.ttl)
		{
			continue;
		}
		//找到一个可达且没被指派的货物
		PlanPath planpath(this->maze, this->N, this->n);
		Point s = Point(bot.x, bot.y);
		Point e = Point(i.x, i.y);
		bot.fetch_good_path = planpath.pathplanning(s, e);
		return 0;
	}
	//如果未在循环内return则表示没有合适的good指派，失败
	return -1;
}

/**
   * assignRobotSend - 根据输入指派某个机器人去送某个货物
   * @bot: 机器人结构体
   * @berths: 泊位结构体集合
   *
   * 接收两个参数，会生成送物路径，存在机器人结构体内部
   *
   * 返回值:
   * 整型值，0表示指派成功，-1表示指派失败
   *
   * 注意事项/约束条件：
   * 指派失败的情况可能有多种，后续完善
   */
int MakeDecision::assighRobotSend(Robot& bot, vector<Berth>& berths)
{
	//无泊位
	if (berths.empty())
	{
		return -1;
	}
	//泊位可以容纳多个机器人，直接挑一个最近的
	int min_dis = INT_MAX;
	int min_id = 0;	//默认去0
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
	//规划送物路径并存入机器人结构体
	PlanPath planpath(this->maze, this->N, this->n);
	Point s = Point(bot.x, bot.y);
	Point e = Point(berths[min_id].x, berths[min_id].y);
	bot.send_good_path = planpath.pathplanning(s, e);

	return 0;
}



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
	//如果在运输中，或者在装货中，就保持不动
	if (0 == boat.status || boat.is_loading)
	{
		res.push_back(Command(-1, boat_id, -1));	//空指令
		return res;
	}
	//在泊位装货完成，go去虚拟点
	else if (1 == boat.status && boat.pos != -1)
	{
		res.push_back(Command(16, boat_id, -1));	//go指令
	}
	/* 其余情况都需要指定一个泊位然后过去 */
	//有未知错误导致没有泊位，船就不动
	if (0 == berths.size())
	{
		res.push_back(Command(-1, boat_id, -1));
		return res;
	}
	//找到泊位中货物价值最大的
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
	res.push_back(Command(8, boat_id, max_val_id));	//ship指令

	return res;
}
