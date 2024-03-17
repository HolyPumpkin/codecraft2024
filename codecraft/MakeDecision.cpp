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
int MakeDecision::assignRobotGet(Robot& bot, list<Good>& goods, int cur_frame_id)
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
		if ((abs(i.x - bot.x) + abs(i.y - bot.y)) >= i.end_frame - cur_frame_id)
		{
			continue;
		}
		//找到一个可达且没被指派的货物
		PlanPath planpath(this->maze, this->N, this->n, this->robots);
		Point s = Point(bot.x, bot.y);
		Point e = Point(i.x, i.y);
		// 每次规划路径的时候都要把游标置零
		bot.fetch_good_path = planpath.pathplanning(s, e);
		bot.fetch_good_cur = 0;
		// 路径规划失败，可能是死路
		if (bot.fetch_good_path.empty())
		{
			return -1;
		}
		// 被指派成功要修改内部变量，记录该货物结束时间，机器人当前价值
		i.is_assigned = true;
		bot.good_end_frame = i.end_frame;
		bot.robot_val = i.val;
		// 测试路径规划
		/*cout << " robot (" << bot.x << "," << bot.y << ")'s fetch_path: " << endl;
		planpath.printPath(bot.fetch_good_path);*/

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
	PlanPath planpath(this->maze, this->N, this->n, this->robots);
	Point s = Point(bot.x, bot.y);
	Point e = Point(berths[min_id].x, berths[min_id].y);
	// 每次规划路径的时候都要把游标置零
	bot.send_good_path = planpath.pathplanning(s, e);
	bot.send_good_cur = 0;
	bot.berth_id = min_id;
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
		//如果送物路径的逻辑指针不合法，则暂时不动
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
				if (bot.send_good_cur == path_size - 2)
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
vector<Command> MakeDecision::makeBoatCmd(Boat& boat, int boat_id, vector<Berth>& berths, int frame_id)
{
	vector<Command> res;	//返回值
	if (frame_id == 1)
	{
		res.push_back(Command(8, boat_id, boat_id));	//ship指令
		//正在去这个泊位，此时要把泊位的状态改为被占用，以免其他轮船重复到达
		berths[boat_id].is_occupied = 1;
		return res;
	}

	//得到轮船当前状态
	int cur_status = this->boatStatusCheck(boat);
	
	//如果在运输中，或者在装货中，就保持不动
	if (0 == boat.status || boat.is_loading)
	{
		res.push_back(Command(-1, boat_id, -1));	//空指令
		return res;
	}
	//状态未改变
	if (cur_status == -1)
	{
		if (boat.status == 1)
		{
			//如果到了装货结束的时间，去虚拟点
			if (frame_id >= boat.end_load_frame)
			{
				res.push_back(Command(16, boat_id, -1));	//go指令
				//离开泊位去虚拟点，需要改变泊位状态为未被占用
				if (boat.pos != -1)
				{
					berths[boat.pos].is_occupied = 0;
				}
				boat.is_loading = false;
			}
		}
	}
	//boat的status从0变成1
	if (1 == cur_status)
	{
		//到虚拟点卸货完成
		if (-1 == boat.pos)
		{
			boat.cur_load = 0;
			//找到泊位中货物价值最大的
			int max_val = berths[0].cur_goods_val;
			int max_val_id = 0;
			for (int i = 1; i < berths.size(); ++i)
			{
				if (berths[i].cur_goods_val > max_val && berths[i].is_occupied == 0)
				{
					// TODO 机器人放货物的时候要改变对应泊位的货物价值和货物量
					max_val = berths[i].cur_goods_val;
					max_val_id = i;
				}
			}
			res.push_back(Command(8, boat_id, max_val_id));	//ship指令
			//正在去这个泊位，此时要把泊位的状态改为被占用，以免其他轮船重复到达
			berths[max_val_id].is_occupied = 1;


		}
		//到泊位装货
		else
		{
			boat.is_loading = true;
			res.push_back(Command(-1, boat_id, -1));	//空指令
		}
	}
	//boat的status从1变成0
	else if (0 == cur_status)
	{
		boat.is_loading = false;
		//如果在泊位
		if (boat.pos != -1)
		{
			//去虚拟点
			res.push_back(Command(16, boat_id, -1));	//go指令
			//离开泊位去虚拟点，需要改变泊位状态为未被占用
			if (boat.pos != -1)
			{
				berths[boat.pos].is_occupied = 0;
			}
		}
		else
		{
			//找到泊位中货物价值最大的
			int max_val = berths[0].cur_goods_val;
			int max_val_id = 0;
			for (int i = 0; i < berths.size(); ++i)
			{
				if (berths[i].cur_goods_val > max_val && berths[i].is_occupied == 0)
				{
					// TODO 机器人放货物的时候要改变对应泊位的货物价值和货物量
					max_val = berths[i].cur_goods_val;
					max_val_id = i;
				}
			}
			res.push_back(Command(8, boat_id, max_val_id));	//ship指令
			//正在去这个泊位，此时要把泊位的状态改为被占用，以免其他轮船重复到达
			berths[boat_id].is_occupied = 1;
		}
	}
	//其他情况直接去虚拟点
	else
	{
		boat.is_loading = false;
		res.push_back(Command(16, boat_id, -1));	//go指令
	}
	return res;
}

/**
 * @brief 生成机器人空指令
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
 * @brief 检测每一帧的机器人输入，其中逻辑指针直到此处才有可能被修改，
 *        所以在判断时逻辑指针指向的其实是机器人上一帧的位置。
 * 
 * @note  此函数主要是保证每一帧生成指令之前，游标的位置是正确的
 * 
 * @param robots 
*/
void MakeDecision::robotInputCheck(vector<Robot>& robots, list<Good>& goods, int cur_frame_id)
{
	this->robots = robots;	//每次先更新内部机器人为最新
	for (int rbt_idx = 0; rbt_idx < robots.size(); ++rbt_idx)
	{
		//如果当前机器人是未携带物品的状态
		if (0 == robots[rbt_idx].is_carry && !robots[rbt_idx].fetch_good_path.empty())
		{
			//游标不合法
			/*if (robots[rbt_idx].fetch_good_cur > robots[rbt_idx].fetch_good_path.size() - 1
				|| robots[rbt_idx].fetch_good_cur < 0)
			{
				continue;
			}*/
			// 取出机器人当前的位置和上一帧的位置
			std::pair<int, int> cur_pos(robots[rbt_idx].x, robots[rbt_idx].y);
			std::pair<int, int> pre_pos = robots[rbt_idx].fetch_good_path[robots[rbt_idx].fetch_good_cur];

			//如果他在取物路径逻辑指针所指的位置，即相比于上一帧没动
			//证明该机器人：1.碰撞；2.get；3.pull；4.空指令
			//所以此时不修改游标
			if (cur_pos == pre_pos)
			{
				continue;
			}
			//如果坐标不相等，说明机器人肯定走了，要么沿着路径走了，要么偏离路径
			else if (cur_pos != pre_pos)
			{
				//判断是否沿着路径走
				//如果上一步在终点，则跳过，会给他重新分配send路径
				if (robots[rbt_idx].fetch_good_cur == robots[rbt_idx].fetch_good_path.size() - 1)
				{
					continue;
				}
				//如果上一步在起点，只判断是否在后一步
				else if (robots[rbt_idx].fetch_good_cur == 0)
				{
					//在路径中，则修改逻辑指针
					if (robots[rbt_idx].fetch_good_path[robots[rbt_idx].fetch_good_cur + 1] == cur_pos)
					{
						robots[rbt_idx].fetch_good_cur = robots[rbt_idx].fetch_good_cur + 1;
					}

					//没有在路径中，需要重新规划路径
					else
					{
						//修改指派变量，后续重新规划
						robots[rbt_idx].is_assigned = 0;
					}
				}
				//如果在路径中（不在起点或终点，防止下标越界）
				else if (robots[rbt_idx].fetch_good_cur < robots[rbt_idx].fetch_good_path.size() - 1
			     && robots[rbt_idx].fetch_good_cur > 0)
				{
					//沿着路径走，前一步或者后一步，都把逻辑指针修改
					if (robots[rbt_idx].fetch_good_path[robots[rbt_idx].fetch_good_cur - 1] == cur_pos)
					{
						robots[rbt_idx].fetch_good_cur = robots[rbt_idx].fetch_good_cur - 1;
					}
					else if (robots[rbt_idx].fetch_good_path[robots[rbt_idx].fetch_good_cur + 1] == cur_pos)
					{
						robots[rbt_idx].fetch_good_cur = robots[rbt_idx].fetch_good_cur + 1;
					}

					//没有在路径中，需要重新规划路径
					else
					{
						//修改指派变量，后续重新规划
						robots[rbt_idx].is_assigned = 0;
					}
				}

			}

		}
		//如果当前机器人是携带物品的状态
		else if (1 == robots[rbt_idx].is_carry && !robots[rbt_idx].send_good_path.empty())
		{
			// 取出机器人当前的位置和上一帧的位置
			std::pair<int, int> cur_pos(robots[rbt_idx].x, robots[rbt_idx].y);
			std::pair<int, int> pre_pos = robots[rbt_idx].send_good_path[robots[rbt_idx].send_good_cur];

			//如果他在取物路径逻辑指针所指的位置，即相比于上一帧没动
			//证明该机器人：1.碰撞；2.get；3.pull；4.空指令
			//所以此时不修改游标
			if (cur_pos == pre_pos)
			{
				continue;
			}
			//如果坐标不相等，说明机器人肯定走了，要么沿着路径走了，要么偏离路径
			else if (cur_pos != pre_pos)
			{
				//判断是否沿着路径走
				//如果上一步在终点，则跳过，会给他重新分配send路径
				if (robots[rbt_idx].send_good_cur == robots[rbt_idx].send_good_path.size() - 1)
				{
					continue;
				}
				//如果上一步在起点，只判断是否在后一步
				else if (robots[rbt_idx].send_good_cur == 0)
				{
					//在路径中，则修改逻辑指针
					if (robots[rbt_idx].send_good_path[robots[rbt_idx].send_good_cur + 1] == cur_pos)
					{
						robots[rbt_idx].send_good_cur = robots[rbt_idx].send_good_cur + 1;
					}

					//没有在路径中，需要重新规划路径
					else
					{
						//修改指派变量，后续重新规划
						robots[rbt_idx].is_assigned = 0;
					}
				}
				//如果在路径中（不在起点或终点，防止下标越界）
				else if (robots[rbt_idx].send_good_cur < robots[rbt_idx].send_good_path.size() - 1
					&& robots[rbt_idx].send_good_cur > 0)
				{
					//沿着路径走，前一步或者后一步，都把逻辑指针修改
					if (robots[rbt_idx].send_good_path[robots[rbt_idx].send_good_cur - 1] == cur_pos)
					{
						robots[rbt_idx].send_good_cur = robots[rbt_idx].send_good_cur - 1;
					}
					else if (robots[rbt_idx].send_good_path[robots[rbt_idx].send_good_cur + 1] == cur_pos)
					{
						robots[rbt_idx].send_good_cur = robots[rbt_idx].send_good_cur + 1;
					}

					//没有在路径中，需要重新规划路径
					else
					{
						//修改指派变量，后续重新规划
						robots[rbt_idx].is_assigned = 0;
					}
				}

			}
		}
	}
}

/**
 * @brief 根据每一帧输入的信息判断轮船的状态
 * @param boat 
 * @return 整数值，不同的值分别对应以下几种状态
 * -1：boat的status没变化
 * 0：boat的status从1变成0
 * 1：boat的status从0变成1
 * 2：boat的status从2变成1
 * 3：boat的status从2变成0
 * 4：boat的status从0变成2
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
 * @brief 根据每一帧的输入更新轮船信息
 * @param boats 
*/
void MakeDecision::boatInputCheck(vector<Boat>& boats, int frame_id)
{
	for (int i = 0; i < boats.size(); ++i)
	{
		// 如果当前帧大于等于结束装货的帧，就修改is_loading，表示装货完成
		if (frame_id >= boats[i].end_load_frame)
		{
			boats[i].is_loading = false;
		}
		// 到虚拟点后要把物品清空
		if (boats[i].pos == -1)
		{
			boats[i].cur_load = 0;
		}
		// 得到轮船当前的状态
		int boat_status = this->boatStatusCheck(boats[i]);
		if (boat_status == 1 || boat_status == 2)
		{
			boats[i].start_load_frame = frame_id;
			boats[i].end_load_frame = frame_id + boats[i].loading_time;
		}

	}
}

/**
 * @brief 迁移轮船的状态信息
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
 * @brief 单个机器人重启。
 * @param robot 
 */
void MakeDecision::robotReboot(Robot& robot)
{
	// 取消指派状态
	robot.is_assigned = 0;

	// 清除路径信息
	robot.fetch_good_cur = 0;
	robot.send_good_cur = 0;
	robot.fetch_good_path.clear();
	robot.send_good_path.clear();

	if (robot.is_carry == 0)		// 如果机器人未携带物品
	{
		robot.last_is_carry = 1;
		robot.good_end_frame = -1;
		robot.robot_val = 0;
	}
	else if (robot.is_carry == 1)	// 如果机器人携带物品
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

