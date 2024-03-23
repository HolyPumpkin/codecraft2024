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
	this->count = 0;
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
int MakeDecision::assignRobotGet(Robot& bot, int robot_id, list<Good>& goods, int cur_frame_id, vector<Berth> berths)
{
	//无货物
	if (goods.empty())
	{
		return -1;
	}
	//死机器人
	if (bot.berth_id == -1)
	{
		return -1;
	}
	// 取第一个货物作为基准
	Good* min_dist_gd;
	min_dist_gd = &goods.front();

	//原逻辑
	//int min_dist = abs(min_dist_gd->x - bot.x) + abs(min_dist_gd->y - bot.y);
	// 
	//现逻辑，取离机器人绑定的泊位最近的
	int min_dist = berths[bot.berth_id].reachable_map[min_dist_gd->x][min_dist_gd->y].dist;

	//定义货物的运价系数，表示对一个货物的整体价值评估，考虑了实际距离和实际价值双重因素
	//float w_gd = 0.0;	//货物权重
	//float w_dist = 1.0;	//距离权重
	//float a_dist = -0.005;	//距离的映射参数，将距离从负相关变为正相关
	//float max_dist_val = w_gd * min_dist_gd->val / 200 + w_dist * exp(a_dist * min_dist);	//运价系数公式

	// 遍历所有货物，先找距离泊位最近的
	for (auto& gd : goods)
	{
		// true表示可达，false表示不可达，如果当前货物不可达，就不去
		if (!bot.reachable_map[gd.x][gd.y])
		{
			continue;
		}
		if (gd.is_assigned)
		{
			continue;
		}
		if (gd.is_ungettable[robot_id])
		{
			continue;
		}
		//原逻辑，取离机器人最近的
		//int dist = abs(gd.x - bot.x) + abs(gd.y - bot.y);
		
		//现逻辑，取离机器人绑定的泊位最近的
		int dist = berths[bot.berth_id].reachable_map[gd.x][gd.y].dist;

		//如果货物在到达之前会消失，就不去取
		if (dist >= (gd.end_frame - cur_frame_id))
		{
			continue;
		}

		//计算货物的运价系数，表示对一个货物的整体价值评估，考虑了实际距离和实际价值双重因素
		//float dist_val = w_gd * gd.val / 200 + w_dist * exp(a_dist * dist);	//运价系数公式

		////现逻辑，去取运价系数最高的
		//if (max_dist_val < dist_val)
		//{
		//	min_dist_gd = &gd;
		//	max_dist_val = dist_val;
		//}

		//原逻辑，取最近的
		if (min_dist > dist)
		{
			min_dist_gd = &gd;
			min_dist = dist;
		}

		//如果初始选的最小距离货物已经被指派，则切换
		if (min_dist_gd->is_assigned)
		{
			min_dist_gd = &gd;
			min_dist = dist;
		}
	}
	//循环之后，min_dist_gd就是离当前泊位最近的货物
	//定义一个区间上限
	int max_dist = int(min_dist * 1.5);
	// 遍历所有货物，在min_dist和max_dist的范围间寻找价值最大的
	for (auto& gd : goods)
	{
		// true表示可达，false表示不可达，如果当前货物不可达，就不去
		if (!bot.reachable_map[gd.x][gd.y])
		{
			continue;
		}
		if (gd.is_assigned)
		{
			continue;
		}
		if (gd.is_ungettable[robot_id])
		{
			continue;
		}
		//现逻辑，取离机器人绑定的泊位最近的
		int dist = berths[bot.berth_id].reachable_map[gd.x][gd.y].dist;
		//如果货物在到达之前会消失，就不去取
		if (dist >= (gd.end_frame - cur_frame_id))
		{
			continue;
		}
		//如果货物距离大于最大区间值，则不取
		if (dist > max_dist)
		{
			continue;
		}
		//寻找价值最大的
		if (gd.val > min_dist_gd->val)
		{
			min_dist_gd = &gd;
		}

		//如果初始选的最小距离货物已经被指派，则切换
		if (min_dist_gd->is_assigned)
		{
			min_dist_gd = &gd;
			min_dist = dist;
		}
	}

	//如果最后找到的已经被指派，则返回-1
	if (min_dist_gd->is_assigned)
	{
		return -1;
	}
	// 找到一个可达且没被指派的货物
	PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	Point s = Point(bot.x, bot.y);
	Point e = Point(min_dist_gd->x, min_dist_gd->y);
	this->count++;
	// 每次规划路径的时候都要把游标置零
	bot.fetch_good_path = planpath.pathplanning(s, e);
	bot.fetch_good_cur = 0;

	// 路径规划失败，可能是死路，这时候要把这个货物设为对该机器人不可达
	if (bot.fetch_good_path.empty())
	{
		min_dist_gd->is_ungettable[robot_id] = true;
		return -1;
	}

	// 被指派成功要修改内部变量，记录该货物结束时间，机器人当前价值
	min_dist_gd->is_assigned = true;
	bot.good_end_frame = min_dist_gd->end_frame;
	bot.robot_val = min_dist_gd->val;

	return 0;
	
	//// 原策略是，发现一个可达货物就去，不看距离
	//for (auto& i : goods)
	//{
	//	//已经被指派给一个机器人
	//	if (i.is_assigned)
	//	{
	//		continue;
	//	}
	//	//不可达，即货物在机器人到之前就消失
	//	if ((abs(i.x - bot.x) + abs(i.y - bot.y)) >= i.end_frame - cur_frame_id)
	//	{
	//		continue;
	//	}
	//	//找到一个可达且没被指派的货物
	//	PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	//	Point s = Point(bot.x, bot.y);
	//	Point e = Point(i.x, i.y);
	//	// 每次规划路径的时候都要把游标置零
	//	bot.fetch_good_path = planpath.pathplanning(s, e);
	//	bot.fetch_good_cur = 0;
	//	// 路径规划失败，可能是死路，这时候要把这个货物去掉，以免后续无效搜索
	//	if (bot.fetch_good_path.empty())
	//	{
	//		i.is_assigned = true;
	//		return -1;
	//	}
	//	// 被指派成功要修改内部变量，记录该货物结束时间，机器人当前价值
	//	i.is_assigned = true;
	//	bot.good_end_frame = i.end_frame;
	//	bot.robot_val = i.val;
	//	// 测试路径规划
	//	/*cout << " robot (" << bot.x << "," << bot.y << ")'s fetch_path: " << endl;
	//	planpath.printPath(bot.fetch_good_path);*/

	//	return 0;
	//}
	////如果未在循环内return则表示没有合适的good指派，失败
	//return -1;
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
int MakeDecision::assignRobotSend(Robot& bot, vector<Berth>& berths)
{
	// 如果机器人未分配，其berth_id为-1，可能有越界问题
	// 如果berth_id为-1，表示是个死机器人，就输出一个空指令
	if (bot.berth_id <= -1)
	{
		return -1;
	}
	// 无泊位
	if (berths.empty())
	{
		return -1;
	}
	
	int target_x = berths[bot.berth_id].x + rand() % 4;
	int target_y = berths[bot.berth_id].y + rand() % 4;
	PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	Point s = Point(bot.x, bot.y);
	Point e = Point(target_x, target_y);
	this->count++;
	// 每次规划路径的时候都要把游标置零
	bot.send_good_path = planpath.pathplanning(s, e);
	bot.send_good_cur = 0;

	// 如果不可达
	if (bot.send_good_path.empty())
	{
		bot.is_ungettable[bot.berth_id] = 0;
		return -1;
	}
	return 0;


	 // 原版本=================================================================
	 
	//无泊位
	//if (berths.empty())
	//{
	//	return -1;
	//}
	////泊位可以容纳多个机器人，直接挑一个最近的
	//int min_dis = INT_MAX;
	//int min_id = 0;	//默认去0
	//int temp = 0;
	//for (int i = 0; i < berths.size(); ++i)
	//{
	//	//当前泊位不可达
	//	if (bot.is_ungettable[berths[i].id] == 0)
	//	{
	//		continue;
	//	}
	//	temp = abs(berths[i].x - bot.x) + abs(berths[i].y - bot.y);
	//	if (temp < min_dis)
	//	{
	//		min_dis = temp;
	//		min_id = i;
	//	}
	//	// 当前离得最近的这个泊位不可达，就要改变泊位
	//	if (bot.is_ungettable[berths[min_id].id] == 0)
	//	{
	//		min_id = i;
	//		min_dis = temp;
	//	}
	//}
	////规划送物路径并存入机器人结构体
	////泊位是4*4的，去任意一个点都行，所以就去随机的一个点
	//int target_x = berths[min_id].x + rand() % 4;
	//int target_y = berths[min_id].y + rand() % 4;
	//PlanPath planpath(this->maze, this->N, this->n, this->own_robots);
	//Point s = Point(bot.x, bot.y);
	//Point e = Point(target_x, target_y);
	//// 每次规划路径的时候都要把游标置零
	//bot.send_good_path = planpath.pathplanning(s, e);
	////如果规划的路径不可达，要设置一下，以免机器人下次还去这个泊位
	//if (bot.send_good_path.empty())
	//{
	//	bot.is_ungettable[berths[min_id].id] = 0;
	//	return -1;
	//}
	//bot.send_good_cur = 0;
	//bot.berth_id = min_id;
	//return 0;
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
	//初始化轮船位置
	if (frame_id == 1)
	{
		//找到一个对应的泊位
		int init_berth_id = -1;
		int min_time = 20000;
		for (int i = 0; i < berths.size(); ++i)
		{
			// 在已经分配好的泊位中挑往返时间最短的
			if (berths[i].is_occupied == 0 && berths[i].rbt_seq.size() > 0)
			{
				if (berths[i].transport_time < min_time)
				{
					init_berth_id = i;
					min_time = berths[i].transport_time;
				}
			}
		}
		if (init_berth_id == -1)
		{
			for (int i = 0; i < berths.size(); ++i)
			{
				// 在已经分配好的泊位中挑货物最多的
				// todo不是均匀分配
				if (berths[i].rbt_seq.size() > 0)
				{
					// 机器人放货物的时候要改变对应泊位的货物价值和货物量
					init_berth_id = i;
					break;
				}
			}
		}
		// 如果再分配还是没有泊位，那就去0号泊位
		if (init_berth_id == -1)
		{
			init_berth_id = 0;
		}
		res.push_back(Command(8, boat_id, init_berth_id));	//ship指令
		//正在去这个泊位，此时要把泊位的状态改为被占用，以免其他轮船重复到达
		boat.berth_id = init_berth_id;
		berths[init_berth_id].is_occupied = 1;
		return res;
	}

	//如果剩余时间不够了，就让轮船进行最后一舞！
	if(boat.pos != -1)
	{
		//留10帧作为空闲量，保证船最后能到虚拟点
		if (15000 - frame_id < berths[boat.pos].transport_time + 10)
		{
			res.push_back(this->lastDance(boat_id));
			return res;
		}
	}

	//得到轮船当前状态
	int cur_status = this->boatStatusCheck(boat);
	
	//如果在运输中，或者在装货中，就保持不动
	if (0 == boat.status)
	{
		res.push_back(Command(-1, boat_id, -1));	//空指令
		return res;
	}
	//如果轮船正在装货
	if (boat.is_loading)
	{
		// todo loading_time可能不准
		boat.loading_time++;
		// 如果轮船在某个泊位装货
		if (boat.pos > -1)
		{
			// todo按轮船capacity的百分比或者机器人运力计算，泊位间移动是500帧，需要保证能多500帧的货物
			// 调参
			int loss = 13;	//500帧会损失的货物，需要大于这个才去
			int temp_goods_num = berths[boat.pos].cur_goods_num;
			int temp_loading_speed = berths[boat.pos].loading_speed;
			// 如果当前泊位有货，则装货并维护信息
			if (berths[boat.pos].cur_goods_num > 0)
			{
				int temp_sum = (temp_goods_num > temp_loading_speed) ? temp_loading_speed : temp_goods_num;
				boat.cur_load += temp_sum;
				berths[boat.pos].cur_goods_num -= temp_sum;
			}
			// 如果当前泊位装完了，但是轮船没满
			else if (berths[boat.pos].cur_goods_num <= 0)
			{
				if(boat.cur_load < boat.capacity - loss)
				{
					int fast_bth_id = -1;
					int min_time = 20000;
					for (auto& berth : berths)
					{
						if (berth.is_occupied == 0 && berth.cur_goods_num > loss)
						{
							if (berth.transport_time < min_time)
							{
								fast_bth_id = berth.id;
								min_time = berth.transport_time;
							}
						}
					}
					if (fast_bth_id != -1)
					{
						berths[fast_bth_id].is_occupied = 1;
						berths[boat.pos].is_occupied = 0;
						boat.is_loading = false;
						res.push_back(Command(8, boat_id, fast_bth_id));
						return res;
					}
				}
				//如果轮船近乎于满，就让它去送货
				else if (boat.cur_load >= boat.capacity - loss)
				{
					res.push_back(Command(16, boat_id, -1));	//go指令
					//离开泊位去虚拟点，需要改变泊位状态为未被占用
					if (boat.pos != -1)
					{
						berths[boat.pos].is_occupied = 0;
					}
					boat.is_loading = false;
					boat.loading_time = 0;
				}
			}

		}
		//res.push_back(Command(-1, boat_id, -1));	//空指令
		//return res;
	}

	//状态未改变
	if (cur_status == -1)
	{
		if (boat.status == 1)
		{
			// todo如果轮船装满了，再去虚拟点
			if (boat.cur_load >= boat.capacity)
			{
				res.push_back(Command(16, boat_id, -1));	//go指令
				//离开泊位去虚拟点，需要改变泊位状态为未被占用
				if (boat.pos != -1)
				{
					berths[boat.pos].is_occupied = 0;
				}
				boat.is_loading = false;
				boat.loading_time = 0;
			}

			//原逻辑
			////如果到了装货结束的时间，去虚拟点
			//if (frame_id >= boat.end_load_frame)
			//{
			//	res.push_back(Command(16, boat_id, -1));	//go指令
			//	//离开泊位去虚拟点，需要改变泊位状态为未被占用
			//	if (boat.pos != -1)
			//	{
			//		berths[boat.pos].is_occupied = 0;

			//		//轮船离开要更新泊位货物数量
			//		int temp_value = berths[boat.pos].cur_goods_num - berths[boat.pos].loading_speed * boat.loading_time;
			//		berths[boat.pos].cur_goods_num = (temp_value > 0) ? temp_value : 0;
			//	}
			//	boat.is_loading = false;
			//}
		}
	}
	//boat的status从0变成1或者从2变成1
	else if (1 == cur_status || 2 == cur_status)
	{
		//到虚拟点卸货完成，去泊位
		if (-1 == boat.pos)
		{
			//选一个最优的泊位去
			boat.cur_load = 0;
			int fast_bth_id = -1;
			int min_time = 20000;
			for (auto& berth : berths)
			{
				if (berth.is_occupied == 0)
				{
					if (berth.transport_time < min_time)
					{
						fast_bth_id = berth.id;
						min_time = berth.transport_time;
					}
				}
			}
			if (fast_bth_id != -1)
			{
				berths[fast_bth_id].is_occupied = 1;
				res.push_back(Command(8, boat_id, fast_bth_id));
			}
			//如果没找到，就去本来对应的泊位
			else
			{
				res.push_back(Command(8, boat_id, boat.berth_id));
				berths[boat.berth_id].is_occupied = 1;
			}

			//// 2024.3.22晚22：53之前的逻辑：轮船去固定的泊位，默认0号
			//boat.cur_load = 0;
			//res.push_back(Command(8, boat_id, boat.berth_id));
			//berths[boat.berth_id].is_occupied = 1;

			//原逻辑：轮船选泊位
			// boat.cur_load = 0;
			////找到泊位中货物数量最大的
			//int max_val = berths[0].cur_goods_num;
			//int max_val_id = 0;
			//for (int i = 1; i < berths.size(); ++i)
			//{
			//	// 在已经分配好的泊位中挑货物最多的
			//	if (berths[i].is_occupied == 0 && berths[i].rbt_seq.size() > 0 && berths[i].cur_goods_num >= max_val)
			//	{
			//		// 机器人放货物的时候要改变对应泊位的货物价值和货物量
			//		max_val = berths[i].cur_goods_num;
			//		max_val_id = i;
			//	}
			//}
			//res.push_back(Command(8, boat_id, max_val_id));	//ship指令
			////正在去这个泊位，此时要把泊位的状态改为被占用，以免其他轮船重复到达
			//berths[max_val_id].is_occupied = 1;
			////动态改变当前轮船的装货时间，根据要去的泊位货物数量和轮船容量、装载速度判断
			//if (berths[max_val_id].cur_goods_num >= boat.capacity)
			//{
			//	boat.loading_time = boat.capacity / berths[max_val_id].loading_speed;
			//}
			//else if (berths[max_val_id].cur_goods_num < boat.capacity)
			//{
			//	boat.loading_time = berths[max_val_id].cur_goods_num / berths[max_val_id].loading_speed;
			//}

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
			//res.push_back(Command(16, boat_id, -1));	//go指令
			//离开泊位去虚拟点，需要改变泊位状态为未被占用
			if (boat.pos != -1)
			{
				berths[boat.pos].is_occupied = 0;

				////轮船离开要更新泊位货物数量
				// todo逻辑有问题
				//int temp_value = berths[boat.pos].cur_goods_num - berths[boat.pos].loading_speed * boat.loading_time;
				//berths[boat.pos].cur_goods_num = (temp_value > 0) ? temp_value : 0;
			}
		}
		else
		{
			// 如果在虚拟点发生从1到0，则说明轮船已经在去泊位的路上了，空指令
			res.push_back(Command(-1, boat_id, -1));	//空指令

			////找到泊位中货物数量最大的
			//int max_val = berths[0].cur_goods_num;
			//int max_val_id = 0;
			//for (int i = 1; i < berths.size(); ++i)
			//{
			//	if (berths[i].cur_goods_num > max_val && berths[i].is_occupied == 0)
			//	{
			//		// 机器人放货物的时候要改变对应泊位的货物价值和货物量
			//		max_val = berths[i].cur_goods_num;
			//		max_val_id = i;
			//	}
			//}
			//res.push_back(Command(8, boat_id, max_val_id));	//ship指令
			////正在去这个泊位，此时要把泊位的状态改为被占用，以免其他轮船重复到达
			//berths[boat_id].is_occupied = 1;

			////动态改变当前轮船的装货时间，根据要去的泊位货物数量和轮船容量、装载速度判断
			//if (berths[max_val_id].cur_goods_num >= boat.capacity)
			//{
			//	boat.loading_time = boat.capacity / berths[max_val_id].loading_speed;
			//}
			//else if (berths[max_val_id].cur_goods_num < boat.capacity)
			//{
			//	boat.loading_time = berths[max_val_id].cur_goods_num / berths[max_val_id].loading_speed;
			//}

		}
	}
	//其他情况直接去虚拟点
	else
	{
		boat.is_loading = false;
		// 生成空指令
		res.push_back(Command(-1, boat_id, -1));	//空指令
		//res.push_back(Command(16, boat_id, -1));	//go指令
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
	this->own_robots = robots;	//每次先更新内部机器人为最新
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
						robots[rbt_idx].is_assigned = 2;
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
						robots[rbt_idx].is_assigned = 2;
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
						robots[rbt_idx].is_assigned = 2;
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
						robots[rbt_idx].is_assigned = 2;
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
		/*if (frame_id >= boats[i].end_load_frame)
		{
			boats[i].is_loading = false;
		}*/
		// 到虚拟点后要把物品清空
		//if (boats[i].pos == -1)
		//{
		//	boats[i].cur_load = 0;
		//}
		//// 得到轮船当前的状态
		//int boat_status = this->boatStatusCheck(boats[i]);
		//if (boat_status == 1 || boat_status == 2)
		//{
		//	boats[i].start_load_frame = frame_id;
		//	boats[i].end_load_frame = frame_id + boats[i].loading_time;
		//}

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
	robot.is_assigned = 2;

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
		//robot.berth_id = -1;
	}
}

/**
 * @brief 重启所有应该被重启的机器人（重设一些状态）
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
 * @brief 机器人状态迁移
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
 * @brief 根据当前帧数对应该消失的货物进行处理
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
 * @brief 封装每一帧对机器人的操作，指派、生成指令等
 * @param robots 
 * @param goods 
 * @param berths 
 * @param frame_id 
*/
void MakeDecision::robotsOperate(vector<Robot>& robots, int robot_num, vector<vector<Command>>& robot_cmd, list<Good>& goods, vector<Berth>& berths, int frame_id)
{
	for (int rbt_idx = 0; rbt_idx < robot_num; ++rbt_idx)
	{
		/* 重构部分 */
		// is_carry发生改变，表示拿到或者放下了货物，此时要修改is_assigned的值为0，表示要重新指派
		if (robots[rbt_idx].is_carry == robots[rbt_idx].last_is_carry)
		{
			// 如果is_carry从1变成0，表示在泊位放下了货物，要更改对应泊位的信息
			if (0 == robots[rbt_idx].is_carry)
			{
				// 如果机器人未分配，其berth_id为-1，可能有越界问题
				// 如果berth_id为-1，表示是个死机器人，就输出一个空指令
				if (robots[rbt_idx].berth_id == -1)
				{
					robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
					continue;
				}
				berths[robots[rbt_idx].berth_id].cur_goods_num++;
				berths[robots[rbt_idx].berth_id].cur_goods_val += robots[rbt_idx].robot_val;
			}
			// 正常取货或者送货变为未指派状态
			robots[rbt_idx].is_assigned = 0;
			// 保证孪生变量与is_carry始终不同
			robots[rbt_idx].last_is_carry = robots[rbt_idx].last_is_carry ^ 1;
		}

		// 如果当前机器人未携带东西
		if (0 == robots[rbt_idx].is_carry)
		{
			// 如果当前机器人已经被指派
			if (1 == robots[rbt_idx].is_assigned)
			{
				// 判断货物的存活时间,如果此时货物已经消失
				if (frame_id >= robots[rbt_idx].good_end_frame)
				{
					robots[rbt_idx].is_assigned = 0;
					// 先指派机器人去拿货物
					int assign_success = this->assignRobotGet(robots[rbt_idx], rbt_idx, goods, frame_id,berths);

					// 如果指派失败，插入空指令，表示不做任何操作
					if (-1 == assign_success)
					{
						robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
						continue;
					}
					// 指派成功
					else if (0 == assign_success) {
						robots[rbt_idx].is_assigned = 1;	// 修改内部变量
					}
				}
				// 直接生成指令
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
			// 如果当前机器人未被指派
			else if (0 == robots[rbt_idx].is_assigned)
			{
				// 先指派机器人去拿货物
				int assign_success = this->assignRobotGet(robots[rbt_idx], rbt_idx, goods, frame_id, berths);

				// 如果指派失败，插入空指令，表示不做任何操作
				if (-1 == assign_success)
				{
					robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
					continue;
				}
				// 指派成功
				else if (0 == assign_success) {
					robots[rbt_idx].is_assigned = 1;	// 修改内部变量
				}
				// 生成指令
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
			else if (2 == robots[rbt_idx].is_assigned)
			{
				// 先指派机器人去拿货物
				int assign_success = this->assignRobotGet(robots[rbt_idx], rbt_idx, goods, frame_id, berths);

				// 如果指派失败，插入空指令，表示不做任何操作
				if (-1 == assign_success)
				{
					robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
					continue;
				}
				// 指派成功
				else if (0 == assign_success) {
					robots[rbt_idx].is_assigned = 1;	// 修改内部变量
				}
				// 生成指令
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
			else
			{
				robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
				continue;
			}
		}
		// 如果当前机器人携带东西
		else if (1 == robots[rbt_idx].is_carry)
		{
			// 如果当前机器人已经被指派
			if (1 == robots[rbt_idx].is_assigned)
			{
				// 直接生成指令
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
			// 如果当前机器人正常取货后变成未被指派
			else if (0 == robots[rbt_idx].is_assigned)
			{
				// 先指派机器人去拿货物
				// todo，调用makePathToBerth生成去泊位的路径
				int assign_success = this->assignRobotSendBFS(robots[rbt_idx], berths);

				// 如果指派失败，插入空指令，表示不做任何操作
				if (-1 == assign_success)
				{
					robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
					continue;
				}
				// 指派成功
				else if (0 == assign_success) {
					robots[rbt_idx].is_assigned = 1;	// 修改内部变量
				}
				// 生成指令
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
			// 如果当前机器人因为碰撞后变成未被指派
			else if (2 == robots[rbt_idx].is_assigned)
			{
				// 先指派机器人去拿货物，调用A*算法，规避恢复状态的机器人
				int assign_success = this->assignRobotSend(robots[rbt_idx], berths);

				// 如果指派失败，插入空指令，表示不做任何操作
				if (-1 == assign_success)
				{
					robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
					continue;
				}
				// 指派成功
				else if (0 == assign_success) {
					robots[rbt_idx].is_assigned = 1;	// 修改内部变量
				}
				// 生成指令
				robot_cmd[rbt_idx] = this->makeRobotCmd(robots[rbt_idx], rbt_idx);
			}
			else
			{
				robot_cmd[rbt_idx] = this->makeNullCmd(rbt_idx);
				continue;
			}
		}
	}
}

/**
 * @brief 封装每一帧对轮船的操作，生成指令等
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
 * @brief 轮船的最后一次送货
 * @param boat 
 * @return 指令数组
*/
Command MakeDecision::lastDance(int boat_id)
{
	return Command(16, boat_id, -1);	//go指令
}

/**
 * @brief 选取可用的泊位，划分区域，该函数结束后，所有活机器人都已分配泊位，死机器人berth_id为-1
 * @param berths 
 * @param robots 
*/
void MakeDecision::chooseBerths(vector<Berth>& berths, vector<Robot>& robots, int frame_id)
{
	if (frame_id > 1)
	{
		return;
	}

	PlanPath planpath(this->maze, this->N, this->n, this->own_robots);

	for (auto& bot : robots)
	{
		for (auto& bth : berths)
		{
			//如果当前泊位分配的机器人小于2个，则可正常分配
			//调参todo 参数设为1则表示平均分给10个泊位，2则表示平均分给5个泊位
			if (bth.rbt_seq.size() < 1)
			{
				int target_x = bth.x + rand() % 4;
				int target_y = bth.y + rand() % 4;
				Point s = Point(bot.x, bot.y);
				Point e = Point(target_x, target_y);
				// 进行一次路径规划
				bot.send_good_path = planpath.pathplanning(s, e);
				// 如果规划的路径不可达，设置当前泊位为该机器人不可达的泊位，表示分配失败，尝试下一个泊位
				if (bot.send_good_path.empty())
				{
					bot.is_ungettable[bth.id] = 0;
					continue;
				}
				// 如果规划的路径可达，就把当前机器人加入当前泊位的rbt_seq，表示机器人与泊位绑定，设置机器人的berth_id为当前泊位id
				else
				{
					bth.rbt_seq.push_back(bot.id);
					bot.berth_id = bth.id;
					break;
				}
			}
			//如果当前泊位分配的机器人大于等于2个，则暂时不分配
			else
			{
				continue;
			}
		}
	}
	// 分配完后检查机器人是否都被分配
	for (auto& bot : robots)
	{
		// 如果已经被分配，则无需再分配
		if (bot.berth_id != -1)
		{
			continue;
		}
		// 如果未被分配，则需要再分配
		else
		{
			// 遍历一遍泊位，有可达的就加进去
			for (auto& bth : berths)
			{
				if (bot.is_ungettable[bth.id] == 0)
				{
					continue;
				}
				int target_x = bth.x + rand() % 4;
				int target_y = bth.y + rand() % 4;
				Point s = Point(bot.x, bot.y);
				Point e = Point(target_x, target_y);
				bot.send_good_path = planpath.pathplanning(s, e);
				// 如果不可达
				if (bot.send_good_path.empty())
				{
					bot.is_ungettable[bth.id] = 0;
					continue;
				}
				// 如果可达
				// todo均匀分配
				else
				{
					bth.rbt_seq.push_back(bot.id);
					bot.berth_id = bth.id;
					break;
				}
			}
			// 如果遍历完所有泊位，机器人还未被分配，那么它就是一个死机器人
			// todo死机器人在生成指令时处理
			if (bot.berth_id == -1)
			{
				continue;
			}
		}
	}
}

/**
 * @brief 计算每个机器人可达的区域并存在机器人结构体中
 * @param robots 
*/
void MakeDecision::calRobotReachableMap(vector<Robot>& robots, int frame_id)
{
	if (frame_id > 1)
	{
		return;
	}
	int dir[4][2] = { {1,0},{0,1},{-1,0},{0,-1} };
	// 每个机器人计算
	for (auto& bot : robots)
	{
		// 先把可达数组初始化
		for (int i = 0; i < 200; ++i)
		{
			vector<bool> temp(200, false);
			bot.reachable_map.push_back(temp);
		}
		// 广度优先搜索
		vector<vector<bool>> visited(200, vector<bool>(200, false));
		queue<pair<int,int>> q;
		q.push({ bot.x,bot.y });
		while (!q.empty())
		{
			pair<int, int> cur = q.front();
			q.pop();
			int x = cur.first;
			int y = cur.second;
			bot.reachable_map[x][y] = true;
			visited[x][y] = true;
			for (int i = 0; i < 4; ++i)
			{
				int nx = x + dir[i][0];
				int ny = y + dir[i][1];
				if (nx < 0 || nx >= 200 || ny < 0 || ny >= 200 || visited[nx][ny] || this->maze[nx][ny] == '*' || this->maze[nx][ny] == '#')
				{
					continue;
				}
				q.push({ nx,ny });
				visited[nx][ny] = true;
			}
			
		}
	}

}

/**
 * @brief 计算每个泊位可达的区域及其信息并存在泊位结构体中
 * @param berths 
 * @param frame_id 
*/
void MakeDecision::calBerthReachableMap(vector<Berth>& berths, int frame_id)
{
	if (frame_id > 1)
	{
		return;
	}
	int dir[4][2] = { {0,1},{0,-1},{-1,0},{1,0} };
	// 反方向映射，如果广搜时向左走，实际上去泊位的方向是向右走
	map<pair<int, int>, int> anti_dir = {
		{{0,1},1},
		{{0,-1},0},
		{{-1,0},3},
		{{1,0},2}
	};
	// 对每个泊位
	for (auto& bth : berths)
	{
		// 先初始化可达地图
		for (int i = 0; i < 200; ++i)
		{
			vector<BerthPoint> temp(200);
			bth.reachable_map.push_back(temp);
		}

		// 选择初始点
		int sx = bth.x + 1;
		int sy = bth.y + 1;

		// 定义是否遍历的数组
		vector<vector<bool>> visited(200, vector<bool>(200, false));

		// 广度优先搜索
		queue<pair<int, int>> q;
		// 压入初始点
		q.push({ sx,sy });
		visited[sx][sy] = true;
		bth.reachable_map[sx][sy].dir = -1;
		bth.reachable_map[sx][sy].dist = 0;

		while (!q.empty())
		{
			// 取出一个点
			pair<int,int> cur = q.front();
			q.pop();
			int x = cur.first;
			int y = cur.second;
			for (int i = 0; i < 4; ++i)
			{
				int nx = x + dir[i][0];
				int ny = y + dir[i][1];
				if (nx < 0 || nx >= 200 || ny < 0 || ny >= 200 || visited[nx][ny] || this->maze[nx][ny] == '*' || this->maze[nx][ny] == '#')
				{
					continue;
				}
				q.push({ nx,ny });
				bth.reachable_map[nx][ny].dir = anti_dir[{dir[i][0],dir[i][1]}];
				bth.reachable_map[nx][ny].dist = bth.reachable_map[x][y].dist + 1;
				visited[nx][ny] = true;
			}
		}
	}

}

/**
 * @brief 通过泊位的可达地图生成当前机器人去泊位的路径
 * @param bot 
 * @param bth 
 * @return 
*/
vector<pair<int, int>> MakeDecision::makePathToBerth(Robot& bot, Berth& bth)
{
	vector<pair<int, int>> res;
	int dir[4][2] = { {0,1},{0,-1},{-1,0},{1,0} };
	// 取出起点
	int x = bot.x;
	int y = bot.y;
	int success = 0;
	// 直到走到泊位
	for (int i = 0; i < 40000; ++i)
	{
		// 通用的操作，将当前点压入结果数组
		res.push_back({ x,y });

		// 如果走到了泊位区域
		if (x >= bth.x && x <= bth.rdx
		 && y >= bth.y && y <= bth.rdy)
		{
			success = 1;
			break;
		}
		// 如果没走到泊位区域，则正常循环
		// 计算下一个点
		int cur_idx = bth.reachable_map[x][y].dir;
		x += dir[cur_idx][0];
		y += dir[cur_idx][1];
	}
	if (res.size() <= 1 || success == 0)
	{
		return vector<pair<int, int>>();
	}
	return res;
}

/**
 * @brief 指派某个机器人去送货物，目前在正常取物后调用
 * @param bot 
 * @param berths 
 * @return 
*/
int MakeDecision::assignRobotSendBFS(Robot& bot, vector<Berth>& berths)
{
	if (bot.berth_id <= -1)
	{
		return -1;
	}
	bot.send_good_path = this->makePathToBerth(bot, berths[bot.berth_id]);
	bot.send_good_cur = 0;
	if (bot.send_good_path.empty())
	{
		return -1;
	}
	return 0;
}

