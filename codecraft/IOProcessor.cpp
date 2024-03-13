#define _CRT_SECURE_NO_WARNINGS
#include "IOProcessor.h"

IOProcessor::IOProcessor(int map_size, int berth_size, int boat_size, int robot_size)
{
	this->map_size = map_size;
	this->berth_size = berth_size;
	this->boat_size = boat_size;
	this->robot_size = robot_size;
}

/**
 * ReadMapFromFile - 从文件读取地图
 * @param file_path : 读取文件路径
 * @param map : 存储地图的变量
 * 
 * @return : {0 : 成功, -1 : 失败}
 * @note : none
 */
int IOProcessor::ReadMapFromFile(string file_path, vector<vector<char>>& map)
{
	ifstream ifs(file_path);
	if (!ifs.is_open())
	{
		cerr << "Error opening the file!" << endl;
		return -1;
	}

	int len = this->map_size, i = 0;
	string line;
	while (getline(ifs, line))
	{
		for (int j = 0; j < len; j++)
			map[i][j] = line[j];
		i++;
	}

	ifs.close();
	return 0;
}

/**
 * OutputMap - 向控制台输出地图
 * @param map : 存储地图的变量
 * 
 * @return : null
 * @note : none
 */
void IOProcessor::OutputMap(vector<vector<char>>& map)
{
	for (int i = 0; i < this->map_size; i++)
	{
		for (int j = 0; j < this->map_size; j++)
		{
			if (j != 0)
			{
				printf(" ");
			}
			printf("%c", map[i][j]);
		}
		printf("\n");
	}
}

/**
 * InitData - 读取初始化数据
 * @param map : 存储地图的变量
 * @param berths : 存储泊口的变量
 * @param boats : 存储轮船的变量
 * 
 * @return : null
 * @note : none
 */
void IOProcessor::InitData(vector<vector<char>>& map, vector<Berth>& berths, vector<Boat>& boats)
{
	// int map_size = map.size(), berth_size = berths.size(), boat_size = boats.size();

	// 读取地图
	string line;
	for (int i = 0; i < this->map_size; i++)
	{
		scanf("%s", &line);
		for (int j = 0; j < this->map_size; j++)
		{
			map[i][j] = line[i];
		}
	}

	// 读取泊口
	for (int i = 0; i < this->berth_size; i++)
	{
		int berth_id;
		scanf("%d", &berth_id);
		scanf("%d%d%d%d", &berths[berth_id].x, &berths[berth_id].y, &berths[berth_id].transport_time, &berths[berth_id].loading_speed);
		berths[berth_id].rdx = berths[berth_id].x + berths[berth_id].r_size - 1;
		berths[berth_id].rdy = berths[berth_id].y + berths[berth_id].c_size - 1;
	}

	int boat_capacity;
	scanf("%d", &boat_capacity);
	for (int i = 0; i < boat_size; i++)
		boats[i].capacity = boat_capacity;

	// 读取一行OK
	char okk[100];
	scanf("%s", okk);

	// 输出OK给判题器
	printf("OK\n");
	fflush(stdout);
}

/**
 * InputFrameData - 读取每一帧的输入
 * @param frame_id : 存储帧数的变量
 * @param money : 存储钱数的变量
 * @param goods : 存储货物的变量
 * @param robots : 存储机器人的变量
 * @param boats : 存储轮船的变量
 * 
 * @return : 当前指令的帧id
 * @note : none
 */
int IOProcessor::InputFrameData(int& frame_id, int& money, vector<Good>& goods, vector<Robot>& robots, vector<Boat>& boats)
{
	// 读取帧数、钱数
	scanf("%d%d", &frame_id, &money);

	// 读取新的货物信息
	int goods_size;
	scanf("%d", &goods_size);
	for (int i = 0; i < goods_size; i++)
	{
		int x, y, val;
		scanf("%d%d%d", &x, &y, &val);
		goods.push_back(Good(x, y, val));
	}

	// 读取机器人信息
	for (int i = 0; i < this->robot_size; i++)
	{
		scanf("%d%d%d%d", &robots[i].is_carry, &robots[i].x, &robots[i].y, &robots[i].status);
	}

	// 读取轮船信息
	for (int i = 0; i < this->boat_size; i++)
	{
		scanf("%d%d\n", &boats[i].status, &boats[i].pos);
	}

	// 读取一行OK
	char okk[100];
	scanf("%s", okk);
	return frame_id;
}

/**
 * OutputCommand - 向判题器输出每一帧的指令
 * @param robot_cmd : 存储一帧的机器人指令集合
 * @param boat_cmd :存储一帧的轮船指令集合
 * 
 * @return : null
 */
void IOProcessor::OutputCommand(vector<vector<Command>>& robot_cmd, vector<vector<Command>>& boat_cmd)
{
	// 输出机器人指令
	for (int i = 0; i < robot_cmd.size(); i++)
	{
		for (int j = 0; j < robot_cmd[0].size(); j++)
		{
			int cmd_key = robot_cmd[i][j].key;
			if (cmd_key == -1)
			{
				printf("%s %d ", this->key_to_name[cmd_key], robot_cmd[i][j].id);
				if (robot_cmd[i][j].param_2 != -1)
				{
					printf("%d\n", robot_cmd[i][j].param_2);
				}
			}
		}
	}

	// 输出轮船指令
	for (int i = 0; i < boat_cmd.size(); i++)
	{
		for (int j = 0; j < boat_cmd[0].size(); j++)
		{
			int cmd_key = boat_cmd[i][j].key;
			if (cmd_key == -1)
			{
				printf("%s %d ", this->key_to_name[cmd_key], boat_cmd[i][j].id);
				if (boat_cmd[i][j].param_2 != -1)
				{
					printf("%d\n", boat_cmd[i][j].param_2);
				}
			}
		}
	}

	// 输出OK
	printf("OK\n");
	fflush(stdout);
}
