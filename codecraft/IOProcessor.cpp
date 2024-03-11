#include "IOProcessor.h"

IOProcessor::IOProcessor(int map_size, int berth_size, int boat_size, int robot_size)
{
	this->map_size = map_size;
	this->berth_size = berth_size;
	this->boat_size = boat_size;
	this->robot_size = robot_size;
}

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
	}

	ifs.close();
	return 0;
}

void IOProcessor::OutputMap(vector<vector<char>>& map)
{
	for (int i = 0; i < this->map_size; i++)
	{
		for (int j = 0; j < this->map_size; j++)
			printf("%c", map[i][j]);
		printf("\n");
	}
}

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