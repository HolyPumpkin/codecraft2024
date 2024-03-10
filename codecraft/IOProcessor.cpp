#include "IOProcessor.h"

void IOProcessor::InitData(vector<vector<char>>& map, vector<Berth>& berths, vector<Boat>& boats)
{
	// 读取地图文件
	//ifstream ifs("map1.txt");
	//if (!ifs.is_open())
	//{
	//	cerr << "Error opening the file!" << endl;
	//	return;
	//}

	//string line;
	//while (getline(ifs, line))
	//{
	//	vector<char> temp;
	//	for (int i = 0; i < line.size(); i++)
	//		temp.push_back(line[i]);
	//	map.push_back(temp);
	//}

	//ifs.close();


	// 读取地图
	int map_size = map.size(), berth_size = berths.size(), boat_size = boats.size();
	string line;
	for (int i = 0; i < map_size; i++)
	{
		scanf("%s", &line);
		for (int j = 0; j < map_size; j++)
		{
			map[i][j] = line[i];
		}
	}

	// 读取泊口
	for (int i = 0; i < berth_size; i++)
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

int IOProcessor::InputFrameData(int& frame, int& money, vector<Good>& goods, vector<Robot>& robots, vector<Boat>& boats)
{
	// 读取帧数、钱数
	scanf("%d%d", &frame, &money);

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
	for (int i = 0; i < robots.size(); i++)
	{
		scanf("%d%d%d%d", &robots[i].is_carry, &robots[i].x, &robots[i].y, &robots[i].status);
	}

	// 读取轮船信息
	for (int i = 0; i < boats.size(); i++)
	{
		scanf("%d%d\n", &boats[i].status, &boats[i].pos);
	}

	// 读取一行OK
	char okk[100];
	scanf("%s", okk);
	return 0;
}