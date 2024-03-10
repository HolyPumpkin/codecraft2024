#include "IOProcessor.h"

void IOProcessor::InitData(vector<vector<char>>& map, vector<Berth>& berth, vector<Boat>& boat)
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
	int map_size = map.size(), berth_size = berth.size(), boat_size = boat.size();
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
		scanf("%d%d%d%d", &berth[berth_id].x, &berth[berth_id].y, &berth[berth_id].transport_time, &berth[berth_id].loading_speed);
	}

	int boat_capacity;
	scanf("%d", &boat_capacity);
	for (int i = 0; i < boat_size; i++)
		boat[i].capacity = boat_capacity;


	char okk[100];
	scanf("%s", okk);
	printf("OK\n");
	fflush(stdout);
}
