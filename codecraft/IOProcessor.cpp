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
 * ReadMapFromFile - ���ļ���ȡ��ͼ
 * @param file_path : ��ȡ�ļ�·��
 * @param map : �洢��ͼ�ı���
 * 
 * @return : {0 : �ɹ�, -1 : ʧ��}
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
 * OutputMap - �����̨�����ͼ
 * @param map : �洢��ͼ�ı���
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
 * InitData - ��ȡ��ʼ������
 * @param map : �洢��ͼ�ı���
 * @param berths : �洢���ڵı���
 * @param boats : �洢�ִ��ı���
 * 
 * @return : null
 * @note : none
 */
void IOProcessor::InitData(vector<vector<char>>& map, vector<Berth>& berths, vector<Boat>& boats)
{
	// int map_size = map.size(), berth_size = berths.size(), boat_size = boats.size();

	// ��ȡ��ͼ
	string line;
	for (int i = 0; i < this->map_size; i++)
	{
		scanf("%s", &line);
		for (int j = 0; j < this->map_size; j++)
		{
			map[i][j] = line[i];
		}
	}

	// ��ȡ����
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

	// ��ȡһ��OK
	char okk[100];
	scanf("%s", okk);

	// ���OK��������
	printf("OK\n");
	fflush(stdout);
}

/**
 * InputFrameData - ��ȡÿһ֡������
 * @param frame_id : �洢֡���ı���
 * @param money : �洢Ǯ���ı���
 * @param goods : �洢����ı���
 * @param robots : �洢�����˵ı���
 * @param boats : �洢�ִ��ı���
 * 
 * @return : ��ǰָ���֡id
 * @note : none
 */
int IOProcessor::InputFrameData(int& frame_id, int& money, vector<Good>& goods, vector<Robot>& robots, vector<Boat>& boats)
{
	// ��ȡ֡����Ǯ��
	scanf("%d%d", &frame_id, &money);

	// ��ȡ�µĻ�����Ϣ
	int goods_size;
	scanf("%d", &goods_size);
	for (int i = 0; i < goods_size; i++)
	{
		int x, y, val;
		scanf("%d%d%d", &x, &y, &val);
		goods.push_back(Good(x, y, val));
	}

	// ��ȡ��������Ϣ
	for (int i = 0; i < this->robot_size; i++)
	{
		scanf("%d%d%d%d", &robots[i].is_carry, &robots[i].x, &robots[i].y, &robots[i].status);
	}

	// ��ȡ�ִ���Ϣ
	for (int i = 0; i < this->boat_size; i++)
	{
		scanf("%d%d\n", &boats[i].status, &boats[i].pos);
	}

	// ��ȡһ��OK
	char okk[100];
	scanf("%s", okk);
	return frame_id;
}

/**
 * OutputCommand - �����������ÿһ֡��ָ��
 * @param robot_cmd : �洢һ֡�Ļ�����ָ���
 * @param boat_cmd :�洢һ֡���ִ�ָ���
 * 
 * @return : null
 */
void IOProcessor::OutputCommand(vector<vector<Command>>& robot_cmd, vector<vector<Command>>& boat_cmd)
{
	// ���������ָ��
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

	// ����ִ�ָ��
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

	// ���OK
	printf("OK\n");
	fflush(stdout);
}
