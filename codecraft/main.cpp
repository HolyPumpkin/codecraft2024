#define _CRT_SECURE_NO_WARNINGS
#include "MyStruct.h"
#include "IOProcessor.h"


/***********************************************************************/
const int n = 200;	// ʵ�ʵ�ͼ��С
const int N = 210;	// �����ͼ��С

const int robot_num = 10;	// ����������
const int berth_num = 10;	// ��������
const int boat_num = 5;		// �ִ�����

int money, frame_id;		// Ǯ����֡��

/***********************************************************************/
vector<vector<char>> map(N, vector<char>(N));
vector<Robot> robots(robot_num);
vector<Berth> berths(berth_num);
vector<Boat> boats(boat_num);

//int gds[N][N];


int main()
{
	IOProcessor iop;
	iop.ReadMapFromFile("../maps/map3.txt", map);
	iop.OutputMap(map);



	return 0;
}