#define _CRT_SECURE_NO_WARNINGS
#include "MyStruct.h"
#include "IOProcessor.h"
#include "Test.h"


/***********************************************************************/
const int n = 200;			// ʵ�ʵ�ͼ��С
const int N = 210;			// �����ͼ��С

const int robot_num = 10;	// ����������
const int berth_num = 10;	// ��������
const int boat_num = 5;		// �ִ�����

int money, frame_id;		// Ǯ����֡��

/***********************************************************************/
vector<vector<char>> ch_map(N, vector<char>(N));	// ��ͼ
vector<Robot> robots(robot_num);				// ������
vector<Berth> berths(berth_num);				// ����
vector<Boat> boats(boat_num);					// �ִ�

// ������һ֡��ָ����ִ�һ֡��ָ�
// ��i�У���i�������˵�ָ�
// 
// �洢��ʽ�����Ϊ�ڽӱ����£�
// [robot1]: [rbt_cmd1, rbt_cmd2, ...]
// [robot2]: [rbt_cmd1, rbt_cmd2, ...]
// ...
vector<vector<Command>> robot_cmd(robot_num);
vector<vector<Command>> boat_cmd(boat_num);

//int gds[N][N];

/***********************************************************************/
int main()
{
	IOProcessor iop;
	iop.ReadMapFromFile("../maps/map3.txt", ch_map);
	iop.OutputMap(ch_map);

	Test test;
	Point s = Point(30, 122);
	Point e = Point(66, 56);
	test.displayPath(ch_map, s, e, N, n);


	return 0;
}