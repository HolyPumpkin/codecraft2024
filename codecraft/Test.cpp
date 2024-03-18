#include "Test.h"

void Test::displayPath(vector<vector<char>> map, Point& start, Point& end, int N, int n)
{
	if (map[start.x][start.y] == '#' || map[start.x][start.y] == '*')
	{
		cout << " start is invalid!, please re-input" << endl;
		return;
	}
	else if (map[end.x][end.y] == '#' || map[end.x][end.y] == '*')
	{
		cout << " start is invalid!, please re-input" << endl;
		return;
	}
	vector<Robot> r(10);
	PlanPath pp = PlanPath(map, N, n, r);
	vector<pair<int, int>> path = pp.pathplanning(start, end);
	for (auto& i : path)
	{
		map[i.first][i.second] = '$';
	}
	map[start.x][start.y] = '0';
	map[end.x][end.y] = 'X';
	IOProcessor iop;
	iop.OutputMap(map);
}

void timetest()
{
	//ʱ����Ե�����
	std::ofstream outputFile("test.txt");	//����
	auto start = std::chrono::steady_clock::now();
	//��Ҫ���ԵĴ���λ��
	auto end = std::chrono::steady_clock::now();
	auto duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	outputFile << "time:" << duration_ms << "us" << std::endl;
	outputFile.close();
	
}