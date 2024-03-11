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
	PlanPath pp = PlanPath(map, N, n);
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
