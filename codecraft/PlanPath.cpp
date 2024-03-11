#include "PlanPath.h"

PlanPath::PlanPath(vector<vector<char>>& _map, int _N, int _n)
{
	this->map = _map;
    this->N = _N;
    this->n = _n;
}

int PlanPath::getDis(Point& start, Point& end)
{
    //默认曼哈顿距离
    return abs(start.x - end.x) + abs(start.y - end.y);
}

Point* PlanPath::getMinF(list<Point*>& l)
{
    Point* res = l.front();
    for (auto& i : l)
    {
        if (i->F < res->F)
        {
            res = i;
        }
    }
    return res;
}

bool PlanPath::isInlist(list<Point*>& l, Point& p)
{
    for (auto& t : l)
    {
        if (t->x == p.x && t->y == p.y)
        {
            return true;
        }
    }
    return false;
}

Point* PlanPath::getPoint(list<Point*>& l, Point& p)
{
    for (auto& i : l)
    {
        if (i->x == p.x && i->y == p.y)
        {
            return i;
        }
    }
    return new Point(-1, -1);
}

void PlanPath::printPath(vector<pair<int, int>>& path)
{
    std::cout << "路径如下：" << endl;
    for (auto& i : path)
    {
        std::cout << "(" << i.first << "," << i.second << ")" << endl;
    }
}

vector<pair<int, int>> PlanPath::pathplanning(Point& start, Point& end)
{
    //初始化
    //二叉堆优化，有难点
    //priority_queue<Point> openlist; //活节点表，存备选节点, 按节点F值升序排列
    list<Point*> openlist;   //基本实现，查找与修改为O(N)
    vector<vector<int>> visited(N, vector<int>(N, 0));    //节点已访问关系,针对closedlist

    //起点初始化
    start.G = 0;
    start.H = getDis(start, end);
    start.F = start.G + start.H;
    Point* p_start = &start;
    openlist.push_back(p_start);
    Point* current = new Point(0, 0);
    while (!openlist.empty())
    {
        //cout << "listsize:" << openlist.size() << endl;
        //取最优节点，并移除出openlist
        current = getMinF(openlist);
        openlist.remove(current);

        //已访问过则跳过
        if (visited[current->x][current->y] == 1)
            continue;
        visited[current->x][current->y] = 1;  //标记为已访问
        //循环终止条件，找到终点
        if (*current == end)
        {
            //cout << "退出循环，找到了终点,current.parent:" << current->parent->x << "," << current->parent->y << endl;
            break;
        }
        //cout << "current:" << current->x << "," << current->y << endl;
        //遍历邻居
        for (int dir = 0; dir < 4; ++dir)
        {
            int nx, ny;
            switch (dir)
            {
                //0、1、2、3->上、右、下、左
                //地图下方为x正，右方为y正
            case 0:
                nx = current->x - 1;
                ny = current->y;
                break;
            case 1:
                nx = current->x;
                ny = current->y + 1;
                break;
            case 2:
                nx = current->x + 1;
                ny = current->y;
                break;
            case 3:
                nx = current->x;
                ny = current->y - 1;
                break;
            }
            //邻居不可达
            if (nx < 0 || nx >= n || ny < 0 || ny >= n
                || map[nx][ny] == '*' || map[nx][ny] == '#')
            {
                continue;
            }

            //得到一个有效邻居坐标
            Point* Neighbor = new Point(nx, ny);
            map[nx][ny] = '/';
            //已访问过邻居节点
            if (visited[Neighbor->x][Neighbor->y] == 1)
                continue;
            //计算neighbor的信息
            int newG = current->G + 1;
            Neighbor->G = newG;
            Neighbor->H = getDis(*Neighbor, end);
            Neighbor->F = Neighbor->G + Neighbor->H;

            //如果不在openlist，就更新并加入
            if (!isInlist(openlist, *Neighbor))
            {

                Neighbor->parent = current;
                openlist.push_front(Neighbor);
                //cout << "不在openlist中," << Neighbor.x << "," << Neighbor.y << "的parent是" << Neighbor.parent->x << "," << Neighbor.parent->y << endl;
            }
            //本来就在，那就看看是否要更新，即通过当前节点到它和原来比哪个更好
            else {
                Point* temp = getPoint(openlist, *Neighbor);
                if (newG < temp->G)
                {

                    //cout << "在openlist中," << Neighbor.x << "," << Neighbor.y << "的parent是" << current.x << "," << current.y << endl;
                    openlist.remove(temp);
                    openlist.push_front(Neighbor);
                    Neighbor->parent = current;
                }
            }
        }
    }
    //根据结果点追溯路径
    vector<pair<int, int>> path;
    Point* p = current;
    while (p != nullptr)
    {
        //cout << "追溯中……" << p->x << "," << p->y;
        path.push_back(make_pair(p->x, p->y));
        p = p->parent;
    }
    //正确排列
    reverse(path.begin(), path.end());
    return path;
    
}
