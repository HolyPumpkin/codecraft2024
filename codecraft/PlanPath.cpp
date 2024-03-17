#include "PlanPath.h"

/**
 * PlanPath - �������빹��·���滮��ʵ��
 * @_maze: ��ά�ַ���ͼ
 * @_N: �����ͼ��С
 * @_n: ʵ�ʵ�ͼ��С
 *
 * �������������������PlanPath��ʵ��
 *
 * ����ֵ:
 * ��
 *
 * ע������/Լ��������
 * ��
 */
PlanPath::PlanPath(vector<vector<char>>& _maze, int _N, int _n, vector<Robot> robots)
{
	this->maze = _maze;
    this->N = _N;
    this->n = _n;
    //ÿ�θ�������Ļ�������Ϣ���µ�ͼ���ָ�״̬�Ļ�����Ҫ�����ϰ��������ײ
    for (auto& i : robots)
    {
        if (i.status == 0)
        {
            this->maze[i.x][i.y] = '#';
        }
    }
}

/**
 * getDis - ������������
 * @start: ���ṹ��
 * @end: �յ�ṹ��
 *
 * ���ݽ��յ�������ṹ�壬���������֮��ľ���
 *
 * ����ֵ:
 * �����ľ���
 *
 * ע������/Լ��������
 * Ĭ��ʵ��Ϊ�����پ���
 */
int PlanPath::getDis(Point& start, Point& end)
{
    //Ĭ�������پ���
    return abs(start.x - end.x) + abs(start.y - end.y);
}

/**
 * getMinF - ����������б���ѯ���õ�����Fֵ��С�ĵ�
 * @l: �洢ָ��Point�ṹ���ָ����б�
 *
 * ��ѯ�б���Fֵ��С�ĵ㣬������
 *
 * ����ֵ:
 * ָ��Point�ṹ���ָ��
 *
 * ע������/Լ��������
 * ��
 */
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

/**
 * isInlist - ���������ж��б����Ƿ����ĳ��
 * @l: �洢ָ��Point�ṹ���ָ����б�
 * @p: ��Ҫ��ѯ��Point�ṹ��
 *
 * ���������������ж�����ĵ��Ƿ����б���
 *
 * ����ֵ:
 * ����ֵ
 *
 * ע������/Լ��������
 * ��
 */
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

/**
 * getPoint - ��������õ��б��ж�Ӧ��Point�ṹ��
 * @l: �洢ָ��Point�ṹ���ָ����б�
 * @p: ��Ҫ������Point�ṹ��
 *
 * ���������������õ��б��������������һ�µĵ�
 *
 * ����ֵ:
 * ָ��Point�ṹ���ָ��
 *
 * ע������/Լ��������
 * Point�ṹ���е�Fֵ�����ǲ�һ���ģ���������������ͬ�ĵ���·��
 * �滮�㷨�в�һ����ͬһ���㣬������ʼ�����б��д洢�ĵ�Ϊ׼
 */
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

/**
 * printPath - �������뽫��Ӧ·����ӡ
 * @path:���������ʽ�洢������
 *
 * ����һ����������ӡ����Ӧ��·��
 *
 * ����ֵ:
 * ��
 *
 * ע������/Լ��������
 * ��
 */
void PlanPath::printPath(vector<pair<int, int>>& path)
{
    std::cout << "·�����£�" << endl;
    for (auto& i : path)
    {
        std::cout << "(" << i.first << "," << i.second << ")" << endl;
    }
}

/**
 * pathplanning - ������������·���滮�㷨
 * @start:��Ϊ����Point�ṹ��
 * @end:��Ϊ�յ��Point�ṹ��
 *
 * �����������������յ㣬����·���滮�㷨���õ�һ�����·����
 *
 * ����ֵ:
 * ���������ʽ�洢�����飬�������㵽�յ��·��
 *
 * ע������/Լ��������
 * Ĭ��ʵ��ΪA star�㷨
 */
vector<pair<int, int>> PlanPath::pathplanning(Point& start, Point& end)
{
    //��ʼ��
    //������Ż������ѵ�
    //priority_queue<Point> openlist; //��ڵ���汸ѡ�ڵ�, ���ڵ�Fֵ��������
    list<Point*> openlist;   //����ʵ�֣��������޸�ΪO(N)
    vector<vector<int>> visited(N, vector<int>(N, 0));    //�ڵ��ѷ��ʹ�ϵ,���closedlist

    //����ʼ��
    start.G = 0;
    start.H = getDis(start, end);
    start.F = start.G + start.H;
    Point* p_start = &start;
    openlist.push_back(p_start);
    Point* current = new Point(0, 0);
    while (!openlist.empty())
    {
        //cout << "listsize:" << openlist.size() << endl;
        //ȡ���Žڵ㣬���Ƴ���openlist
        current = getMinF(openlist);
        openlist.remove(current);

        //�ѷ��ʹ�������
        if (visited[current->x][current->y] == 1)
            continue;
        visited[current->x][current->y] = 1;  //���Ϊ�ѷ���
        //ѭ����ֹ�������ҵ��յ�
        if (*current == end)
        {
            //cout << "�˳�ѭ�����ҵ����յ�,current.parent:" << current->parent->x << "," << current->parent->y << endl;
            break;
        }
        //cout << "current:" << current->x << "," << current->y << endl;
        //�����ھ�
        for (int dir = 0; dir < 4; ++dir)
        {
            int nx, ny;
            switch (dir)
            {
                //0��1��2��3->�ϡ��ҡ��¡���
                //��ͼ�·�Ϊx�����ҷ�Ϊy��
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
            //�ھӲ��ɴ�
            if (nx < 0 || nx >= n || ny < 0 || ny >= n
                || maze[nx][ny] == '*' || maze[nx][ny] == '#')
            {
                continue;
            }

            //�õ�һ����Ч�ھ�����
            Point* Neighbor = new Point(nx, ny);
            //�ѷ��ʹ��ھӽڵ�
            if (visited[Neighbor->x][Neighbor->y] == 1)
                continue;
            //����neighbor����Ϣ
            int newG = current->G + 1;
            Neighbor->G = newG;
            Neighbor->H = getDis(*Neighbor, end);
            Neighbor->F = Neighbor->G + Neighbor->H;

            //�������openlist���͸��²�����
            if (!isInlist(openlist, *Neighbor))
            {

                Neighbor->parent = current;
                openlist.push_front(Neighbor);
                //cout << "����openlist��," << Neighbor.x << "," << Neighbor.y << "��parent��" << Neighbor.parent->x << "," << Neighbor.parent->y << endl;
            }
            //�������ڣ��ǾͿ����Ƿ�Ҫ���£���ͨ����ǰ�ڵ㵽����ԭ�����ĸ�����
            else {
                Point* temp = getPoint(openlist, *Neighbor);
                if (newG < temp->G)
                {

                    //cout << "��openlist��," << Neighbor.x << "," << Neighbor.y << "��parent��" << current.x << "," << current.y << endl;
                    openlist.remove(temp);
                    openlist.push_front(Neighbor);
                    Neighbor->parent = current;
                }
            }
        }
    }
    //���ݽ����׷��·��
    vector<pair<int, int>> path;
    Point* p = current;
    while (p != nullptr)
    {
        //cout << "׷���С���" << p->x << "," << p->y;
        path.push_back(make_pair(p->x, p->y));
        p = p->parent;
    }
    //��ȷ����
    std::reverse(path.begin(), path.end());
    return path;
    
}
