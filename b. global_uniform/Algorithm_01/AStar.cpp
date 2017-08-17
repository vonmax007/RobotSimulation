
/*
	���㷨ʵ�֣�ʹ��STL list��¼
	��unsigned charΪ�˽�ʡ�ռ䣬
	״̬���࣬255������
	Դ�ļ�
*/

#include "StdAfx.h"
#include "AStar.h"

AStar::AStar()
{
}

AStar::~AStar()
{
	//��Ҫ������������ɾ������;
	//������
	//�ͷ���Ƕmap,isMapModified ��ά����ռ�
	for (short i = 0; i < rows; ++i)
	{
		delete[] map[i];
		delete[] isMapModified[i];
	}
	delete[] map;
	delete[] isMapModified;
	this->Open.clear();
	this->Closed.clear();
}


void AStar::PrintMap()
{	//�����ͼ
	int i, j;
	for (i = 0; i<rows; i++)
	{
		for (j = 0; j<cols; j++)
		{
			if (map[i][j] == Tile_Open || map[i][j] == Tile_UP ||
				map[i][j] == Tile_DOWN || map[i][j] == Tile_LEFT
				|| map[i][j] == Tile_RIGHT || map[i][j] == Tile_UP_LEFT
				|| map[i][j] == Tile_UP_RIGHT || map[i][j] == Tile_DOWN_LEFT
				|| map[i][j] == Tile_DOWN_RIGHT) {
				printf("��");
			}
			else if (map[i][j] == Tile_Lock) {
				printf("��");
			}
			else if (map[i][j] == Tile_Start) {
				printf("��");
			}
			else if (map[i][j] == Tile_End) {
				printf("��");
			}
			else if (map[i][j] == Tile_Path) {
				printf("��");
			}
		}
		printf("\n");
	}
	printf("\n\n");
}

//ѡȡOpen����fֵ��С�Ľڵ㣬���ظýڵ��ַ
AStarNode AStar::getNodeFromOpen()
{	
	if (this->Open.size() == 0)return AStarNode();
	AStarNode minx;//Ҫ���ص���СֵAStar�ڵ��ָ��;
	int minF = this->Open.begin()->f;
	list <AStarNode>::iterator oit = this->Open.begin();
	list <AStarNode>::iterator minit = this->Open.begin();
	for (; oit != Open.end(); ++oit)
	{
		if (oit->f < minF)
		{
			minit = oit;	//min��ʾf��С�������
			minF = oit->f;	//ָ��min��ָ�룬Ϊ�����ں�����ͷ����õ�
		}
	}
	minx = * minit;
	Open.erase(minit++);
	return minx;
}

//У��Open��,��ʵ���ǲ��Ҵ��к��еĽڵ�ʵ��;
AStarNode AStar::checkOpen(int row, int col)
{
	if (Open.size() == 0)return AStarNode();
	list <AStarNode>::iterator oit = Open.begin();
	for (; oit != Open.end(); ++oit)
	{
		if (oit->row == row && oit->col == col)
			return *oit;
	}
	return AStarNode();
}

//�Ƿ������Close��
bool AStar::isInClose(int row, int col)
{
	if (Closed.size() == 0)return false;
	list <AStarNode>::iterator clit = Closed.begin();
	for (; clit != Closed.end(); ++clit)
	{
		if (clit->row == row && clit->col == col)
			return true;
	}
	return false;
}

//�õ���ͼ���Hֵ
int AStar::getH(int row, int col)
{
	int h;
	if (short(isMapModified[row][col]) == 0)	//δ�޸ģ��ͷ���,�򷵻�ԭʼֵ
		h = abs(destinationRow - row) + abs(destinationCol - col) + 1;
	else h = pow(PenaltyScale, short(isMapModified[row][col])) * (abs(destinationRow - row) + abs(destinationCol - col) + 1);
	return h * 10;
}


//����ͼ���Ƿ��ͨ��
bool AStar::isCanMove(int frow, int fcol, int row, int col)
{
	if (col < 0 || col >= cols)
		return false;
	if (row < 0 || row >= rows)
		return false;
	if (map[row][col] == Tile_Lock)
		return false;

	if ((row == frow - 1 && col == fcol) &&
		(map[row][col] == Tile_DOWN ||
			map[row][col] == Tile_DOWN_LEFT ||
			map[row][col] == Tile_DOWN_RIGHT)
		)return false;	//���ʲ���ͨ���Ϸ�

	if ((row == frow + 1 && col == fcol) &&
		(map[row][col] == Tile_UP ||
			map[row][col] == Tile_UP_LEFT ||
			map[row][col] == Tile_UP_RIGHT)
		)return false;	//���ʲ���ͨ���·�

	if ((row == frow && col == fcol - 1) &&
		(map[row][col] == Tile_RIGHT ||
			map[row][col] == Tile_UP_RIGHT ||
			map[row][col] == Tile_DOWN_RIGHT)
		)return false;	//���ʲ���ͨ����

	if ((row == frow && col == fcol + 1) &&
		(map[row][col] == Tile_LEFT ||
			map[row][col] == Tile_UP_LEFT ||
			map[row][col] == Tile_DOWN_LEFT)
		)return false;	//���ʲ���ͨ���ҷ�

	return true;
}


//������һȦ��A�ǽڵ�
void AStar::creatNextLapNode(AStarNode & bestNode, int row, int col, int G_OFFSET)
{
	int g = bestNode.g + G_OFFSET;
	if (!isInClose(row, col))
	{
		AStarNode oldNode;
		if ((oldNode = checkOpen(row, col)).col != -1)
		{
			if (oldNode.g > g)
			{
				oldNode.parent = & bestNode;
				oldNode.g = g;
				oldNode.f = g + oldNode.h;
			}
		}
		else
		{
			AStarNode node;
			node.parent = & bestNode;
			node.g = g;
			node.h = getH(row, col);
			node.f = node.g + node.h;
			node.row = row;
			node.col = col;

			this->Open.push_back(node);
		}
	}
}

//���ݴ���Ľڵ������ӽڵ�
void AStar::seachSeccessionNode(AStarNode & bestNode)
{
	int row, col;
	//�ϲ��ڵ�
	if (isCanMove(bestNode.row, bestNode.col, row = bestNode.row - 1, col = bestNode.col))
	{
		creatNextLapNode(bestNode, row, col, G_OFFSET1);
	}
	//�²��ڵ�
	if (isCanMove(bestNode.row, bestNode.col, row = bestNode.row + 1, col = bestNode.col))
	{
		creatNextLapNode(bestNode, row, col, G_OFFSET1);
	}
	//�󲿽ڵ�
	if (isCanMove(bestNode.row, bestNode.col, row = bestNode.row, col = bestNode.col - 1))
	{
		creatNextLapNode(bestNode, row, col, G_OFFSET1);
	}
	//�Ҳ��ڵ�
	if (isCanMove(bestNode.row, bestNode.col, row = bestNode.row, col = bestNode.col + 1))
	{
		creatNextLapNode(bestNode, row, col, G_OFFSET1);
	}
	
	this->Closed.push_back(bestNode);
}

bool AStar::isSamePath(list <Point> & path1, list <Point> & path2)
{
	if (path1.size() != path2.size())return false;	//ֱ�ӷ����䲻ͬ·��Ҳ
	list <Point>::iterator it1, it2;
	for (it1 = path1.begin(), it2 = path2.begin();
	it1 != path1.end(), it2 != path2.end();
		++it1, ++it2)
	{
		if (it1->i != it2->i || it1->j != it2->j)return false;
	}
	return true;
}

bool AStar::isPathExisted(list <list <Point>> & PathSet, list <Point> & path)
{
	list <list <Point>>::iterator its;
	for (its = PathSet.begin(); its != PathSet.end(); ++its)
	{
		if (isSamePath(*its, path))return true;
	}
	return false;
}

void AStar::modifyPath(list <Point> & path, int & StartX, int & StartY, int & EndX, int & EndY)
{
	//�޸�ָ��·���ϵ���Դ
	double a, b;				//alpha and belta
	srand((unsigned)time(nullptr)); //srand(3)
	a = rand() / double(RAND_MAX);
	list <Point>::iterator it;
	for (it = path.begin(); it != path.end(); ++it)
	{
		bool b1 = (abs(it->i - StartX) + abs(it->j - StartY) < PenaltyD);
		bool b2 = (abs(it->i - EndX) + abs(it->j - EndY) < PenaltyD);
		if (b1 || b2)continue;//����������յ㸽�������ͷ�;
		else
		{
			//����bû����ʱ�������������ΪCPU̫���ˣ�ʱ�����̫�ӽ����
			b = rand() / double(RAND_MAX);	//�Ը���belta < alpha �ͷ�
			if (b < a)isMapModified[it->i][it->j] += 1;
		}
	}
}

//������,����k��·��
list <list <Point>> AStar::findKPath(int k, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY)
{
	PathSet kpath;
	rows = inRow;
	cols = inCol;
	//��̬������Ƕmap,isMapModified �����Խ�ʡ�ռ�
	map = new TileType *[rows];
	isMapModified = new TileType *[rows];
	for (short i = 0; i < rows; ++i)
	{
		map[i] = new TileType[cols];
		isMapModified[i] = new TileType[cols];
	}

	//���մ�������ֵ;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			map[i][j] = inMap[i][j];
			isMapModified[i][j] = 0;	//��ʼĬ����δ���޸ĵ�
			if (i == StartX && j == StartY)
			{
				map[i][j] = Tile_Start;
			}
			if (i == EndX && j == EndY)
			{
				map[i][j] = Tile_End;
			}
		}
	}

	//PrintMap();
	//std::cout << "\n\n *****************************  \n\n";

	list <AStarNode> OpenSave;
	int pathCount = 0;
	int failCount = 0;
	while (pathCount < k && failCount < MAXFAIL)
	{
		AStarNode startNode;	// = new AStarNode;
		destinationRow = EndX;
		destinationCol = EndY;
		startNode.parent = nullptr;
		startNode.row = StartX;
		startNode.col = StartY;
		startNode.g = 0;
		startNode.h = getH(startNode.row, startNode.col);
		startNode.f = startNode.g + startNode.h;
		this->Open.push_back(startNode);

		list<Point> path;
		Point point;
		while (true)
		{
			AStarNode bestNode = getNodeFromOpen();//��OPEN����ȡ��fֵ��С�Ľڵ�
			OpenSave.push_back(bestNode);
			if (bestNode.col == -1)			//δ�ҵ�·��
			{
				OpenSave.clear();
				break;
			}
			else if (bestNode.row == destinationRow && bestNode.col == destinationCol)
			{
				map[bestNode.row][bestNode.col] = Tile_Path;
				while ( bestNode.parent != nullptr )
				{
					map[bestNode.row][bestNode.col] = Tile_Path;
					point.i = bestNode.row;
					point.j = bestNode.col;
					path.push_front(point);			   //ѹ��Ŀ�ĵص�
					
					bestNode = * bestNode.parent;					
				}
				path.push_front(Point(StartX, StartY));//�����Ҳѹ��ȥ
				break;
			}

			seachSeccessionNode(OpenSave.back());
		}
		if (!isPathExisted(kpath, path))
		{
			kpath.push_front(path);		
			//PrintMap();
			modifyPath(path, StartX, StartY, EndX, EndY);
			path.clear();
			failCount = 0;	//���ʧ��ͳ��
			pathCount++;	//·��ͳ��ֵ����
		}
		else failCount++;

		this->Open.clear();
		this->Closed.clear();
	}

	OpenSave.clear();
	return kpath;
}
