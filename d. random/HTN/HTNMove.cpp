
#include "StdAfx.h"
#include "HTNMove.h"

HTNMove::HTNMove()
{
}


HTNMove::~HTNMove()
{
	for (short i = 0; i < rows; ++i)
		delete[] map[i];
	delete[] map;
}


void HTNMove::PrintMap()
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

//����ͼ���Ƿ��ͨ��
bool HTNMove::isCanMove(HTNPoint start, int enter_slot, int end_slot, ResEntity ** ResEntity_list, int frow, int fcol, int row, int col)
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

	if (start.parent != nullptr &&  
		start.parent->i == row && start.parent->j == col)return false;	//���ʸ��ڵ�

						//�˴�Ϊ�����·�����Ҳ��ж��Ƿ��н�����ͻ
	if (ResEntity_list[row][col].isFreePass(enter_slot, end_slot) == -1)
		return false;	//���ж��Ƿ�ʱ�䴰�ɴ�;�˺�������ֵ-1��ʾ����ͨ��;

	return true;
}

bool HTNMove::isInPath(list<HTNPoint> & searchTree, int & si, int & sj)
{
	list<HTNPoint>::iterator hit = searchTree.begin();
	for (; hit != searchTree.end(); ++hit)
	{
		if (hit->i == si && hit->j == sj)return true;
	}
	return false;
}

list<HTNPoint>::iterator getTreeEle(list<HTNPoint> & searchTree, int & si, int & sj)
{
	list<HTNPoint>::iterator hit = searchTree.begin();
	for (; hit != searchTree.end(); ++hit)
	{
		if (hit->i == si && hit->j == sj)return hit;
	}
	return hit;
}



//�ݹ麯��
HTNPoint & HTNMove::searchNext(int & curr_slot, HTNPoint & start, int destX, int destY, ResEntity ** ResEntity_list, list<HTNPoint> & searchTree, list<HTNPoint> & path)
{
	if (start.i == destX && start.j == destY)return start;//���ص�Ŀ�ĵص�ʱ���
	else if (start.enter_slot - curr_slot >= LoopLength ||
		searchTree.size() >= this->rows * this->cols)
	{
		HTNPoint em(-1, -1, -1, -1);
		searchTree.push_back(em);
		return searchTree.back();
	}
	else//��Ѱ����
	{
		int row, col;
		list<HTNPoint *> nearPoints;
		int end_slot = ResEntity_list[start.i][start.j].locatedslot(start.enter_slot);
		//�ϲ��ڵ�
		if (isCanMove(start, start.enter_slot, end_slot, ResEntity_list, start.i, start.j, row = start.i - 1, col = start.j)
			&& !isInPath(path, row, col))
		{
			HTNPoint chi;
			chi.i = row, chi.j = col;
			chi.enter_slot = ResEntity_list[row][col].isFreePass(start.enter_slot, end_slot);
			chi.leave_slot = -1;	//���ұ�ʾδֵ֪;
			chi.heuristic = abs(chi.i - destX) + abs(chi.j - destY) + 1;
			chi.parent = &start;
			searchTree.push_back(chi);
			nearPoints.push_back(&searchTree.back());
		}
		//�²��ڵ�
		if (isCanMove(start, start.enter_slot, end_slot, ResEntity_list, start.i, start.j, row = start.i + 1, col = start.j)
			&& !isInPath(path, row, col))
		{
			HTNPoint chi;
			chi.i = row, chi.j = col;
			chi.enter_slot = ResEntity_list[row][col].isFreePass(start.enter_slot, end_slot);
			chi.leave_slot = -1;	//���ұ�ʾδֵ֪;
			chi.heuristic = abs(chi.i - destX) + abs(chi.j - destY) + 1;
			chi.parent = &start;
			searchTree.push_back(chi);
			nearPoints.push_back(&searchTree.back());
		}
		//�󲿽ڵ�
		if (isCanMove(start, start.enter_slot, end_slot, ResEntity_list, start.i, start.j, row = start.i, col = start.j - 1)
			&& !isInPath(path, row, col))
		{
			HTNPoint chi;
			chi.i = row, chi.j = col;
			chi.enter_slot = ResEntity_list[row][col].isFreePass(start.enter_slot, end_slot);
			chi.leave_slot = -1;	//���ұ�ʾδֵ֪;
			chi.heuristic = abs(chi.i - destX) + abs(chi.j - destY) + 1;
			chi.parent = &start;
			searchTree.push_back(chi);
			nearPoints.push_back(&searchTree.back());
		}
		//�Ҳ��ڵ�
		if (isCanMove(start, start.enter_slot, end_slot, ResEntity_list, start.i, start.j, row = start.i, col = start.j + 1)
			&& !isInPath(path, row, col))
		{
			HTNPoint chi;
			chi.i = row, chi.j = col;
			chi.enter_slot = ResEntity_list[row][col].isFreePass(start.enter_slot, end_slot);
			chi.leave_slot = -1;	//���ұ�ʾδֵ֪;
			chi.heuristic = abs(chi.i - destX) + abs(chi.j - destY) + 1;
			chi.parent = &start;
			searchTree.push_back(chi);
			nearPoints.push_back(&searchTree.back());
		}

		if (nearPoints.empty())
		{
			HTNPoint em(-1, -1, -1, -1);
			searchTree.push_back(em);
			return searchTree.back();
		}
		else
		{
			//int  x = nearPoints.size();
			//cout<< x <<endl;
			nearPoints.sort(HTNPoint());
			list<HTNPoint *> ::iterator nit = nearPoints.begin();
			for (; nit != nearPoints.end(); ++nit)
			{
				path.push_back(**nit);
				HTNPoint tt = path.back();
				tt;
				HTNPoint search = searchNext(curr_slot, **nit, destX, destY, ResEntity_list, searchTree, path);
				if (search.i == destX)
				{
					searchTree.push_back(search);
					return searchTree.back();//�ѵ������
				}
				else path.pop_back();
			}

			//���е�·��������;
			HTNPoint em(-1, -1, -1, -1);
			searchTree.push_back(em);
			return searchTree.back();
		}
	}
}

//HTNѰ·�㷨
list <HTNPoint> HTNMove::findPath(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY)
{
	//·������Ѱ·��������
	list<HTNPoint> path;		//Ҫ���ص�����·��
	list<HTNPoint> pathTemp;	//�ݴ�����·��
	list<HTNPoint> searchTree;	//·��������

	this->rows = inRow;
	this->cols = inCol;
	//��̬������Ƕmap,isMapModified �����Խ�ʡ�ռ�
	map = new TileType *[rows];
	for (short i = 0; i < rows; ++i)
		map[i] = new TileType[cols];

	//���մ�������ֵ;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			map[i][j] = inMap[i][j];
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

	HTNPoint startPoint;
	startPoint.i = StartX;
	startPoint.j = StartY;
	startPoint.enter_slot = curr_slot;
	startPoint.leave_slot = -1;
	startPoint.heuristic = abs(startPoint.i - EndX) + abs(startPoint.j - EndY) + 1;
	startPoint.parent = nullptr;

	searchTree.push_back(startPoint);
	pathTemp.push_back(startPoint);
	//��Ѱ·��
	HTNPoint end = searchNext(curr_slot, startPoint, EndX, EndY, ResEntity_list, searchTree, pathTemp);
	if(end.i == -1)return path;//���ؿ�·��, endδ���յ�
	
	//������ǵĻ���˵����·���ҵ���
	HTNPoint chi;
	end.leave_slot = end.enter_slot;
	while (!(end.i == StartX && end.j == StartY))
	{
		map[end.i][end.j] = Tile_Path;
		chi = end;
		path.push_front(chi);
		end = * end.parent;	//����
		end.leave_slot = chi.enter_slot - 1;//�����뿪ʱ��			
	}
	path.push_front(end);//�����Ҳѹ��ȥ
	path.back().leave_slot++;//������䣬����ռ��ʱ��******************;
	
	return path;
}
