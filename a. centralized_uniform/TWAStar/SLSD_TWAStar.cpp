#include "StdAfx.h"
#include "SLSD_TWAStar.h"

SLSD_TWAStar::SLSD_TWAStar()
{
}

SLSD_TWAStar::~SLSD_TWAStar()
{
	//������
	//�ͷ���Ƕmap,isMapModified ��ά����ռ�
	for (short i = 0; i < rows; ++i)
		delete[] map[i];		
	delete[] map;
}



void SLSD_TWAStar::PrintMap()
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
AStarNode SLSD_TWAStar::getNodeFromOpen()
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
	minx = *minit;
	Open.erase(minit++);
	return minx;
}

//�õ�Hֵ����;
int SLSD_TWAStar::getH(int row, int col)
{
	//�õ���ͼ���Hֵ,�����������پ���/�������ٶ�;
	int h = abs(destinationRow - row) + abs(destinationCol - col);
	return h;	//�����h��ʾ���Ƶ���Ŀ�ĵ�ʱ��
}

bool isBuff(int row, int col, Point * buff, int numDest)
{
	for (int i = 0; i < numDest; ++i)
	{
		if (buff[i].i == row && buff[i].j == col)return true;
	}
	return false;
}


//����ͼ���Ƿ��ͨ��
bool SLSD_TWAStar::isCanMove(int enter_slot, int end_slot, ResEntity ** ResEntity_list, int frow, int fcol, int row, int col, Point * buff, int numDest)
{
	if (col < 0 || col >= cols)
		return false;
	if (row < 0 || row >= rows)
		return false;
	if (map[row][col] == Tile_Lock)
		return false;

	if (isBuff(frow, fcol, buff, numDest) && !(fcol == col + 1 && frow == row))return false;
	if (isBuff(row, col, buff, numDest) && !(fcol == col - 1 && frow == row))return false;

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

	//�˴�Ϊ�����·�����Ҳ��ж��Ƿ��н�����ͻ
	if ( ResEntity_list[row][col].isFreePass(enter_slot, end_slot) == -1 )
		return false;	//���ж��Ƿ�ʱ�䴰�ɴ�;�˺�������ֵ-1��ʾ����ͨ��;

	return true;
}

//У��Open��,��ʵ���ǲ��Ҵ��к��еĽڵ�ʵ��;
AStarNode SLSD_TWAStar::checkOpen(int row, int col)
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
bool SLSD_TWAStar::isInClose(int row, int col)
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

void SLSD_TWAStar::creatNextLapNode(AStarNode & bestNode, int row, int col, int G_OFFSET)
{//������һȦ��node
	int g = G_OFFSET;	//�����gָ�����״ε���˽ڵ�ʱ���
	if (!isInClose(row, col))
	{
		AStarNode oldNode;
		if ((oldNode = checkOpen(row, col)).col != -1)
		{
			if (oldNode.g > g)
			{
				oldNode.parent = &bestNode;
				oldNode.g = g;
				oldNode.f = g + oldNode.h;
			}
		}
		else
		{
			AStarNode node;
			node.parent = &bestNode;
			node.g = g;
			node.h = getH(row, col);
			node.f = node.g + node.h;
			node.row = row;
			node.col = col;

			this->Open.push_back(node);
		}
	}
}

void SLSD_TWAStar::seachSeccessionNode(AStarNode & bestNode, int enter_slot, int end_slot, ResEntity ** ResEntity_list, Point * buff, int numDest)
{
	//���ݴ���Ľڵ������ӽڵ�
	int row, col;
	//�ϲ��ڵ�
	if (isCanMove(enter_slot, end_slot, ResEntity_list, bestNode.row, bestNode.col, row = bestNode.row - 1, col = bestNode.col, buff, numDest))
	{
		int next_enter_slot = ResEntity_list[row][col].isFreePass(enter_slot, end_slot);
		creatNextLapNode(bestNode, row, col, next_enter_slot);
	}
	//�²��ڵ�
	if (isCanMove(enter_slot, end_slot, ResEntity_list, bestNode.row, bestNode.col, row = bestNode.row + 1, col = bestNode.col, buff, numDest))
	{
		int next_enter_slot = ResEntity_list[row][col].isFreePass(enter_slot, end_slot);
		creatNextLapNode(bestNode, row, col, next_enter_slot);
	}
	//�󲿽ڵ�
	if (isCanMove(enter_slot, end_slot, ResEntity_list, bestNode.row, bestNode.col, row = bestNode.row, col = bestNode.col - 1, buff, numDest))
	{
		int next_enter_slot = ResEntity_list[row][col].isFreePass(enter_slot, end_slot);
		creatNextLapNode(bestNode, row, col, next_enter_slot);
	}
	//�Ҳ��ڵ�
	if (isCanMove(enter_slot, end_slot, ResEntity_list, bestNode.row, bestNode.col, row = bestNode.row, col = bestNode.col + 1, buff, numDest))
	{
		int next_enter_slot = ResEntity_list[row][col].isFreePass(enter_slot, end_slot);
		creatNextLapNode(bestNode, row, col, next_enter_slot);
	}
	
	this->Closed.push_back(bestNode);
}

//�ڴ�ʱ�䴰��·���У���A*��˼��Ѱ·;
//findPathTW: find path in time windows
list <TWASPoint> SLSD_TWAStar::findPathTW(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY, Point * buff, int numDest)
{
	//������,�ڴ�ʱ�䴰�Ļ���������Ѱ·��
	list<TWASPoint> path;	//Ҫ���ص�����·��

	rows = inRow;
	cols = inCol;
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

	list <AStarNode> OpenSave;
	AStarNode startNode;
	AStarNode bestNode;
	destinationRow = EndX;
	destinationCol = EndY;

	startNode.parent = NULL;
	startNode.row = StartX;
	startNode.col = StartY;
	startNode.g = curr_slot;		//������gֵ
	startNode.h = getH(startNode.row, startNode.col);
	startNode.f = startNode.g + startNode.h;
	
	this->Open.push_back(startNode);

	TWASPoint point;
	while (true)
	{
		bestNode = getNodeFromOpen(); //��OPEN����ȡ��fֵ��С�Ľڵ�
		OpenSave.push_back(bestNode);
		if (bestNode.col == -1)//δ�ҵ�·��
		{
			OpenSave.clear();
			break;
		}
		else if (bestNode.row == destinationRow && bestNode.col == destinationCol)
		{
			map[bestNode.row][bestNode.col] = Tile_Path;
			point.i = bestNode.row;
			point.j = bestNode.col;
			point.enter_slot = bestNode.g;
			point.leave_slot = bestNode.g;
			int last_enter_slot = bestNode.g;
			path.push_front(point);//ʵ������ȥ��,��Ȼ�ᵽ�����һ���������۷�
			while ( bestNode.parent != nullptr )
			{
				map[bestNode.row][bestNode.col] = Tile_Path;
				
				point.i = bestNode.row;
				point.j = bestNode.col;
				point.enter_slot = bestNode.g;
				point.leave_slot = last_enter_slot - 1;//�˵��뿪ʱ��Ϊ·����һ�����ʱ��-1
				last_enter_slot = point.enter_slot;		//������һ�����ʱ��
				path.push_front(point);

				bestNode = *bestNode.parent;
			}
			path.push_front(TWASPoint(StartX, StartY, curr_slot, last_enter_slot - 1));//�����Ҳѹ��ȥ
			path.back().leave_slot++;//����һʱ��۵�Ͷ�ݻ�װ�ص�ռ��ʱ��;*************************
			break;
		}

		int end_slot = ResEntity_list[bestNode.row][bestNode.col].locatedslot(bestNode.g);
		seachSeccessionNode(OpenSave.back(), OpenSave.back().g, end_slot, ResEntity_list, buff, numDest);
	}
	
	this->Open.clear();
	this->Closed.clear();
	OpenSave.clear();

	return path;
}



list <AStarNode>::iterator SLSD_TWAStar::getMinVal()
{
	if (this->Open.size() == 0)return this->Open.end();
	else if (this->Open.size() == 1)return this->Open.begin();
	else
	{
		AStarNode minx;//Ҫ���ص���Сֵ�ڵ�ָ��;
		int minF = this->Open.begin()->f;
		int minFF = (++this->Open.begin())->f;
		list <AStarNode>::iterator minit = this->Open.begin();
		list <AStarNode>::iterator mminit = this->Open.begin(); ++mminit;
		if (minF > minFF)
		{
			int temp = minF;
			minF = minFF;
			minFF = temp;
			list <AStarNode>::iterator it = minit;
			minit = mminit;
			mminit = it;
		}
		list <AStarNode>::iterator oit = ++++this->Open.begin();
		for (; oit != Open.end(); ++oit)
		{
			if (oit->f > minFF);
			else if (oit->f > minF && oit->f < minFF)
			{
				minFF = oit->f;
				mminit = oit;
			}
			else 
			{
				minFF = minF;
				minF = oit->f;
				mminit = minit;
				minit = oit;

			}
		}
		return mminit;
	}
}