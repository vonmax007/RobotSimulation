#include "StdAfx.h"
#include "SLSD_TWAStar.h"

SLSD_TWAStar::SLSD_TWAStar()
{
}

SLSD_TWAStar::~SLSD_TWAStar()
{
	//清理步骤
	//释放内嵌map,isMapModified 二维数组空间
	for (short i = 0; i < rows; ++i)
		delete[] map[i];		
	delete[] map;
}



void SLSD_TWAStar::PrintMap()
{	//输出地图
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
				printf("□");
			}
			else if (map[i][j] == Tile_Lock) {
				printf("■");
			}
			else if (map[i][j] == Tile_Start) {
				printf("☆");
			}
			else if (map[i][j] == Tile_End) {
				printf("★");
			}
			else if (map[i][j] == Tile_Path) {
				printf("●");
			}
		}
		printf("\n");
	}
	printf("\n\n");
}

//选取Open表上f值最小的节点，返回该节点地址
AStarNode SLSD_TWAStar::getNodeFromOpen()
{	
	if (this->Open.size() == 0)return AStarNode();
	AStarNode minx;//要返回的最小值AStar节点的指针;
	int minF = this->Open.begin()->f;
	list <AStarNode>::iterator oit = this->Open.begin();
	list <AStarNode>::iterator minit = this->Open.begin();
	for (; oit != Open.end(); ++oit)
	{
		if (oit->f < minF)
		{
			minit = oit;	//min表示f最小的链表节
			minF = oit->f;	//指向min的指针，为的是在后面的释放中用到
		}
	}
	minx = *minit;
	Open.erase(minit++);
	return minx;
}

//得到H值函数;
int SLSD_TWAStar::getH(int row, int col)
{
	//得到该图块的H值,这里是曼哈顿距离/机器人速度;
	int h = abs(destinationRow - row) + abs(destinationCol - col);
	return h;	//这里的h表示估计到达目的地时间
}

bool isBuff(int row, int col, Point * buff, int numDest)
{
	for (int i = 0; i < numDest; ++i)
	{
		if (buff[i].i == row && buff[i].j == col)return true;
	}
	return false;
}


//检测该图块是否可通行
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
		)return false;	//访问不可通行上方

	if ((row == frow + 1 && col == fcol) &&
		(map[row][col] == Tile_UP ||
			map[row][col] == Tile_UP_LEFT ||
			map[row][col] == Tile_UP_RIGHT)
		)return false;	//访问不可通行下方

	if ((row == frow && col == fcol - 1) &&
		(map[row][col] == Tile_RIGHT ||
			map[row][col] == Tile_UP_RIGHT ||
			map[row][col] == Tile_DOWN_RIGHT)
		)return false;	//访问不可通行左方

	if ((row == frow && col == fcol + 1) &&
		(map[row][col] == Tile_LEFT ||
			map[row][col] == Tile_UP_LEFT ||
			map[row][col] == Tile_DOWN_LEFT)
		)return false;	//访问不可通行右方

	//此处为单向道路，暂且不判断是否有交换冲突
	if ( ResEntity_list[row][col].isFreePass(enter_slot, end_slot) == -1 )
		return false;	//先判断是否时间窗可达;此函数返回值-1表示不能通行;

	return true;
}

//校验Open表,其实就是查找此行和列的节点实体;
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

//是否存在于Close表
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
{//创建下一圈的node
	int g = G_OFFSET;	//这个的g指的是首次到达此节点时间槽
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
	//根据传入的节点生成子节点
	int row, col;
	//上部节点
	if (isCanMove(enter_slot, end_slot, ResEntity_list, bestNode.row, bestNode.col, row = bestNode.row - 1, col = bestNode.col, buff, numDest))
	{
		int next_enter_slot = ResEntity_list[row][col].isFreePass(enter_slot, end_slot);
		creatNextLapNode(bestNode, row, col, next_enter_slot);
	}
	//下部节点
	if (isCanMove(enter_slot, end_slot, ResEntity_list, bestNode.row, bestNode.col, row = bestNode.row + 1, col = bestNode.col, buff, numDest))
	{
		int next_enter_slot = ResEntity_list[row][col].isFreePass(enter_slot, end_slot);
		creatNextLapNode(bestNode, row, col, next_enter_slot);
	}
	//左部节点
	if (isCanMove(enter_slot, end_slot, ResEntity_list, bestNode.row, bestNode.col, row = bestNode.row, col = bestNode.col - 1, buff, numDest))
	{
		int next_enter_slot = ResEntity_list[row][col].isFreePass(enter_slot, end_slot);
		creatNextLapNode(bestNode, row, col, next_enter_slot);
	}
	//右部节点
	if (isCanMove(enter_slot, end_slot, ResEntity_list, bestNode.row, bestNode.col, row = bestNode.row, col = bestNode.col + 1, buff, numDest))
	{
		int next_enter_slot = ResEntity_list[row][col].isFreePass(enter_slot, end_slot);
		creatNextLapNode(bestNode, row, col, next_enter_slot);
	}
	
	this->Closed.push_back(bestNode);
}

//在带时间窗的路径中，用A*的思想寻路;
//findPathTW: find path in time windows
list <TWASPoint> SLSD_TWAStar::findPathTW(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY, Point * buff, int numDest)
{
	//主函数,在带时间窗的环境网中搜寻路径
	list<TWASPoint> path;	//要返回的搜索路径

	rows = inRow;
	cols = inCol;
	//动态创建内嵌map,isMapModified 对象，以节省空间
	map = new TileType *[rows];
	for (short i = 0; i < rows; ++i)
		map[i] = new TileType[cols];
		

	//接收传进来的值;
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
	startNode.g = curr_slot;		//出发点g值
	startNode.h = getH(startNode.row, startNode.col);
	startNode.f = startNode.g + startNode.h;
	
	this->Open.push_back(startNode);

	TWASPoint point;
	while (true)
	{
		bestNode = getNodeFromOpen(); //从OPEN表中取出f值最小的节点
		OpenSave.push_back(bestNode);
		if (bestNode.col == -1)//未找到路径
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
			path.push_front(point);//实际运行去掉,不然会到达最后一个格子再折返
			while ( bestNode.parent != nullptr )
			{
				map[bestNode.row][bestNode.col] = Tile_Path;
				
				point.i = bestNode.row;
				point.j = bestNode.col;
				point.enter_slot = bestNode.g;
				point.leave_slot = last_enter_slot - 1;//此点离开时刻为路径下一点进入时刻-1
				last_enter_slot = point.enter_slot;		//更新下一点进入时间
				path.push_front(point);

				bestNode = *bestNode.parent;
			}
			path.push_front(TWASPoint(StartX, StartY, curr_slot, last_enter_slot - 1));//把起点也压进去
			path.back().leave_slot++;//增加一时间槽的投递或装载的占用时间;*************************
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
		AStarNode minx;//要返回的最小值节点指针;
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