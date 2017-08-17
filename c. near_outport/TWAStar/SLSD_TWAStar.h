#pragma once
#include "StdAfx.h"
#include "ResEntity.h"

/*
	基于带时间窗A*的多机器人规划算法
	参照并借用了通用A*的一些操作;
	15S151482
	vonmax007
*/


#define MAXFAIL 5

#define G_OFFSET1 10			//每个图块G值的增加值
#define TileSize  1				//图块大小
#define Tile_Lock 0
#define Tile_Open 1
#define Tile_Start 2
#define Tile_End  3
#define Tile_Path 4
#define PenaltyD 5				//路径远离多远开始惩罚
#define PenaltyScale 2			//惩罚倍率，这里设置为2
//方向指示位的定义;
#define Tile_UP  100			//↑
#define Tile_DOWN  101			//↓
#define Tile_LEFT  102			//←
#define Tile_RIGHT  103			//→
#define Tile_UP_RIGHT  104		//↑→
#define Tile_UP_LEFT  105		//←↑
#define Tile_DOWN_RIGHT  106	//↓→
#define Tile_DOWN_LEFT  107		//←↓

//A星节点结构体
struct AStarNode
{
	int row;  //该节点所在行
	int col;  //该节点所在列
	int f, g, h;

	AStarNode * parent;
	AStarNode() :f(0), g(0), h(0),
		row(-1), col(-1), parent(nullptr) {}//默认构造函数
};


//路径节点;
struct TWASPoint
{
	int i, j;
	int enter_slot, leave_slot;
	TWASPoint() {}
	TWASPoint(int _i, int _j, int _enter_slot, int _leave_slot) :
		i(_i), j(_j), enter_slot(_enter_slot), leave_slot(_leave_slot) {}
};


//改进的带时间窗的A*算法
class SLSD_TWAStar
{
public:
	SLSD_TWAStar();
	~SLSD_TWAStar();
	
	list <TWASPoint> findPathTW(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY, Point * buff, int numDest);
	void PrintMap();//测试用，打印地图
	AStarNode getNodeFromOpen();
	AStarNode checkOpen(int row, int col);
	int getH(int row, int col);
	bool isInClose(int row, int col);
	void creatNextLapNode(AStarNode & bestNode, int row, int col, int G_OFFSET);
	list <AStarNode>::iterator getMinVal();
	bool isCanMove(int enter_slot, int end_slot, ResEntity ** ResEntity_list, int frow, int fcol, int row, int col, Point * buff, int numDest);
	void seachSeccessionNode(AStarNode & bestNode, int enter_slot, int end_slot, ResEntity ** ResEntity_list, Point * buff, int numDes);

private:
	TileType ** map;
	//保持两张表
	list <AStarNode> Open;
	list <AStarNode> Closed;
	int rows;
	int cols;
	int destinationRow;
	int destinationCol;
};