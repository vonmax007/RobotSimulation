#pragma once
#include "StdAfx.h"
#include "ResEntity.h"

/*
	基于HTN规划的多机器人规划算法
	by 15S151482, 凡振宇
*/

#define MAXREACH  80000
#define LoopLength  1000
#define MAXFAIL 5

#define G_OFFSET1 10			//每个图块G值的增加值
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

//算法搜索节点结构
struct HTNPoint
{
	int i, j;
	int enter_slot, leave_slot;
	int heuristic;				//用此表示启发值，到目的点的距离
	HTNPoint * parent;
	bool operator()(HTNPoint * h1, HTNPoint * h2)
	{
		return (h1->heuristic < h2->heuristic);
	}
	HTNPoint() {}
	HTNPoint(int _i, int _j, int _enter_slot, int _leave_slot) :
		i(_i), j(_j), enter_slot(_enter_slot), leave_slot(_leave_slot) {}
};


class HTNMove
{
public:
	HTNMove();
	~HTNMove();
public:
	//HTN类操作函数
	bool isInPath(list<HTNPoint> & searchTree, int & si, int & sj);
	list <HTNPoint> findPath(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY);
	void PrintMap();//测试用，打印地图
	bool isCanMove(HTNPoint start, int enter_slot, int end_slot, ResEntity ** ResEntity_list, int frow, int fcol, int row, int col);
	HTNPoint & searchNext(int & curr_slot, HTNPoint & start, int destX, int destY, ResEntity ** ResEntity_list, list<HTNPoint> & searchTree, list<HTNPoint> & path);
	

private:
	TileType ** map;
	int rows;
	int cols;
	int destinationRow;
	int destinationCol;

};
