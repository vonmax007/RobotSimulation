
/*
	A*寻路算法 by 凡振宇.
	15S151482
	vonmax007
	头文件
*/
#pragma once
#include "StdAfx.h"

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

//A星节点结构体
struct AStarNode
{	
	int row;  //该节点所在行
	int col;  //该节点所在列
	int f, g, h;
	
	AStarNode * parent;
	AStarNode() :f(0), g(0), h(0),
		row(-1),col(-1), parent(nullptr) {}//默认构造函数
};

//A*类声明
class AStar
{
public:
	AStar();
	~AStar();

	//操作函数集合;
	void modifyPath(list <Point> & path, int & StartX, int & StartY, int & EndX, int & EndY);
	list <list <Point>> findKPath(int k, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY);
	void PrintMap();//测试用，打印地图
	AStarNode getNodeFromOpen();
	AStarNode checkOpen(int row, int col);
	int getH(int row, int col);
	bool isInClose(int row, int col);
	void creatNextLapNode(AStarNode & bestNode, int row, int col, int G_OFFSET);
	bool isSamePath(list <Point> & path1, list <Point> & path2);
	bool isPathExisted(list <list <Point>> & PathSet, list <Point> & path);
	bool isCanMove(int frow, int fcol, int row, int col);
	void seachSeccessionNode(AStarNode & bestNode);

private:
	TileType ** map;
	TileType ** isMapModified;
	//保持两张表
	list <AStarNode> Open;
	list <AStarNode> Closed;
	int rows;
	int cols;
	int destinationRow;
	int destinationCol;
};