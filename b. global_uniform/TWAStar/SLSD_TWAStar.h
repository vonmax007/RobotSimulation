#pragma once
#include "StdAfx.h"
#include "ResEntity.h"

/*
	���ڴ�ʱ�䴰A*�Ķ�����˹滮�㷨
	���ղ�������ͨ��A*��һЩ����;
	15S151482
	vonmax007
*/


#define MAXFAIL 5

#define G_OFFSET1 10			//ÿ��ͼ��Gֵ������ֵ
#define TileSize  1				//ͼ���С
#define Tile_Lock 0
#define Tile_Open 1
#define Tile_Start 2
#define Tile_End  3
#define Tile_Path 4
#define PenaltyD 5				//·��Զ���Զ��ʼ�ͷ�
#define PenaltyScale 2			//�ͷ����ʣ���������Ϊ2
//����ָʾλ�Ķ���;
#define Tile_UP  100			//��
#define Tile_DOWN  101			//��
#define Tile_LEFT  102			//��
#define Tile_RIGHT  103			//��
#define Tile_UP_RIGHT  104		//����
#define Tile_UP_LEFT  105		//����
#define Tile_DOWN_RIGHT  106	//����
#define Tile_DOWN_LEFT  107		//����

//A�ǽڵ�ṹ��
struct AStarNode
{
	int row;  //�ýڵ�������
	int col;  //�ýڵ�������
	int f, g, h;

	AStarNode * parent;
	AStarNode() :f(0), g(0), h(0),
		row(-1), col(-1), parent(nullptr) {}//Ĭ�Ϲ��캯��
};


//·���ڵ�;
struct TWASPoint
{
	int i, j;
	int enter_slot, leave_slot;
	TWASPoint() {}
	TWASPoint(int _i, int _j, int _enter_slot, int _leave_slot) :
		i(_i), j(_j), enter_slot(_enter_slot), leave_slot(_leave_slot) {}
};


//�Ľ��Ĵ�ʱ�䴰��A*�㷨
class SLSD_TWAStar
{
public:
	SLSD_TWAStar();
	~SLSD_TWAStar();
	
	list <TWASPoint> findPathTW(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY, Point * buff, int numDest);
	void PrintMap();//�����ã���ӡ��ͼ
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
	//�������ű�
	list <AStarNode> Open;
	list <AStarNode> Closed;
	int rows;
	int cols;
	int destinationRow;
	int destinationCol;
};