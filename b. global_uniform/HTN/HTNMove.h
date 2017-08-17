#pragma once
#include "StdAfx.h"
#include "ResEntity.h"

/*
	����HTN�滮�Ķ�����˹滮�㷨
	by 15S151482, ������
*/

#define MAXREACH  80000
#define LoopLength  1000
#define MAXFAIL 5

#define G_OFFSET1 10			//ÿ��ͼ��Gֵ������ֵ
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

//�㷨�����ڵ�ṹ
struct HTNPoint
{
	int i, j;
	int enter_slot, leave_slot;
	int heuristic;				//�ô˱�ʾ����ֵ����Ŀ�ĵ�ľ���
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
	//HTN���������
	bool isInPath(list<HTNPoint> & searchTree, int & si, int & sj);
	list <HTNPoint> findPath(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY);
	void PrintMap();//�����ã���ӡ��ͼ
	bool isCanMove(HTNPoint start, int enter_slot, int end_slot, ResEntity ** ResEntity_list, int frow, int fcol, int row, int col);
	HTNPoint & searchNext(int & curr_slot, HTNPoint & start, int destX, int destY, ResEntity ** ResEntity_list, list<HTNPoint> & searchTree, list<HTNPoint> & path);
	

private:
	TileType ** map;
	int rows;
	int cols;
	int destinationRow;
	int destinationCol;

};
