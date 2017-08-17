
/*
	A*Ѱ·�㷨 by ������.
	15S151482
	vonmax007
	ͷ�ļ�
*/
#pragma once
#include "StdAfx.h"

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

//A�ǽڵ�ṹ��
struct AStarNode
{	
	int row;  //�ýڵ�������
	int col;  //�ýڵ�������
	int f, g, h;
	
	AStarNode * parent;
	AStarNode() :f(0), g(0), h(0),
		row(-1),col(-1), parent(nullptr) {}//Ĭ�Ϲ��캯��
};

//A*������
class AStar
{
public:
	AStar();
	~AStar();

	//������������;
	void modifyPath(list <Point> & path, int & StartX, int & StartY, int & EndX, int & EndY);
	list <list <Point>> findKPath(int k, TileType ** inMap, int inRow, int inCol, int StartX, int StartY, int EndX, int EndY);
	void PrintMap();//�����ã���ӡ��ͼ
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
	//�������ű�
	list <AStarNode> Open;
	list <AStarNode> Closed;
	int rows;
	int cols;
	int destinationRow;
	int destinationCol;
};