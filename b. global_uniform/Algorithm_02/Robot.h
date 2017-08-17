#pragma once
#include "StdAfx.h"
#include "ResEntity.h"

#define MAXReah 800000	//��ʾ�޷�����Ŀ�ĵ�;


//Ѱ�ҿ���ʱ��۵Ķ��нṹ
struct retime
{
	int i, j;
	int enter_slot;
	int leave_slot;
	
	list <FTW_item> FTWTable;
	list <FTW_item>::iterator itw;		//·��ʱ�䴰ָʾ��
	retime(int _i, int _j) :
		i(_i), j(_j), enter_slot(-1), leave_slot(-1) {}
};

//��¼·��-ʱ���;
struct TPoint
{
	int i, j;
	int enter_slot;
	int leave_slot;
	TPoint() {}
	TPoint(int i, int j, int en, int le) :
		i(i), j(j), enter_slot(en), leave_slot(le) {}
};

class Robot
{
public:
	Robot(void);
	~Robot(void);
	Robot(int & Rid, Point & sp, Point & dp, int & is);			//���캯��
	void findKPath(TileType ** inMap, int inRow, int inCol);//ÿ�θ���·�����㷨
	int getEarliestArr(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot);
	void selectBestRoute(ResEntity ** ResEntity_list, int & curr_slot);
	list <retime> findTW(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot);
	bool reserveRoute(ResEntity ** ResEntity_list, int & curr_slot);//����ԤԼ��·��
	void printTW(ResEntity ** ResEntity_list, int & curr_slot);
	//Robot attributes
public:

	int Rid;				//�����˵�ID
	Point startPort;		//ԭʼ�������ID��������ɺ�Ҫ���ش˸��
	Point destPort;			//Ŀ�ķּ��ڵ�ID
	Point currPos;			//��ǰ����λ��

	int ignition_slot;		//�����˵�һ������ʱ��ʱ��Ƭ
	int k = K_way;			//ÿ��Ѱ�ҵ�·������k,����Ĭ��Ϊ10��������Ҫ��֤�Ĳ���
	list <list <Point>> ps; //ÿ�λ������ҵ���·������;
	list <Point> currRoute; //��ǰ��ʻ·��;
	list <TPoint> currRouteTime; //��ǰ��ʻ·��-ʱ��;
	bool stay = false;		//��ǰ�Ƿ���Ҫ�ȴ���
	TileType isFinMission = 1;//�Ƿ���ɵ�ǰ����1�����������Ҫ����;
	//2�Ǵ��ڴ� �����ڵ� Ŀ�Ŀڵ�·�ϣ�3�Ǵ�Ŀ�Ŀ���Ҫ�������أ�4�Ǵ��ڷ��ص�·��;
	//5�����ڴ��ڻ�����״̬;
};
