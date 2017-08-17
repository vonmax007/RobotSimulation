#pragma once
#include "StdAfx.h"
#include "ResEntity.h"

#define MAXReah 800000	//��ʾ�޷�����Ŀ�ĵ�;

//Ѱ�ҿ���ʱ��۵Ķ��нṹ
struct retime
{
	int i, j;
	int enter_slot;
	int end_slot;	//��λ�ڵĿ���ʱ�䴰�Ľ���ʱ���
	retime * backpointer;//����ָ�룬����ָʾ�˽ṹ��չ����һ����Դ-ʱ���
	retime(int _i, int _j, int _enter_slot, int _end_slot):i(_i), j(_j), enter_slot(_enter_slot), end_slot (_end_slot) {}
	retime(int _i, int _j, int _enter_slot, int _end_slot, retime * _backpointer) :
		i(_i), j(_j), enter_slot(_enter_slot), end_slot(_end_slot), backpointer(_backpointer){}
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
	bool reserveRoute(ResEntity ** ResEntity_list, int & curr_slot);//����ԤԼ��·��
	void printTW(ResEntity ** ResEntity_list, int & curr_slot);
	bool isSamePath(list <Point> & path1, list <Point> & path2);
	//Robot attributes
public:

	int Rid;				//�����˵�ID
	Point startPort;		//ԭʼ�������ID��������ɺ�Ҫ���ش˸��
	Point destPort;			//Ŀ�ķּ��ڵ�ID
	Point currPos;			//��ǰ����λ��

	int ignition_slot;		//�����˵�һ������ʱ��ʱ��Ƭ
	int k = K_way;			//ÿ��Ѱ�ҵ�·������k,����Ĭ��Ϊ10��������Ҫ��֤�Ĳ���
	//int tltRef = 0;			//time left to refresh������ʱȥˢ�µ�ʱ��
	list <list <Point>> ps; //ÿ�λ������ҵ���·������;
	list <Point> currRoute; //��ǰ��ʻ·��;
	list <TPoint> currRouteTime; //��ǰ��ʻ·��-ʱ��;
	bool stay = false;		  //��ǰ�Ƿ���Ҫ�ȴ���
	TileType isFinMission = 1;//�Ƿ���ɵ�ǰ����1�����������Ҫ����;
	//2�Ǵ��ڴ� �����ڵ� Ŀ�Ŀڵ�·�ϣ�3�Ǵ�Ŀ�Ŀ���Ҫ�������أ�4�Ǵ��ڷ��ص�·��;
	//5�����ڴ��ڻ�����״̬;
};
