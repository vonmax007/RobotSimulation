#pragma once
#include "StdAfx.h"
#include "ResEntity.h"

#define MAXReah 800000	//��ʾ�޷�����Ŀ�ĵ�;


class Robot
{
public:
	Robot(void);
	~Robot(void);
	Robot(int & Rid, Point & sp, Point & dp, int & is);			//���캯��
	bool getRoute(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol);
	void printTW(ResEntity ** ResEntity_list, int & curr_slot);
	//Robot attributes
public:

	int Rid;			//�����˵�ID
	Point startPort;	//ԭʼ�������ID��������ɺ�Ҫ���ش˸��
	Point destPort;		//Ŀ�ķּ��ڵ�ID
	Point currPos;		//��ǰ����λ��
	bool stay = false;	//��ǰ�Ƿ���Ҫ�ȴ���
	int ignition_slot;	//�����˵�һ������ʱ��ʱ��Ƭ
	list <HTNPoint> currRoute;//��ǰ��ʻ·��;
	TileType isFinMission = 1;//�Ƿ���ɵ�ǰ����1�����������Ҫ����;
	//2�Ǵ��ڴ� �����ڵ� Ŀ�Ŀڵ�·�ϣ�3�Ǵ�Ŀ�Ŀ���Ҫ�������أ�4�Ǵ��ڷ��ص�·��;
	//5�����ڴ��ڻ�����״̬;
};
