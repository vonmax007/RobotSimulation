#pragma once
#include "StdAfx.h"
#include "Robot.h"
#include "ResEntity.h"


class VirtualEnvironment
{
public:
	VirtualEnvironment(void);
	~VirtualEnvironment(void);

	void CreateEnvSLSD();			//����ģ��һ�ĳ���
	void PrintMap(unsigned char ** map, int row, int col);//��ӡ��ͼ����
	void CreateRobot();
	//void CreateDest();
	void CreateDest_centralized_uniform();	//a. ����
	void CreateDest_global_uniform();		//b. ����
	void CreateDest_near_outport();			//c. ����
	void CreateDest_random();				//d. ����
	void CreateOutPort();
	void StepRobot();				//�������л����ˣ�ǰ��һ��
	void RunSimulation();			//ʵ����ѭ��,������ǰ��
	bool CheackCorrectness();	    //�㷨��ȷ�Լ���

public:
	int time_slot = 1;				//����ʱ���,���Ǽٶ�ʱ���1��ʼ����3600������
	ResEntity ** ResEntity_list;		//��Դ��ά����
	vector <Robot> robot_list;		//���������飬�±��ʾ��ID
	unsigned char ** real_map;		//������ʵ�ʵ�ͼ
	list <int> ** posMap;			//λ��ͳ�ƴ洢��ͼ
	Point * desSet;                 //Ŀ�ĵؿڼ���
	Point * pitSet;                 //�ּ�ڼ���
	Point * bufSet;                 //����������
	Point * outPortSet;             //�����ڼ���
	list <int> Destinations;		//����Ŀ�ĵؼ���

	//ʵ����Ʋ���
	int map_row = 4, map_col = 4;	//Ĭ�ϵ�ͼ��С//Ĭ����4��*4�У� �����лᵼ�����½ǡ�����ͻ��  ż���������лᵼ�����½ǡ�����ͻ������ֻ����ż����ż����
	int runSlots = 320;				//Ĭ���������320��ʱ��Ƭ
	int numRobot = 4;				//�����������ĳ�ʼֵ
	int numDestination = 4;			//Ĭ��������4��Ŀ�ĵ�
	int numOutPort = 4;				//Ĭ��������4��������

	//ʵ��ͳ������
	int totalCount = 0;				//�����ʹ��������ͳ��,��ʵ�������Ŀ��
	int failRCount = 0;				//������ʧ��ͳ��
	int maxActiveRobots = 0;		//���������ɻ������ͳ��
};
