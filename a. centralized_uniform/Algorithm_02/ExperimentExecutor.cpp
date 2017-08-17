
#include "StdAfx.h"
#include "ExperimentExecutor.h"


ExperimentExecutor::ExperimentExecutor(void)
{
	//cout<<"Welcome to Robot simulation main Interface \n";
}


ExperimentExecutor::~ExperimentExecutor(void)
{
}

void ExperimentExecutor::exec()		//ʵ����ѭ������;
{
	//ʵ����Ʋ���
	//int map_row = 4, map_col = 4;		//Ĭ�ϵ�ͼ��С//Ĭ����4��*4�У� �����лᵼ�����½ǡ�����ͻ��  ż���������лᵼ�����½ǡ�����ͻ������ֻ����ż����ż����
	//int runSlots = 320;				//Ĭ���������320��ʱ��Ƭ
	//int numRobot = 4;					//�����������ĳ�ʼֵ
	//int numDestination = 4;			//Ĭ��������4��Ŀ�ĵ�
	//int numOutPort = 4;				//Ĭ��������4��������

	//"Size_row_col"  "num_Robot"  "Cargo_Count"  "maxActiveRobots"  "Robot_fail_Count";
	for (int Erow_col = 4; Erow_col <= 22; Erow_col += 2)
	{
		string ExpName = "SLSD_Alg02_";
		ExpName += "size_" + to_string(Erow_col) + "_.txt";
		ofstream fout(ExpName);
		for (int Erobot = 1500; Erobot <= 2000; Erobot += 500)
		{
			VirtualEnvironment * ve = new VirtualEnvironment();
			ve->map_row = Erow_col;
			ve->map_col = Erow_col;
			ve->numRobot = Erobot;

			if(Erow_col == 4)ve->numOutPort = 4;	//���ó��س�������Ŀ;
			else if(Erow_col == 6)ve->numOutPort = 8;
			else if(Erow_col == 8)ve->numOutPort = 16;
			else ve->numOutPort = (Erow_col - 2) * 3;	//��ͨ����;

			if(Erow_col == 4)ve->numDestination = 4;	//���ó��طּ����Ŀ;
			else if(Erow_col == 6)ve->numDestination = 16;
			else if(Erow_col == 8)ve->numDestination = 40;
			else ve->numDestination = (Erow_col - 2) * 15;	//��ͨ����;
			ve->CreateEnvSLSD();
			ve->RunSimulation();
			fout << Erow_col << "\t" << Erobot << "\t" << ve->totalCount << "\t" << ve->maxActiveRobots << "\t" << ve->failRCount << endl;

			delete ve;
			ve = nullptr;
		}
		fout.close();
	}
}
