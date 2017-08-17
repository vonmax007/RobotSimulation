
#include "StdAfx.h"
#include "ExperimentExecutor.h"


ExperimentExecutor::ExperimentExecutor(void)
{
	//cout<<"Welcome to Robot simulation main Interface \n";
}


ExperimentExecutor::~ExperimentExecutor(void)
{
}

void ExperimentExecutor::exec()		//实验主循环界面;
{
	//实验控制参数
	//int map_row = 4, map_col = 4;		//默认地图大小//默认是4行*4列， 奇数行会导致左下角↓←冲突，  偶数行奇数列会导致右下角→↓冲突，所以只能是偶数行偶数列
	//int runSlots = 320;				//默认最低运行320个时间片
	//int numRobot = 4;					//机器人数量的初始值
	//int numDestination = 4;			//默认至少有4个目的地
	//int numOutPort = 4;				//默认至少有4个出货口

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

			if(Erow_col == 4)ve->numOutPort = 4;	//设置场地出货口数目;
			else if(Erow_col == 6)ve->numOutPort = 8;
			else if(Erow_col == 8)ve->numOutPort = 16;
			else ve->numOutPort = (Erow_col - 2) * 3;	//普通递增;

			if(Erow_col == 4)ve->numDestination = 4;	//设置场地分拣口数目;
			else if(Erow_col == 6)ve->numDestination = 16;
			else if(Erow_col == 8)ve->numDestination = 40;
			else ve->numDestination = (Erow_col - 2) * 15;	//普通递增;
			ve->CreateEnvSLSD();
			ve->RunSimulation();
			fout << Erow_col << "\t" << Erobot << "\t" << ve->totalCount << "\t" << ve->maxActiveRobots << "\t" << ve->failRCount << endl;

			delete ve;
			ve = nullptr;
		}
		fout.close();
	}
}
