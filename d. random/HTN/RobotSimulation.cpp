// RobotSimulation.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "StdAfx.h"


int main()
{
	//FILE *fp;
	//freopen_s(&fp, "E:\\Projects\\2017-5-17-Experiment_log-01.txt", "w", stdout);
	//system("mode con cols=800 lines=10000");	//Windows�²���
	
	//_CrtSetBreakAlloc(1971923);
	ExperimentExecutor ee;
	ee.exec();
	ee.~ExperimentExecutor();

#ifndef Linx
	_CrtDumpMemoryLeaks();
#endif
    return 0;
}