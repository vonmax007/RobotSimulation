// RobotSimulation.cpp : 定义控制台应用程序的入口点。
//

#include "StdAfx.h"


int main()
{
	//FILE *fp;
	//freopen_s(&fp, "2017-5-09-Experiment_log-01.txt", "w", stdout);
	//system("mode con cols=80 lines=10000");	//Windows下测试
	
	//_CrtSetBreakAlloc(1971923);
	ExperimentExecutor ee;
	ee.exec();
	ee.~ExperimentExecutor();
#ifndef Linx
	_CrtDumpMemoryLeaks();
#endif
    return 0;
}