#pragma once
#include "StdAfx.h"
#include "Robot.h"
#include "ResEntity.h"


class VirtualEnvironment
{
public:
	VirtualEnvironment(void);
	~VirtualEnvironment(void);

	void CreateEnvSLSD();			//产生模拟一的场景
	void PrintMap(unsigned char ** map, int row, int col);//打印地图函数
	void CreateRobot();
	//void CreateDest();
	void CreateDest_centralized_uniform();	//a. 类型
	void CreateDest_global_uniform();		//b. 类型
	void CreateDest_near_outport();			//c. 类型
	void CreateDest_random();				//d. 类型
	void CreateOutPort();
	void StepRobot();				//单步运行机器人，前进一步
	void RunSimulation();			//实验主循环,机器人前进
	bool CheackCorrectness();	    //算法正确性检验

public:
	int time_slot = 1;				//运行时间槽,我们假定时间从1开始，到3600结束吧
	ResEntity ** ResEntity_list;		//资源二维数组
	vector <Robot> robot_list;		//机器人数组，下标表示其ID
	unsigned char ** real_map;		//创建的实际地图
	list <int> ** posMap;			//位置统计存储地图
	Point * desSet;                 //目的地口集合
	Point * pitSet;                 //分拣口集合
	Point * bufSet;                 //缓冲区集合
	Point * outPortSet;             //出货口集合
	list <int> Destinations;		//载入目的地集合

	//实验控制参数
	int map_row = 4, map_col = 4;	//默认地图大小//默认是4行*4列， 奇数行会导致左下角↓←冲突，  偶数行奇数列会导致右下角→↓冲突，所以只能是偶数行偶数列
	int runSlots = 320;				//默认最低运行320个时间片
	int numRobot = 4;				//机器人数量的初始值
	int numDestination = 4;			//默认至少有4个目的地
	int numOutPort = 4;				//默认至少有4个出货口

	//实验统计数据
	int totalCount = 0;				//最终送达货物数量统计,本实验的最终目标
	int failRCount = 0;				//机器人失败统计
	int maxActiveRobots = 0;		//本场景最多可活动机器人统计
};
