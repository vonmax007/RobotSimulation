// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#define Linx	//linux下运行开关,默认在Windows下运行;

#ifndef Linx
/* Linux下运行删除*/
#include "targetver.h"
#include <stdio.h>
#include <tchar.h>
#include <Windows.h>	//这个头文件在Linux下需要删除，测试所用
#include <stdlib.h>  
#include <crtdbg.h> 
#define _CRTDBG_MAP_ALLOC 
#endif
// TODO:  在此处引用程序需要的其他头文件

#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <vector>
#include <queue>
#include <cmath>
#include <time.h>		//测试时间
using namespace std;

//自定义类型
struct Point
{
	unsigned short i, j;		//i是行,j是列
	Point() {}
	Point(unsigned short _i, unsigned short _j) : i(_i), j(_j) {}
};
typedef list <Point> path;
typedef list <list <Point>> PathSet;
typedef unsigned char TileType;	//声明各方格所使用的类型
#define TTL 5					//预约失效时间,假设每隔5时间槽重新规划一次
#define K_way 10				//每次机器人找到K_way条路径
#define START_slot 1			//开始时间槽从1开始
#define END_slot 10000			//假定实验运行10000个时间槽


//自定义类的头文件
#include "SLSD_TWAStar.h"
#include "VirtualEnvironment.h"
#include "Robot.h"
#include "ResEntity.h"
#include "ExperimentExecutor.h"

