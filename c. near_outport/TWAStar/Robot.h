#pragma once
#include "StdAfx.h"
#include "ResEntity.h"

#define MAXReah 800000	//表示无法到达目的地;

//寻找可用时间槽的队列结构
struct retime
{
	int i, j;
	int enter_slot;
	int end_slot;	//其位于的空闲时间窗的结束时间槽
	retime * backpointer;//反向指针，用于指示此结构扩展与哪一个资源-时间对
	retime(int _i, int _j, int _enter_slot, int _end_slot):i(_i), j(_j), enter_slot(_enter_slot), end_slot (_end_slot) {}
	retime(int _i, int _j, int _enter_slot, int _end_slot, retime * _backpointer) :
		i(_i), j(_j), enter_slot(_enter_slot), end_slot(_end_slot), backpointer(_backpointer){}
};


class Robot
{
public:
	Robot(void);
	~Robot(void);
	Robot(int & Rid, Point & sp, Point & dp, int & is);			//构造函数
	bool getRoute(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol, Point * buff, int numDest);
	void printTW(ResEntity ** ResEntity_list, int & curr_slot);

	//Robot attributes
public:

	int Rid;			//机器人的ID
	Point startPort;	//原始出发格口ID，运载完成后要返回此格口
	Point destPort;		//目的分拣格口的ID
	Point currPos;		//当前所在位置

	int ignition_slot;	//机器人第一次启动时的时间片
	list <TWASPoint> currRoute;//当前行驶路径;
	bool stay = false;		//当前是否需要等待？
	TileType isFinMission = 1; //是否完成当前任务，1是在在起点正要出发;
	//2是处于从 出货口到 目的口的路上，3是从目的口正要出发返回，4是处于返回的路上;
	//5是属于处于缓冲区状态;
};
