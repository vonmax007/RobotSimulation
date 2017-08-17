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

//记录路径-时间对;
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
	Robot(int & Rid, Point & sp, Point & dp, int & is);			//构造函数
	void findKPath(TileType ** inMap, int inRow, int inCol);//每次更新路径集算法
	int getEarliestArr(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot);
	void selectBestRoute(ResEntity ** ResEntity_list, int & curr_slot);
	bool reserveRoute(ResEntity ** ResEntity_list, int & curr_slot);//竞争预约此路径
	void printTW(ResEntity ** ResEntity_list, int & curr_slot);
	bool isSamePath(list <Point> & path1, list <Point> & path2);
	//Robot attributes
public:

	int Rid;				//机器人的ID
	Point startPort;		//原始出发格口ID，运载完成后要返回此格口
	Point destPort;			//目的分拣格口的ID
	Point currPos;			//当前所在位置

	int ignition_slot;		//机器人第一次启动时的时间片
	int k = K_way;			//每次寻找的路径个数k,这里默认为10，后期需要验证的参数
	//int tltRef = 0;			//time left to refresh，倒计时去刷新的时间
	list <list <Point>> ps; //每次机器人找到的路径集合;
	list <Point> currRoute; //当前行驶路径;
	list <TPoint> currRouteTime; //当前行驶路径-时间;
	bool stay = false;		  //当前是否需要等待？
	TileType isFinMission = 1;//是否完成当前任务，1是在在起点正要出发;
	//2是处于从 出货口到 目的口的路上，3是从目的口正要出发返回，4是处于返回的路上;
	//5是属于处于缓冲区状态;
};
