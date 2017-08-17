#pragma once

//资源预约表-表项
struct ResTable_item
{
	int robot_id;			//机器人ID
	int occupy_slot;		//占用时间槽，因为单个资源通行长度为1，所以此也表示进入时间槽
	ResTable_item(int _robot_id, int _occupy_slot) :
		robot_id(_robot_id), occupy_slot(_occupy_slot) {}
};

//空闲时间窗-表项Free Time Window
struct FTW_item
{
	int start_slot;			//开始时间槽
	int end_slot;			//结束时间槽
	FTW_item(int _start_slot, int _end_slot) : start_slot(_start_slot), end_slot(_end_slot) {}
};


class ResEntity
{
public:
	ResEntity(void);
	~ResEntity(void);

	list <FTW_item> getRobFTW(int Rid);	//返回属于当前机器人的空闲时间窗
	list <int> getResSlot(int Rid);		//得到当前机器人预约要通行的时间槽，测试用
	void delResSlot(int Rid);
	void clearResTable(int curr_slot);	//清除过期预约表
	bool addResTable(int Rid, int enter_slot, int leave_slot);

	//自定义属性
public:

	TileType Rtype;                 //资源类型,为unsigned char 型,
	// 1代表出货口,2代表投递格口(坑),3代表普通路径资源,4代表缓冲区,
	// 5代表未知不可行区域,6代表目的地集合(特殊路径资源);
	list <ResTable_item> ResTable;	//资源预约表,资源的核心功能
};