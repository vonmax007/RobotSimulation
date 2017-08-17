
#include "StdAfx.h"
#include "ResEntity.h"



ResEntity::ResEntity(void)
{
	this->Rtype = 3; //默认是普通资源
}


ResEntity::~ResEntity(void)
{
	ResTable.clear();
}

void printE()
{
	cout << "\n\n**************** An exception is here ****************\n\n";
	//return false;
}


//插入占用时间窗表并且更新 空闲时间窗表
bool ResEntity::addResTable(int Rid, int enter_slot, int leave_slot)
{
	delResSlot(Rid);

	if (this->ResTable.size() == 0)	//如果当前资源 预约表为空的话
	{
		for (int occupy_slot = enter_slot; occupy_slot <= leave_slot; ++occupy_slot)
			this->ResTable.push_back(ResTable_item(Rid, occupy_slot));
		return true;				//插入成功
	}
	//当前资源预约时间表大小为1的情况的插入情况
	else if (this->ResTable.size() == 1)
	{
		if (this->ResTable.begin()->occupy_slot > leave_slot)
		{
			for (int occupy_slot = leave_slot; occupy_slot >= enter_slot; --occupy_slot)
				this->ResTable.push_front(ResTable_item(Rid, occupy_slot));
			return true;				//头部插入成功
		}

		else if (this->ResTable.begin()->occupy_slot < enter_slot)
		{
			for (int occupy_slot = enter_slot; occupy_slot <= leave_slot; ++occupy_slot)
				this->ResTable.push_back(ResTable_item(Rid, occupy_slot));
			return true;				//尾部插入成功
		}

		else if (leave_slot == enter_slot && this->ResTable.begin()->occupy_slot == enter_slot
			&& this->ResTable.begin()->robot_id == Rid)
			return true;//单时间槽插入成功			
	}
	//预约表大于等于2的话，需要查找此预约窗需要存在的位置
	else
	{
		//头部插入预约时间槽成功
		if (this->ResTable.begin()->occupy_slot > leave_slot &&
			enter_slot >= 1)
		{
			for (int occupy_slot = leave_slot; occupy_slot >= enter_slot; --occupy_slot)
				this->ResTable.push_front(ResTable_item(Rid, occupy_slot));
			return true;				//头部插入成功
		}

		//在预约时间槽尾部插入此占用成功
		else if ((-- this->ResTable.end())->occupy_slot < enter_slot
			&& END_slot >= leave_slot)
		{
			for (int occupy_slot = enter_slot; occupy_slot <= leave_slot; ++occupy_slot)
				this->ResTable.push_back(ResTable_item(Rid, occupy_slot));
			return true;				//尾部插入成功
		}

		//在预约时间槽表中间插入此预约，这个就复杂了
		//occupy_slot处于 begin 和 end之间某个位置
		else
		{
			list <ResTable_item> ::iterator it1 = this->ResTable.begin();
			list <ResTable_item> ::iterator it2 = this->ResTable.begin(); ++it2;
			for (; it2 != this->ResTable.end(); ++it1, ++it2)
			{
				if (it1->occupy_slot < enter_slot &&
					it2->occupy_slot > leave_slot)
				{
					for (int occupy_slot = enter_slot; occupy_slot <= leave_slot; ++occupy_slot)
						this->ResTable.insert(it2, ResTable_item(Rid, occupy_slot));
					return true;				//中间插入成功
				}
			}
		}
	}
	return false;	//默认是插入不成功的
}


void ResEntity::clearResTable(int curr_slot)
{
	for (list <ResTable_item> ::iterator it = this->ResTable.begin();
	it != this->ResTable.end(); )
	{
		if (it->occupy_slot < curr_slot)	    //删除的时刻到了
			this->ResTable.erase(it++);			//删除元素需要跳出循环，免得下一次遍历这个失效的it
		else ++it;								//下一个位置
	}
}


list <FTW_item> ResEntity::getRobFTW(int Rid)
{
	list <FTW_item> FTWTableT;	//返回的此机器人的空闲时间窗
	if (this->Rtype == 1)
	{
		FTWTableT.push_back(FTW_item(1, END_slot));
		return FTWTableT;
	}
	else if (this->ResTable.size() == 0)
	{
		//size 为0，直接返回就行了
		FTWTableT.push_back(FTW_item(1, END_slot));
		return FTWTableT;
	}
	list <ResTable_item> TempRT(this->ResTable);
	list <ResTable_item> ::iterator it1 = TempRT.begin();
	
	//删除属于本机器人的预约
	for (; it1 != TempRT.end(); )
	{
		if (it1->robot_id == Rid)TempRT.erase(it1++);
		else ++it1;
	}
	if (TempRT.size() == 0)
	{
		//删除属于本机器人的预约后，size 为0，直接返回
		FTWTableT.push_back(FTW_item(1, END_slot));
		return FTWTableT;
	}
	list <ResTable_item> ::iterator it2 = TempRT.begin(); ++it2;
	//处理头部
	if (TempRT.begin()->occupy_slot > 1)
		FTWTableT.push_back(FTW_item(1, TempRT.begin()->occupy_slot - 1));
		
	//处理中间
	for (it1 = TempRT.begin(); it2 != TempRT.end(); ++it1, ++it2)
	{
		if (it2->occupy_slot - it1->occupy_slot > 1)
			FTWTableT.push_back(FTW_item(it1->occupy_slot + 1, it2->occupy_slot - 1));
	}
	//处理尾部
	if ((-- TempRT.end())->occupy_slot < END_slot)
		FTWTableT.push_back(FTW_item((-- TempRT.end())->occupy_slot + 1, END_slot));

	return FTWTableT;
}

list <int> ResEntity::getResSlot(int Rid)
{
	list <int> otl;		//返回占据的时间槽列表
	list <ResTable_item> ::iterator it = this->ResTable.begin();
	for (; it != this->ResTable.end(); ++it)
	{
		//这里有一个问题，如果同一个机器人在不同时间槽预约一个资源多次呢？
		if (it->robot_id == Rid)otl.push_back(it->occupy_slot);
	}
	return otl;
}

void ResEntity::delResSlot(int Rid)
{
	list <ResTable_item> ::iterator it = this->ResTable.begin();
	for (; it != this->ResTable.end(); )
	{
		//这里有一个问题，如果同一个机器人在不同时间槽预约一个资源多次呢？
		if (it->robot_id == Rid)
			this->ResTable.erase(it++);
		else ++it;
	}
}


list <FTW_item> ResEntity::getFTW()
{
	list <FTW_item> FTWTableT;	//返回所需要的空闲时间窗列表
	if (this->Rtype == 1 || this->ResTable.size() == 0)
	{
		//size 为0，直接返回就行了
		FTWTableT.push_back(FTW_item(1, END_slot));
		return FTWTableT;
	}
	
	list <ResTable_item> ::iterator it1 = this->ResTable.begin();
	list <ResTable_item> ::iterator it2 = this->ResTable.begin(); ++it2;
	//处理头部
	if (this->ResTable.begin()->occupy_slot > 1)
		FTWTableT.push_back(FTW_item(1, this->ResTable.begin()->occupy_slot - 1));

	//处理中间
	for (; it2 != this->ResTable.end(); ++it1, ++it2)
	{
		if (it2->occupy_slot - it1->occupy_slot > 1)
			FTWTableT.push_back(FTW_item(it1->occupy_slot + 1, it2->occupy_slot - 1));
	}
	//处理尾部
	if ((--this->ResTable.end())->occupy_slot < END_slot)
		FTWTableT.push_back(FTW_item((--this->ResTable.end())->occupy_slot + 1, END_slot));

	return FTWTableT;
}


//enter_slot是计划进入的时间槽
int ResEntity::isFreePass(int last_enter_slot, int last_end_slot)
{
	if (this->Rtype == 1 || this->ResTable.size() == 0)return last_enter_slot + 1;
	else if(this->Rtype == 6)//目的地集合,判断停留 2 slot;
	{
		list <FTW_item> FTWR = this->getFTW();
		list <FTW_item>::iterator ftwit = FTWR.begin();	//空闲时间窗表指针
		for (; ftwit != FTWR.end(); ++ftwit)
		{
			if (ftwit->end_slot - ftwit->start_slot >= 1)
			{
				if (
					(last_end_slot + 1 >= ftwit->start_slot && last_end_slot + 2 <= ftwit->end_slot)
					|| (last_end_slot + 1 >= ftwit->end_slot && last_enter_slot + 2 <= ftwit->end_slot)
					)return max(last_enter_slot + 1, ftwit->start_slot);
			}	
		}
	}
	else//普通路径资源;
	{
		list <FTW_item> FTWR = this->getFTW();
		list <FTW_item>::iterator ftwit = FTWR.begin();	//空闲时间窗表指针
		for (; ftwit != FTWR.end(); ++ftwit)
		{
			if (
				(last_end_slot + 1 >= ftwit->start_slot && last_end_slot + 1 <= ftwit->end_slot)
				|| (last_end_slot + 1 >= ftwit->end_slot && last_enter_slot + 1 <= ftwit->end_slot)
				)return max(last_enter_slot + 1, ftwit->start_slot);
		}
	}
	return -1;//默认是无法通过的;
}

int ResEntity::locatedslot(int enter_slot)
{
	if (this->Rtype == 1 || this->ResTable.size() == 0)return END_slot;
	else
	{
		list <FTW_item> FTWR = this->getFTW();
		list <FTW_item>::iterator ftwit = FTWR.begin();	//空闲时间窗表指针
		for (; ftwit != FTWR.end(); ++ftwit)
		{
			if (enter_slot >= ftwit->start_slot && enter_slot <= ftwit->end_slot)
				return ftwit->end_slot;		
		}
	}
	return -1;//如果都没发现的话，那就是-1咯;
}
