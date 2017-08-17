#include "StdAfx.h"
#include "Robot.h"


Robot::Robot(void)
{
}

Robot::~Robot(void)
{
	this->ps.clear();
	this->currRoute.clear();
	this->currRouteTime.clear();
}

//机器人初始化时的构造函数
Robot::Robot(int & Rid, Point & sp, Point & dp, int & is)
{
	this->Rid = Rid;
	this->startPort = sp;	//原始出发格口ID，运载完成后要返回此格口
	this->currPos = sp;		//当前所在位置,一开始出发就在出发格口
	this->destPort = dp;	//目的分拣格口的ID
	this->ignition_slot = is;//机器人第一次启动时的时间片
}

void Robot::findKPath(TileType ** inMap, int inRow, int inCol)
{
	AStar a;
	this->ps = a.findKPath(this->k, inMap, inRow, inCol, this->currPos.i, this->currPos.j, this->destPort.i, this->destPort.j);
	//a.destroy();
}

//深度搜索，得到最早到达时间;
int Robot::getEarliestArr(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot)
{
	queue <retime> que;
	list <FTW_item> FTWTable;
	FTWTable = ResEntity_list[route.begin()->i][route.begin()->j].getRobFTW(this->Rid);
	list <FTW_item> ::iterator itw = FTWTable.begin();//此机器人的路径第一个资源的时间窗
	for (; itw != FTWTable.end(); ++itw)
	{
		//查找当前起点，占用的时间槽curr_slot 位于哪个空闲时间窗内
		if(curr_slot <= itw->end_slot && curr_slot >= itw ->start_slot )
			que.push(retime(route.begin()->i, route.begin()->j, curr_slot, itw->end_slot));
	}
	//深度搜索时间窗
	while (!que.empty())
	{
		retime re = que.front();//得到最早的时间窗元素
		que.pop();
		if (re.i == (--route.end())->i && re.j == (--route.end())->j)
			return re.enter_slot + 1;//返回最早到达并且占用的时间槽

		list <Point> ::iterator itr2 = route.begin();//指示当前路径资源的下一个资源;
		for (; itr2 != route.end(); ++itr2)
		{
			if (itr2->i == re.i && itr2->j == re.j)
			{
				++itr2;
				break;
			}
		}
		//找到了当前资源的下一个资源itr2
		if (ResEntity_list[itr2->i][itr2->j].Rtype == 6)//目的地集合,判断停留 2 slot;
		{
			FTWTable = ResEntity_list[itr2->i][itr2->j].getRobFTW(this->Rid);
			itw = FTWTable.begin();
			for (; itw != FTWTable.end(); ++itw)
			{
				if (itw->end_slot - itw->start_slot >= 1)
				{
					//假如下一个资源的 此时间窗可达,并且满足停留2个时间窗长度;
					if ((re.end_slot + 1 >= itw->start_slot && re.end_slot + 2 <= itw->end_slot)
						|| (re.end_slot + 1 >= itw->end_slot && re.enter_slot + 2 <= itw->end_slot)
						)
					{
						int enter_slot = max(re.enter_slot + 1, itw->start_slot);
						//if(constraints_ok())then
						que.push(retime(itr2->i, itr2->j, enter_slot, itw->end_slot));
					}
				}				
			}
		}
		else
		{
			FTWTable = ResEntity_list[itr2->i][itr2->j].getRobFTW(this->Rid);
			itw = FTWTable.begin();
			for (; itw != FTWTable.end(); ++itw)
			{
				//假如下一个资源的 此时间窗可达
				if ((re.end_slot + 1 >= itw->start_slot && re.end_slot + 1 <= itw->end_slot)
					|| (re.end_slot + 1 >= itw->end_slot && re.enter_slot + 1 <= itw->end_slot)
					)
				{
					int enter_slot = max(re.enter_slot + 1, itw->start_slot);
					//if(constraints_ok())then
					que.push(retime(itr2->i, itr2->j, enter_slot, itw->end_slot));
				}
			}
		}
	}
	return MAXReah;
}

//选择当前最优路径路径
void Robot::selectBestRoute(ResEntity ** ResEntity_list, int & curr_slot)
{	
	int reach_slot = MAXReah;//初始化一下,并且获得当前保存路径的最早达到时间槽;
	int curr_reach_slot = MAXReah + 1;//如果当前路径为空，则初始化其为最大时间槽

	if (!this->currRoute.empty())//如果当前路径不为空，则计算其到达时间槽
		curr_reach_slot = getEarliestArr(this->currRoute, ResEntity_list, curr_slot);

	list <list <Point>> ::iterator itps = this->ps.begin();//每次机器人找到的路径集合;
	for (; itps != this->ps.end(); ++itps)
	{
		reach_slot = getEarliestArr(*itps, ResEntity_list, curr_slot);
		if (curr_reach_slot > reach_slot)//如果此候选路径 比当前路径 更早到达;
		{
			this->currRoute = *itps;//只替换可行路径;
			curr_reach_slot = reach_slot;
			this->currRouteTime.clear();
		}
	}
}

//预约路径上的资源函数
bool Robot::reserveRoute(ResEntity ** ResEntity_list, int & curr_slot)
{
	this->currRouteTime.clear();

	//每隔reftime时间间隔必须重新刷新预约
	list <retime> lrt;
	queue <retime> que;
	list <FTW_item> FTWTable;
	FTWTable = ResEntity_list[currRoute.begin()->i][currRoute.begin()->j].getRobFTW(this->Rid);
	list <FTW_item> ::iterator itw = FTWTable.begin();
	for (; itw != FTWTable.end(); ++itw)
	{
		//查找当前起点，占用的时间槽curr_slot 位于哪个空闲时间窗内
		if (curr_slot <= itw->end_slot && curr_slot >= itw->start_slot)
			que.push(retime(currRoute.begin()->i, currRoute.begin()->j, curr_slot, itw->end_slot, nullptr));
	}
	//深度搜索时间窗
	while (!que.empty())
	{
		retime re = que.front();//得到最早的时间窗元素
		lrt.push_back(re);		//压入链表，反向寻找
		que.pop();
		//到达目的地，开始回溯;
		if (re.i == (--currRoute.end())->i && re.j == (--currRoute.end())->j)
		{
			int leave_slot = re.enter_slot + 1;//保留一时间槽投递时间;***************
			while (re.backpointer != nullptr)
			{
				this->currRouteTime.push_front(TPoint(re.i, re.j, re.enter_slot, leave_slot));
				if(ResEntity_list[re.i][re.j].Rtype == 3 || ResEntity_list[re.i][re.j].Rtype == 6)//如果是3，普通路径资源，预约判断;
					if (!ResEntity_list[re.i][re.j].addResTable(this->Rid, re.enter_slot, leave_slot))
					{
						return false;
					}
						
				leave_slot = re.enter_slot - 1;
				re = * re.backpointer;
			}
			this->currRouteTime.push_front(TPoint(re.i, re.j, re.enter_slot, leave_slot));
			//路径头部资源的情况;
			if (ResEntity_list[re.i][re.j].Rtype == 3 || ResEntity_list[re.i][re.j].Rtype == 6)//如果是3，普通路径资源，预约判断;
				if (!ResEntity_list[re.i][re.j].addResTable(this->Rid, re.enter_slot, leave_slot))
					return false;

			//预约完毕
			return true;//返回竞争成功
		}

		list <Point> ::iterator itr2 = currRoute.begin();//指示当前路径资源的下一个资源;
		for (; itr2 != currRoute.end(); ++itr2)
		{
			if (itr2->i == re.i && itr2->j == re.j)
			{
				++itr2;
				break;
			}
		}
		//找到了当前资源的下一个资源itr2
		if (ResEntity_list[itr2->i][itr2->j].Rtype == 6)//目的地集合,判断停留 2 slot;
		{
			FTWTable = ResEntity_list[itr2->i][itr2->j].getRobFTW(this->Rid);
			itw = FTWTable.begin();
			for (; itw != FTWTable.end(); ++itw)
			{
				if (itw->end_slot - itw->start_slot >= 1)
				{
					//假如下一个资源的 此时间窗可达,并且满足停留2个时间窗长度;
					if ((re.end_slot + 1 >= itw->start_slot && re.end_slot + 2 <= itw->end_slot)
						|| (re.end_slot + 1 >= itw->end_slot && re.enter_slot + 2 <= itw->end_slot)
						)
					{
						int enter_slot = max(re.enter_slot + 1, itw->start_slot);
						//if(constraints_ok())then
						que.push(retime(itr2->i, itr2->j, enter_slot, itw->end_slot, &(*(--lrt.end()))));
					}
				}
			}
		}
		else
		{
			FTWTable = ResEntity_list[itr2->i][itr2->j].getRobFTW(this->Rid);
			itw = FTWTable.begin();
			for (; itw != FTWTable.end(); ++itw)
			{
				//假如下一个资源的 此时间窗可达
				if ((re.end_slot + 1 >= itw->start_slot && re.end_slot + 1 <= itw->end_slot)
					|| (re.end_slot + 1 >= itw->end_slot && re.enter_slot + 1 <= itw->end_slot)
					)
				{
					int enter_slot = max(re.enter_slot + 1, itw->start_slot);
					//if(constraints_ok())then
					que.push(retime(itr2->i, itr2->j, enter_slot, itw->end_slot, &(*(--lrt.end()))));
				}
			}
		}
	}

	return false;			//默认没有成功竞争此资源
}

//打印当前预约路径上的时间窗的函数，测试使用
void Robot::printTW(ResEntity ** ResEntity_list, int & curr_slot)
{
	cout << endl << "当前保留路径的长度是："<< this->currRoute .size()<<"\n";
	cout << endl << "当前保留路径上经过资源的时间窗都是：\n";
	list <FTW_item> FTWTable;
	list <Point>::iterator ir = this->currRoute.begin();
	for (; ir != this->currRoute.end(); ++ir)
	{
		list <int> otl = ResEntity_list[ir->i][ir->j].getResSlot(this->Rid);
		cout << "(" << ir->i << "," << ir->j << ")" << " -> 空闲时间窗如下： -- 通行时刻： ";
		for (list <int>::iterator it = otl.begin(); it != otl.end(); ++it)cout << *it << "  ";
		cout << endl;
		FTWTable = ResEntity_list[ir->i][ir->j].getRobFTW(this->Rid);
		list <FTW_item> ::iterator itw = FTWTable.begin();
		for (; itw != FTWTable.end(); ++itw)
		{
			cout << "\t " << itw->start_slot << " : " << itw->end_slot << endl;
		}
		cout << "\n";
	}
}

bool Robot::isSamePath(list <Point> & path1, list <Point> & path2)
{
	if (path1.size() != path2.size())return false;	//直接返回其不同路径也
	list <Point>::iterator it1, it2;
	for (it1 = path1.begin(), it2 = path2.begin();
	it1 != path1.end(), it2 != path2.end();
		++it1, ++it2)
	{
		if (it1->i != it2->i || it1->j != it2->j)return false;
	}
	return true;
}