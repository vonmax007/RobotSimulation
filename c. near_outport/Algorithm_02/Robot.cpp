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

//operate time window
int optw(list <retime> & pathTime, int & curr_slot)
{
	list <retime>::iterator itre1 = pathTime.begin();
	list <retime>::iterator itre2 = pathTime.begin(); ++itre2;
	itre1->enter_slot = curr_slot;		//初始化首元素进入时间槽
	itre1->leave_slot = curr_slot;		//初始化首元素离开时间槽
	itre1->itw = itre1->FTWTable.begin();//此机器人的路径第一个资源的时间窗
	for (; itre1->itw != itre1->FTWTable.end(); ++itre1->itw)
	{
		//查找当前起点，占用的时间槽curr_slot 位于哪个空闲时间窗内
		if (curr_slot <= itre1->itw->end_slot && curr_slot >= itre1->itw->start_slot)
			break;
	}
	//起点都没有申请到，这尴尬;
	if (itre1->itw == itre1->FTWTable.end())return -1;//直接返回咯

	for (; itre2 != (--pathTime.end()); ++itre1, ++itre2)
	{
		for (; itre2->itw != itre2->FTWTable.end(); ++(itre2->itw))
		{
			if (itre2->itw->end_slot >= itre1->leave_slot + 1)//可达性判断
			{
				itre2->enter_slot = max(itre2->itw->start_slot, itre1->leave_slot + 1);
				itre2->leave_slot = itre2->enter_slot;
				break;	//已经找到此时间窗指针，跳出
			}
		}
		//itre1也即是前节点时间窗延长;
		itre1->leave_slot = itre2->enter_slot - 1;
	}
	//itre2 == pathTime.end();
	for (; itre2->itw != itre2->FTWTable.end(); ++(itre2->itw))
	{
		if (itre2->itw->end_slot >= itre1->leave_slot + 2)//可达性判断,2 slot停留判断;
		{
			itre2->enter_slot = max(itre2->itw->start_slot, itre1->leave_slot + 1);
			itre2->leave_slot = itre2->enter_slot + 1;//最后目标节点加1slot的占用;******
			break;	//已经找到此时间窗指针，跳出
		}
	}
	//itre1也即是前节点时间窗延长;
	itre1->leave_slot = itre2->enter_slot - 1;

	//开始路径时间窗重叠检测;
	itre1 = pathTime.begin();
	//itre2 = pathTime.begin(); ++itre2;
	for (; itre1 != pathTime.end(); ++itre1)
	{
		if (itre1->leave_slot > itre1->itw->end_slot)//如果此节点发生重叠;
		{
			if (itre1 == pathTime.begin())return -1;//返回-1表示路径不可行，在起点发生重叠
			else
			{
				itre1->itw++;	//取重叠路径点的下一个时间窗
				return 1;		//返回1表示路径存在重叠，且已经处理
			}			
		}		
	}
	return 0;					//返回0表示路径OK，可以通行
}


//寻找某一指定路径上合适到达时间
//反向遍历，延长时间窗，消除时间窗重叠;
int Robot::getEarliestArr(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot)
{
	if(route.size() <= 1)return MAXReah;//长度为1的路径，不可取啊
	
	list <retime> pathTime;	
	list <Point> ::iterator rit;		//路径点指示器
	list <FTW_item>::iterator itw;		//路径时间窗指示器

	//初始化寻路时间表
	for (rit = route.begin(); rit != route.end(); ++rit)
	{
		retime temp(rit->i, rit->j);
		temp.FTWTable = ResEntity_list[rit->i][rit->j].getRobFTW(this->Rid);
		//temp.itw = temp.FTWTable.begin();//这里是个非常经典的错误，指针指向临时内存，导致后面错误
		pathTime.push_back(temp);
		pathTime.back().itw = pathTime.back().FTWTable.begin();
	}

	list <retime>::iterator itre1 = pathTime.begin();
	list <retime>::iterator itre2 = pathTime.begin(); ++itre2;
	itre1->enter_slot = curr_slot;		//初始化首元素进入时间槽
	int overlap = optw(pathTime, curr_slot);//默认是不停止搜索，且有重叠的
	//optw,返回-1表示路径不可行，返回1表示路径存在重叠，返回0表示路径OK，可以通行

	while (overlap == 1)
		overlap = optw(pathTime, curr_slot);

	if(overlap == -1)return MAXReah;
	else return pathTime.back().enter_slot;
}

//选择当前最优路径路径
void Robot::selectBestRoute(ResEntity ** ResEntity_list, int & curr_slot)
{
	int reach_slot = MAXReah;//初始化一下,并且获得当前保存路径的最早达到时间槽;
	int curr_reach_slot = MAXReah + 1;//如果当前路径为空，则初始化其为最大时间槽

	list <list <Point>> ::iterator itps = this->ps.begin();//每次机器人找到的路径集合;
	for (; itps != this->ps.end(); ++itps)
	{
		reach_slot = getEarliestArr(*itps, ResEntity_list, curr_slot);
		if (curr_reach_slot > reach_slot)//如果此候选路径 比当前路径 更早到达;
		{
			this->currRoute = *itps;
			curr_reach_slot = reach_slot;
		}
	}
}

//搜寻路径上节点到达离开时间
list <retime> Robot::findTW(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot)
{
	list <retime> pathTime;
	if (route.size() <= 1)return pathTime;//长度为1的路径，不可取啊
	
	list <Point> ::iterator rit;		//路径点指示器
	list <FTW_item>::iterator itw;		//路径时间窗指示器
										//初始化寻路时间表
	for (rit = route.begin(); rit != route.end(); ++rit)
	{
		retime temp(rit->i, rit->j);
		temp.FTWTable = ResEntity_list[rit->i][rit->j].getRobFTW(this->Rid);
		
		pathTime.push_back(temp);
		pathTime.back().itw = pathTime.back().FTWTable.begin();
	}

	list <retime>::iterator itre1 = pathTime.begin();
	list <retime>::iterator itre2 = pathTime.begin(); ++itre2;
	itre1->enter_slot = curr_slot;		//初始化首元素进入时间槽
	int overlap = optw(pathTime, curr_slot);//默认是不停止搜索，且有重叠的
											//optw,返回-1表示路径不可行，返回1表示路径存在重叠，返回0表示路径OK，可以通行
	while (overlap == 1)
		overlap = optw(pathTime, curr_slot);

	if (overlap == -1) { pathTime.clear(); return pathTime; }
	else return pathTime;
}

//预约路径上的资源函数
bool Robot::reserveRoute(ResEntity ** ResEntity_list, int & curr_slot)
{
	this->currRouteTime.clear();
	list <retime> pathTime = findTW(this->currRoute, ResEntity_list, curr_slot);
	if (pathTime.size() == 0)return false;//此路不通!!!
	list <retime>::iterator pit = pathTime.begin();
	for (; pit != pathTime.end(); ++pit)
	{
		this->currRouteTime.push_back(TPoint(pit->i, pit->j, pit->enter_slot, pit->leave_slot));
		//当前占有资源的预约请求
		if (ResEntity_list[pit->i][pit->j].Rtype == 3 ||
			ResEntity_list[pit->i][pit->j].Rtype == 6)//如果是3,需要进行预约判断;
		{
			if (!ResEntity_list[pit->i][pit->j].addResTable(this->Rid, pit->enter_slot, pit->leave_slot))
			{
				this->currRouteTime.clear();
				return false;
			}	
		}
	}

	//预约完毕
	return true;//默认返回竞争成功
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
