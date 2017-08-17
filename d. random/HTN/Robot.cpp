#include "StdAfx.h"
#include "Robot.h"


Robot::Robot(void)
{
}

Robot::~Robot(void)
{
	this->currRoute.clear();
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

bool Robot::getRoute(int curr_slot, ResEntity ** ResEntity_list, TileType ** inMap, int inRow, int inCol)
{
	HTNMove a;
	this->currRoute = a.findPath(curr_slot, ResEntity_list, inMap, inRow, inCol,
		this->startPort.i, this->startPort.j, this->destPort.i, this->destPort.j);
	//a.destroy();

	if (this->currRoute.size() == 0 || 
		(--this->currRoute.end())->i != this->destPort.i || 
		(--this->currRoute.end())->j != this->destPort.j ||
		currRoute.size() > 3 * (abs(currPos.i - destPort.i) + abs(currPos.j - destPort.j)))
	{
		//std::cout << "\n\n *************Fail to seek path*************  \n\n";
		return false;//寻路失败;
	}
	else
	{
		//开始预约路径上的资源;
		list <HTNPoint> ::iterator rit = this->currRoute.begin();
		for (; rit != this->currRoute.end(); ++rit)
			ResEntity_list[rit->i][rit->j].addResTable(this->Rid, rit->enter_slot, rit->leave_slot);
	}

	return true;	//默认返回true;
}



//打印当前预约路径上的时间窗的函数，测试使用
void Robot::printTW(ResEntity ** ResEntity_list, int & curr_slot)
{
	cout << endl << "当前保留路径的长度是："<< this->currRoute .size()<<"\n";
	cout << endl << "当前保留路径上经过资源的时间窗都是：\n";
	list <FTW_item> FTWTable;
	list <HTNPoint>::iterator ir = this->currRoute.begin();
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
