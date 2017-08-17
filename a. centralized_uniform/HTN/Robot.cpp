#include "StdAfx.h"
#include "Robot.h"


Robot::Robot(void)
{
}

Robot::~Robot(void)
{
	this->currRoute.clear();
}

//�����˳�ʼ��ʱ�Ĺ��캯��
Robot::Robot(int & Rid, Point & sp, Point & dp, int & is)
{
	this->Rid = Rid;
	this->startPort = sp;	//ԭʼ�������ID��������ɺ�Ҫ���ش˸��
	this->currPos = sp;		//��ǰ����λ��,һ��ʼ�������ڳ������
	this->destPort = dp;	//Ŀ�ķּ��ڵ�ID
	this->ignition_slot = is;//�����˵�һ������ʱ��ʱ��Ƭ
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
		return false;//Ѱ·ʧ��;
	}
	else
	{
		//��ʼԤԼ·���ϵ���Դ;
		list <HTNPoint> ::iterator rit = this->currRoute.begin();
		for (; rit != this->currRoute.end(); ++rit)
			ResEntity_list[rit->i][rit->j].addResTable(this->Rid, rit->enter_slot, rit->leave_slot);
	}

	return true;	//Ĭ�Ϸ���true;
}



//��ӡ��ǰԤԼ·���ϵ�ʱ�䴰�ĺ���������ʹ��
void Robot::printTW(ResEntity ** ResEntity_list, int & curr_slot)
{
	cout << endl << "��ǰ����·���ĳ����ǣ�"<< this->currRoute .size()<<"\n";
	cout << endl << "��ǰ����·���Ͼ�����Դ��ʱ�䴰���ǣ�\n";
	list <FTW_item> FTWTable;
	list <HTNPoint>::iterator ir = this->currRoute.begin();
	for (; ir != this->currRoute.end(); ++ir)
	{
		list <int> otl = ResEntity_list[ir->i][ir->j].getResSlot(this->Rid);
		cout << "(" << ir->i << "," << ir->j << ")" << " -> ����ʱ�䴰���£� -- ͨ��ʱ�̣� ";
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
