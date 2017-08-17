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

//�����˳�ʼ��ʱ�Ĺ��캯��
Robot::Robot(int & Rid, Point & sp, Point & dp, int & is)
{
	this->Rid = Rid;
	this->startPort = sp;	//ԭʼ�������ID��������ɺ�Ҫ���ش˸��
	this->currPos = sp;		//��ǰ����λ��,һ��ʼ�������ڳ������
	this->destPort = dp;	//Ŀ�ķּ��ڵ�ID
	this->ignition_slot = is;//�����˵�һ������ʱ��ʱ��Ƭ
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
	itre1->enter_slot = curr_slot;		//��ʼ����Ԫ�ؽ���ʱ���
	itre1->leave_slot = curr_slot;		//��ʼ����Ԫ���뿪ʱ���
	itre1->itw = itre1->FTWTable.begin();//�˻����˵�·����һ����Դ��ʱ�䴰
	for (; itre1->itw != itre1->FTWTable.end(); ++itre1->itw)
	{
		//���ҵ�ǰ��㣬ռ�õ�ʱ���curr_slot λ���ĸ�����ʱ�䴰��
		if (curr_slot <= itre1->itw->end_slot && curr_slot >= itre1->itw->start_slot)
			break;
	}
	//��㶼û�����뵽��������;
	if (itre1->itw == itre1->FTWTable.end())return -1;//ֱ�ӷ��ؿ�

	for (; itre2 != (--pathTime.end()); ++itre1, ++itre2)
	{
		for (; itre2->itw != itre2->FTWTable.end(); ++(itre2->itw))
		{
			if (itre2->itw->end_slot >= itre1->leave_slot + 1)//�ɴ����ж�
			{
				itre2->enter_slot = max(itre2->itw->start_slot, itre1->leave_slot + 1);
				itre2->leave_slot = itre2->enter_slot;
				break;	//�Ѿ��ҵ���ʱ�䴰ָ�룬����
			}
		}
		//itre1Ҳ����ǰ�ڵ�ʱ�䴰�ӳ�;
		itre1->leave_slot = itre2->enter_slot - 1;
	}
	//itre2 == pathTime.end();
	for (; itre2->itw != itre2->FTWTable.end(); ++(itre2->itw))
	{
		if (itre2->itw->end_slot >= itre1->leave_slot + 2)//�ɴ����ж�,2 slotͣ���ж�;
		{
			itre2->enter_slot = max(itre2->itw->start_slot, itre1->leave_slot + 1);
			itre2->leave_slot = itre2->enter_slot + 1;//���Ŀ��ڵ��1slot��ռ��;******
			break;	//�Ѿ��ҵ���ʱ�䴰ָ�룬����
		}
	}
	//itre1Ҳ����ǰ�ڵ�ʱ�䴰�ӳ�;
	itre1->leave_slot = itre2->enter_slot - 1;

	//��ʼ·��ʱ�䴰�ص����;
	itre1 = pathTime.begin();
	//itre2 = pathTime.begin(); ++itre2;
	for (; itre1 != pathTime.end(); ++itre1)
	{
		if (itre1->leave_slot > itre1->itw->end_slot)//����˽ڵ㷢���ص�;
		{
			if (itre1 == pathTime.begin())return -1;//����-1��ʾ·�������У�����㷢���ص�
			else
			{
				itre1->itw++;	//ȡ�ص�·�������һ��ʱ�䴰
				return 1;		//����1��ʾ·�������ص������Ѿ�����
			}			
		}		
	}
	return 0;					//����0��ʾ·��OK������ͨ��
}


//Ѱ��ĳһָ��·���Ϻ��ʵ���ʱ��
//����������ӳ�ʱ�䴰������ʱ�䴰�ص�;
int Robot::getEarliestArr(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot)
{
	if(route.size() <= 1)return MAXReah;//����Ϊ1��·��������ȡ��
	
	list <retime> pathTime;	
	list <Point> ::iterator rit;		//·����ָʾ��
	list <FTW_item>::iterator itw;		//·��ʱ�䴰ָʾ��

	//��ʼ��Ѱ·ʱ���
	for (rit = route.begin(); rit != route.end(); ++rit)
	{
		retime temp(rit->i, rit->j);
		temp.FTWTable = ResEntity_list[rit->i][rit->j].getRobFTW(this->Rid);
		//temp.itw = temp.FTWTable.begin();//�����Ǹ��ǳ�����Ĵ���ָ��ָ����ʱ�ڴ棬���º������
		pathTime.push_back(temp);
		pathTime.back().itw = pathTime.back().FTWTable.begin();
	}

	list <retime>::iterator itre1 = pathTime.begin();
	list <retime>::iterator itre2 = pathTime.begin(); ++itre2;
	itre1->enter_slot = curr_slot;		//��ʼ����Ԫ�ؽ���ʱ���
	int overlap = optw(pathTime, curr_slot);//Ĭ���ǲ�ֹͣ�����������ص���
	//optw,����-1��ʾ·�������У�����1��ʾ·�������ص�������0��ʾ·��OK������ͨ��

	while (overlap == 1)
		overlap = optw(pathTime, curr_slot);

	if(overlap == -1)return MAXReah;
	else return pathTime.back().enter_slot;
}

//ѡ��ǰ����·��·��
void Robot::selectBestRoute(ResEntity ** ResEntity_list, int & curr_slot)
{
	int reach_slot = MAXReah;//��ʼ��һ��,���һ�õ�ǰ����·��������ﵽʱ���;
	int curr_reach_slot = MAXReah + 1;//�����ǰ·��Ϊ�գ����ʼ����Ϊ���ʱ���

	list <list <Point>> ::iterator itps = this->ps.begin();//ÿ�λ������ҵ���·������;
	for (; itps != this->ps.end(); ++itps)
	{
		reach_slot = getEarliestArr(*itps, ResEntity_list, curr_slot);
		if (curr_reach_slot > reach_slot)//����˺�ѡ·�� �ȵ�ǰ·�� ���絽��;
		{
			this->currRoute = *itps;
			curr_reach_slot = reach_slot;
		}
	}
}

//��Ѱ·���Ͻڵ㵽���뿪ʱ��
list <retime> Robot::findTW(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot)
{
	list <retime> pathTime;
	if (route.size() <= 1)return pathTime;//����Ϊ1��·��������ȡ��
	
	list <Point> ::iterator rit;		//·����ָʾ��
	list <FTW_item>::iterator itw;		//·��ʱ�䴰ָʾ��
										//��ʼ��Ѱ·ʱ���
	for (rit = route.begin(); rit != route.end(); ++rit)
	{
		retime temp(rit->i, rit->j);
		temp.FTWTable = ResEntity_list[rit->i][rit->j].getRobFTW(this->Rid);
		
		pathTime.push_back(temp);
		pathTime.back().itw = pathTime.back().FTWTable.begin();
	}

	list <retime>::iterator itre1 = pathTime.begin();
	list <retime>::iterator itre2 = pathTime.begin(); ++itre2;
	itre1->enter_slot = curr_slot;		//��ʼ����Ԫ�ؽ���ʱ���
	int overlap = optw(pathTime, curr_slot);//Ĭ���ǲ�ֹͣ�����������ص���
											//optw,����-1��ʾ·�������У�����1��ʾ·�������ص�������0��ʾ·��OK������ͨ��
	while (overlap == 1)
		overlap = optw(pathTime, curr_slot);

	if (overlap == -1) { pathTime.clear(); return pathTime; }
	else return pathTime;
}

//ԤԼ·���ϵ���Դ����
bool Robot::reserveRoute(ResEntity ** ResEntity_list, int & curr_slot)
{
	this->currRouteTime.clear();
	list <retime> pathTime = findTW(this->currRoute, ResEntity_list, curr_slot);
	if (pathTime.size() == 0)return false;//��·��ͨ!!!
	list <retime>::iterator pit = pathTime.begin();
	for (; pit != pathTime.end(); ++pit)
	{
		this->currRouteTime.push_back(TPoint(pit->i, pit->j, pit->enter_slot, pit->leave_slot));
		//��ǰռ����Դ��ԤԼ����
		if (ResEntity_list[pit->i][pit->j].Rtype == 3 ||
			ResEntity_list[pit->i][pit->j].Rtype == 6)//�����3,��Ҫ����ԤԼ�ж�;
		{
			if (!ResEntity_list[pit->i][pit->j].addResTable(this->Rid, pit->enter_slot, pit->leave_slot))
			{
				this->currRouteTime.clear();
				return false;
			}	
		}
	}

	//ԤԼ���
	return true;//Ĭ�Ϸ��ؾ����ɹ�
}

//��ӡ��ǰԤԼ·���ϵ�ʱ�䴰�ĺ���������ʹ��
void Robot::printTW(ResEntity ** ResEntity_list, int & curr_slot)
{
	cout << endl << "��ǰ����·���ĳ����ǣ�"<< this->currRoute .size()<<"\n";
	cout << endl << "��ǰ����·���Ͼ�����Դ��ʱ�䴰���ǣ�\n";
	list <FTW_item> FTWTable;
	list <Point>::iterator ir = this->currRoute.begin();
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
