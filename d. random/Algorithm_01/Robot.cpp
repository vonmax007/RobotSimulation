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

//����������õ����絽��ʱ��;
int Robot::getEarliestArr(list <Point> route, ResEntity ** ResEntity_list, int & curr_slot)
{
	queue <retime> que;
	list <FTW_item> FTWTable;
	FTWTable = ResEntity_list[route.begin()->i][route.begin()->j].getRobFTW(this->Rid);
	list <FTW_item> ::iterator itw = FTWTable.begin();//�˻����˵�·����һ����Դ��ʱ�䴰
	for (; itw != FTWTable.end(); ++itw)
	{
		//���ҵ�ǰ��㣬ռ�õ�ʱ���curr_slot λ���ĸ�����ʱ�䴰��
		if(curr_slot <= itw->end_slot && curr_slot >= itw ->start_slot )
			que.push(retime(route.begin()->i, route.begin()->j, curr_slot, itw->end_slot));
	}
	//�������ʱ�䴰
	while (!que.empty())
	{
		retime re = que.front();//�õ������ʱ�䴰Ԫ��
		que.pop();
		if (re.i == (--route.end())->i && re.j == (--route.end())->j)
			return re.enter_slot + 1;//�������絽�ﲢ��ռ�õ�ʱ���

		list <Point> ::iterator itr2 = route.begin();//ָʾ��ǰ·����Դ����һ����Դ;
		for (; itr2 != route.end(); ++itr2)
		{
			if (itr2->i == re.i && itr2->j == re.j)
			{
				++itr2;
				break;
			}
		}
		//�ҵ��˵�ǰ��Դ����һ����Դitr2
		if (ResEntity_list[itr2->i][itr2->j].Rtype == 6)//Ŀ�ĵؼ���,�ж�ͣ�� 2 slot;
		{
			FTWTable = ResEntity_list[itr2->i][itr2->j].getRobFTW(this->Rid);
			itw = FTWTable.begin();
			for (; itw != FTWTable.end(); ++itw)
			{
				if (itw->end_slot - itw->start_slot >= 1)
				{
					//������һ����Դ�� ��ʱ�䴰�ɴ�,��������ͣ��2��ʱ�䴰����;
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
				//������һ����Դ�� ��ʱ�䴰�ɴ�
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

//ѡ��ǰ����·��·��
void Robot::selectBestRoute(ResEntity ** ResEntity_list, int & curr_slot)
{	
	int reach_slot = MAXReah;//��ʼ��һ��,���һ�õ�ǰ����·��������ﵽʱ���;
	int curr_reach_slot = MAXReah + 1;//�����ǰ·��Ϊ�գ����ʼ����Ϊ���ʱ���

	if (!this->currRoute.empty())//�����ǰ·����Ϊ�գ�������䵽��ʱ���
		curr_reach_slot = getEarliestArr(this->currRoute, ResEntity_list, curr_slot);

	list <list <Point>> ::iterator itps = this->ps.begin();//ÿ�λ������ҵ���·������;
	for (; itps != this->ps.end(); ++itps)
	{
		reach_slot = getEarliestArr(*itps, ResEntity_list, curr_slot);
		if (curr_reach_slot > reach_slot)//����˺�ѡ·�� �ȵ�ǰ·�� ���絽��;
		{
			this->currRoute = *itps;//ֻ�滻����·��;
			curr_reach_slot = reach_slot;
			this->currRouteTime.clear();
		}
	}
}

//ԤԼ·���ϵ���Դ����
bool Robot::reserveRoute(ResEntity ** ResEntity_list, int & curr_slot)
{
	this->currRouteTime.clear();

	//ÿ��reftimeʱ������������ˢ��ԤԼ
	list <retime> lrt;
	queue <retime> que;
	list <FTW_item> FTWTable;
	FTWTable = ResEntity_list[currRoute.begin()->i][currRoute.begin()->j].getRobFTW(this->Rid);
	list <FTW_item> ::iterator itw = FTWTable.begin();
	for (; itw != FTWTable.end(); ++itw)
	{
		//���ҵ�ǰ��㣬ռ�õ�ʱ���curr_slot λ���ĸ�����ʱ�䴰��
		if (curr_slot <= itw->end_slot && curr_slot >= itw->start_slot)
			que.push(retime(currRoute.begin()->i, currRoute.begin()->j, curr_slot, itw->end_slot, nullptr));
	}
	//�������ʱ�䴰
	while (!que.empty())
	{
		retime re = que.front();//�õ������ʱ�䴰Ԫ��
		lrt.push_back(re);		//ѹ����������Ѱ��
		que.pop();
		//����Ŀ�ĵأ���ʼ����;
		if (re.i == (--currRoute.end())->i && re.j == (--currRoute.end())->j)
		{
			int leave_slot = re.enter_slot + 1;//����һʱ���Ͷ��ʱ��;***************
			while (re.backpointer != nullptr)
			{
				this->currRouteTime.push_front(TPoint(re.i, re.j, re.enter_slot, leave_slot));
				if(ResEntity_list[re.i][re.j].Rtype == 3 || ResEntity_list[re.i][re.j].Rtype == 6)//�����3����ͨ·����Դ��ԤԼ�ж�;
					if (!ResEntity_list[re.i][re.j].addResTable(this->Rid, re.enter_slot, leave_slot))
					{
						return false;
					}
						
				leave_slot = re.enter_slot - 1;
				re = * re.backpointer;
			}
			this->currRouteTime.push_front(TPoint(re.i, re.j, re.enter_slot, leave_slot));
			//·��ͷ����Դ�����;
			if (ResEntity_list[re.i][re.j].Rtype == 3 || ResEntity_list[re.i][re.j].Rtype == 6)//�����3����ͨ·����Դ��ԤԼ�ж�;
				if (!ResEntity_list[re.i][re.j].addResTable(this->Rid, re.enter_slot, leave_slot))
					return false;

			//ԤԼ���
			return true;//���ؾ����ɹ�
		}

		list <Point> ::iterator itr2 = currRoute.begin();//ָʾ��ǰ·����Դ����һ����Դ;
		for (; itr2 != currRoute.end(); ++itr2)
		{
			if (itr2->i == re.i && itr2->j == re.j)
			{
				++itr2;
				break;
			}
		}
		//�ҵ��˵�ǰ��Դ����һ����Դitr2
		if (ResEntity_list[itr2->i][itr2->j].Rtype == 6)//Ŀ�ĵؼ���,�ж�ͣ�� 2 slot;
		{
			FTWTable = ResEntity_list[itr2->i][itr2->j].getRobFTW(this->Rid);
			itw = FTWTable.begin();
			for (; itw != FTWTable.end(); ++itw)
			{
				if (itw->end_slot - itw->start_slot >= 1)
				{
					//������һ����Դ�� ��ʱ�䴰�ɴ�,��������ͣ��2��ʱ�䴰����;
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
				//������һ����Դ�� ��ʱ�䴰�ɴ�
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

	return false;			//Ĭ��û�гɹ���������Դ
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

bool Robot::isSamePath(list <Point> & path1, list <Point> & path2)
{
	if (path1.size() != path2.size())return false;	//ֱ�ӷ����䲻ͬ·��Ҳ
	list <Point>::iterator it1, it2;
	for (it1 = path1.begin(), it2 = path2.begin();
	it1 != path1.end(), it2 != path2.end();
		++it1, ++it2)
	{
		if (it1->i != it2->i || it1->j != it2->j)return false;
	}
	return true;
}