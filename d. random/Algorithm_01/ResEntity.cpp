
#include "StdAfx.h"
#include "ResEntity.h"



ResEntity::ResEntity(void)
{
	this->Rtype = 3; //Ĭ������ͨ·����Դ
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


//����ռ��ʱ�䴰���Ҹ��� ����ʱ�䴰��
bool ResEntity::addResTable(int Rid, int enter_slot, int leave_slot)
{
	delResSlot(Rid);

	if (this->ResTable.size() == 0)	//�����ǰ��Դ ԤԼ��Ϊ�յĻ�
	{
		for (int occupy_slot = enter_slot; occupy_slot <= leave_slot; ++occupy_slot)
			this->ResTable.push_back(ResTable_item(Rid, occupy_slot));
		return true;				//����ɹ�
	}
	//��ǰ��ԴԤԼʱ����СΪ1������Ĳ������
	else if (this->ResTable.size() == 1)
	{
		if (this->ResTable.begin()->occupy_slot > leave_slot)
		{
			for (int occupy_slot = leave_slot; occupy_slot >= enter_slot; --occupy_slot)
				this->ResTable.push_front(ResTable_item(Rid, occupy_slot));
			return true;				//ͷ������ɹ�
		}

		else if (this->ResTable.begin()->occupy_slot < enter_slot)
		{
			for (int occupy_slot = enter_slot; occupy_slot <= leave_slot; ++occupy_slot)
				this->ResTable.push_back(ResTable_item(Rid, occupy_slot));
			return true;				//β������ɹ�
		}

		else if (leave_slot == enter_slot && this->ResTable.begin()->occupy_slot == enter_slot
			&& this->ResTable.begin()->robot_id == Rid)
			return true;//��ʱ��۲���ɹ�			
	}
	//ԤԼ����ڵ���2�Ļ�����Ҫ���Ҵ�ԤԼ����Ҫ���ڵ�λ��
	else
	{
		//ͷ������ԤԼʱ��۳ɹ�
		if (this->ResTable.begin()->occupy_slot > leave_slot &&
			enter_slot >= 1)
		{
			for (int occupy_slot = leave_slot; occupy_slot >= enter_slot; --occupy_slot)
				this->ResTable.push_front(ResTable_item(Rid, occupy_slot));
			return true;				//ͷ������ɹ�
		}
		
		//��ԤԼʱ���β�������ռ�óɹ�
		else if ((-- this->ResTable.end())->occupy_slot < enter_slot
					&& END_slot >= leave_slot)
		{
			for (int occupy_slot = enter_slot; occupy_slot <= leave_slot; ++occupy_slot)
				this->ResTable.push_back(ResTable_item(Rid, occupy_slot));
			return true;				//β������ɹ�
		}

		//��ԤԼʱ��۱��м�����ԤԼ������͸�����
		//occupy_slot���� begin �� end֮��ĳ��λ��
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
					return true;				//�м����ɹ�
				}
			}
		}
	}
	return false;	//Ĭ���ǲ��벻�ɹ���
}


void ResEntity::clearResTable(int curr_slot)
{
	for (list <ResTable_item> ::iterator it = this->ResTable.begin();
	it != this->ResTable.end(); )
	{
		if (it->occupy_slot < curr_slot)	    //ɾ����ʱ�̵���
			this->ResTable.erase(it++); //ɾ��Ԫ����Ҫ����ѭ���������һ�α������ʧЧ��it
		else ++it;					    //��һ��λ��
	}
}


list <FTW_item> ResEntity::getRobFTW(int Rid)
{
	list <FTW_item> FTWTableT;	//���صĴ˻����˵Ŀ���ʱ�䴰
	if (this->Rtype == 1)
	{
		FTWTableT.push_back(FTW_item(1, END_slot));
		return FTWTableT;
	}
	else if (this->ResTable.size() == 0)
	{
		//size Ϊ0��ֱ�ӷ��ؾ�����
		FTWTableT.push_back(FTW_item(1, END_slot));
		return FTWTableT;
	}
	list <ResTable_item> TempRT(this->ResTable);
	list <ResTable_item> ::iterator it1 = TempRT.begin();
	
	//ɾ�����ڱ������˵�ԤԼ
	for (; it1 != TempRT.end(); )
	{
		if (it1->robot_id == Rid)TempRT.erase(it1++);
		else ++it1;
	}
	if (TempRT.size() == 0)
	{
		//ɾ�����ڱ������˵�ԤԼ��size Ϊ0��ֱ�ӷ���
		FTWTableT.push_back(FTW_item(1, END_slot));
		return FTWTableT;
	}
	list <ResTable_item> ::iterator it2 = TempRT.begin(); ++it2;
	//����ͷ��
	if (TempRT.begin()->occupy_slot > 1)
		FTWTableT.push_back(FTW_item(1, TempRT.begin()->occupy_slot - 1));
		
	//�����м�
	for (it1 = TempRT.begin(); it2 != TempRT.end(); ++it1, ++it2)
	{
		if (it2->occupy_slot - it1->occupy_slot > 1)
			FTWTableT.push_back(FTW_item(it1->occupy_slot + 1, it2->occupy_slot - 1));
	}
	//����β��
	if ((-- TempRT.end())->occupy_slot < END_slot)
		FTWTableT.push_back(FTW_item((-- TempRT.end())->occupy_slot + 1, END_slot));

	return FTWTableT;
}

list <int> ResEntity::getResSlot(int Rid)
{
	list <int> otl;		//����ռ�ݵ�ʱ����б�
	list <ResTable_item> ::iterator it = this->ResTable.begin();
	for (; it != this->ResTable.end(); ++it)
	{
		//������һ�����⣬���ͬһ���������ڲ�ͬʱ���ԤԼһ����Դ����أ�
		if (it->robot_id == Rid)otl.push_back(it->occupy_slot);
	}
	return otl;
}

void ResEntity::delResSlot(int Rid)
{
	list <ResTable_item> ::iterator it = this->ResTable.begin();
	for (; it != this->ResTable.end(); )
	{
		//������һ�����⣬���ͬһ���������ڲ�ͬʱ���ԤԼһ����Դ����أ�
		if (it->robot_id == Rid)
			this->ResTable.erase(it++);
		else ++it;
	}
}