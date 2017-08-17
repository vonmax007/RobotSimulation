#pragma once

//��ԴԤԼ��-����
struct ResTable_item
{
	int robot_id;			//������ID
	int occupy_slot;		//ռ��ʱ��ۣ���Ϊ������Դͨ�г���Ϊ1�����Դ�Ҳ��ʾ����ʱ���
	ResTable_item(int _robot_id, int _occupy_slot) :
		robot_id(_robot_id), occupy_slot(_occupy_slot) {}
};

//����ʱ�䴰-����Free Time Window
struct FTW_item
{
	int start_slot;			//��ʼʱ���
	int end_slot;			//����ʱ���
	FTW_item(int _start_slot, int _end_slot) : start_slot(_start_slot), end_slot(_end_slot) {}
};


class ResEntity
{
public:
	ResEntity(void);
	~ResEntity(void);

	list <FTW_item> getRobFTW(int Rid);	//�������ڵ�ǰ�����˵Ŀ���ʱ�䴰
	list <int> getResSlot(int Rid);		//�õ���ǰ������ԤԼҪͨ�е�ʱ��ۣ�������
	void delResSlot(int Rid);
	void clearResTable(int curr_slot);	//�������ԤԼ��
	bool addResTable(int Rid, int enter_slot, int leave_slot);

	//�Զ�������
public:

	TileType Rtype;                 //��Դ����,Ϊunsigned char ��,
	// 1���������,2����Ͷ�ݸ��(��),3������ͨ·����Դ,4��������,
	// 5����δ֪����������,6����Ŀ�ĵؼ���(����·����Դ);
	list <ResTable_item> ResTable;	//��ԴԤԼ��,��Դ�ĺ��Ĺ���
};