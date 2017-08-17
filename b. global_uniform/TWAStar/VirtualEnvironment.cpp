#include "StdAfx.h"
#include "VirtualEnvironment.h"


VirtualEnvironment::VirtualEnvironment(void)
{
	//����Ŀ�ĵ��ļ�;
#ifdef Linx
	string fname = "//home//fan//Projects//DestFiles_342//DestinationsReal_" + to_string(this->numDestination) +
		".txt";
#else
	string fname = "E:\\Projects\\DestFiles\\DestinationsReal_" + to_string(this->numDestination) +
		".txt";
#endif
	ifstream fin(fname);
	int dest = 0;
	while (fin >> dest)this->Destinations.push_back(dest);
	fin.close();
}


VirtualEnvironment::~VirtualEnvironment(void)
{
	//��Ҫ������������ɾ������,����������;

	int real_row = (map_row - 1) * 3 + 1;
	for (int i = 0; i < real_row; ++i)
	{
		delete[] this->real_map[i];
		delete[] this->posMap[i];
		delete[] this->ResEntity_list[i];
	}
	delete[] this->real_map;
	delete[] this->posMap;
	delete[] this->ResEntity_list;
	robot_list.clear();

	delete[] this->desSet;
	delete[] this->pitSet;
	delete[] this->bufSet;
	delete[] this->outPortSet;      //�����ڼ���
	this->Destinations.clear();		//����Ŀ�ĵؼ���
}

//����ʱ����;
inline timespec diff(timespec & start, timespec & end)
{
	timespec temp;
	if ((end.tv_nsec - start.tv_nsec) < 0)
	{
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	}
	else
	{
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return temp;
}

//ʵ����ѭ��
void VirtualEnvironment::RunSimulation()
{
#ifdef Linx
	timespec start, end;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
#endif
	/* ��ʾ��Ϣ */
	cout << "\n\n ******** Real Map Size : " << (map_row - 1) * 3 + 1 << " rows and " << (map_col - 1) * 3 + 1 << " columns ******** \n";
	cout << "  With: " << this->numOutPort << " OutPorts\n";
	cout << "  With: " << this->numDestination << " Destinations\n";
	cout << "  With: " << this->numRobot << " Robots\n";

	for (int i = 0; i < this->runSlots; ++i)
		StepRobot();

#ifndef Linx
	cout << "\n In this experiment...  :  " << this->numRobot << " Robots completed duty cycle in " << this->runSlots << " time slots ...\n\n";
#else
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);
	cout << "\n This experiment runs in " << diff(start, end).tv_sec << " s " << diff(start, end).tv_nsec / 1000000 << " ms ...\n\n";
#endif
	cout << "   In " << this->runSlots << " time slots, total finish " << this->totalCount << " Cargo .\n";
	cout << "   In " << this->runSlots << " time slots, max active robots " << this->maxActiveRobots << " to explore .\n";
	cout << "   In " << this->runSlots << " time slots, robots total fail " << this->failRCount << " times to search .\n";
	cout << "........................................................................\n";
}


void VirtualEnvironment::CreateEnvSLSD()			//����ģ��һ�ĳ���
{
	//����ָ����С����Դ��ͼ
	//����ָ����С�������ͼ
	int real_row = (map_row - 1) * 3 + 1;
	int real_col = (map_col - 1) * 3 + 1;

	this->real_map = new unsigned char * [real_row];//���ٵ�ͼ�ռ�
	this->posMap = new list <int> *[real_row];		//���ٵ�ͼ�ռ�
	this->ResEntity_list = new ResEntity *[real_row];
	for (int i = 0; i < real_row; ++i)
	{
		this->real_map[i] = new unsigned char[real_col];
		this->posMap[i] = new list <int>[real_col];
		this->ResEntity_list[i] = new ResEntity [real_col];
	}

	for (int i = 0; i < real_row; ++i)
		for (int j = 0; j < real_col; ++j)
		{
			if (i % 3 == 0 && i % 6 == 0 && j % 3 != 0)
				this->real_map[i][j] = Tile_LEFT;	//��
			else if (i % 3 == 0 && i % 6 != 0 && j % 3 != 0)
				this->real_map[i][j] = Tile_RIGHT;	//��
			else if (j % 3 == 0 && j % 6 == 0 && i % 3 != 0)
				this->real_map[i][j] = Tile_DOWN;	//��
			else if (j % 3 == 0 && j % 6 != 0 && i % 3 != 0)
				this->real_map[i][j] = Tile_UP;	//��

			else if (i % 3 == 0 && i % 6 == 0 && j % 3 == 0 && j % 6 != 0)
				this->real_map[i][j] = Tile_UP_LEFT;	//��
			else if (i % 3 == 0 && i % 6 == 0 && j % 3 == 0 && j % 6 == 0)
				this->real_map[i][j] = Tile_DOWN_LEFT;	//��
			else if (i % 3 == 0 && i % 6 != 0 && j % 3 == 0 && j % 6 != 0)
				this->real_map[i][j] = Tile_UP_RIGHT;	//��
			else if (i % 3 == 0 && i % 6 != 0 && j % 3 == 0 && j % 6 == 0)
				this->real_map[i][j] = Tile_DOWN_RIGHT;	//��
			else
			{
				this->real_map[i][j] = Tile_Lock;
				this->ResEntity_list[i][j].Rtype = 5;//δ֪����������;
				continue;
			}
		}

	//PrintMap(this->real_map, real_row, real_col);

	this->CreateDest_global_uniform();	//b. ����
	this->CreateOutPort();	//���������ں���
	this->CreateRobot();	//���������˺���

	for (int i = 0; i < numOutPort; i++)
	{
		//cout << outPortSet[i].i << " - " << outPortSet[i].j << endl;
		ResEntity_list[outPortSet[i].i][outPortSet[i].j].Rtype = 1;
	}

	for (int i = 0; i < numDestination; i++)
	{
		//cout << desSet[i].i << " - " << desSet[i].j << "\n";
		ResEntity_list[desSet[i].i][desSet[i].j].Rtype = 6; //����Ŀ�ĵ�����Ϊ6;
		this->real_map[desSet[i].i][desSet[i].j] = Tile_Open;//���û�����Ϊopen����;

		ResEntity_list[desSet[i].i][desSet[i].j + 1].Rtype = 4; //���û���������Ϊ4;
		this->real_map[desSet[i].i][desSet[i].j + 1] = Tile_Open;//���û�����Ϊopen����;

		ResEntity_list[desSet[i].i][desSet[i].j - 1].Rtype = 2; //���÷ּ�������Ϊ2;
		this->real_map[desSet[i].i][desSet[i].j - 1] = Tile_Lock;//���÷ּ���Ϊ�����У��ӣ�;
	}
}


//��ÿ��ʱ����ﶼҪ��������
void VirtualEnvironment::StepRobot()
{
	CheackCorrectness();//����㷨��ȷ��;

	//����������ʱ���,ֻ����ͨ·����Դ��ҪԤԼ������;
	for (int i = 0; i < (map_row - 1) * 3 + 1; ++i)
		for (int j = 0; j < (map_col - 1) * 3 + 1; ++j)
			if (ResEntity_list[i][j].Rtype == 3 || ResEntity_list[i][j].Rtype == 6)
				this->ResEntity_list[i][j].clearResTable(this->time_slot);

	//���л����˵���
	int maRC = 0;//���������ͳ��;
	vector <Robot>::iterator it = this->robot_list.begin();//���������飬�±��ʾ��ID
	for (; it != this->robot_list.end(); ++it)
	{
		if (it->ignition_slot <= this->time_slot)		  //��������ʱ��۲ŻῪʼ
		{
			if (it->stay || it->isFinMission == 1 || it->isFinMission == 3)
			{
				//cout << endl << it->Rid << " �Ż�������ʱ��� " << this->time_slot << " ˢ�������� ...\n**********************************\n";
				if (it->getRoute(this->time_slot, this->ResEntity_list, this->real_map, (this->map_row - 1) * 3 + 1, (this->map_col - 1) * 3 + 1, this->bufSet, this->numDestination))
				{
					//Ѱ·�ɹ�,�������߱�־λ;
					if (it->isFinMission == 1)it->isFinMission = 2;
					else if (it->isFinMission == 3 || it->isFinMission == 5)
						it->isFinMission = 4;
					it->stay = false;//Ѱ·�ɹ��Ͳ�ͣ����;
				}
				else//Ѱ·ʧ�ܣ��˿���������˴��ȴ�����
				{
					it->stay = true;	//Ѱ·ʧ�ܣ���Ҫ����Ѱ·����Ҫǰ��;
					this->failRCount++;//Ѱ·ʧ���ۼ�ͳ��;
					//Ѱ·ʧ�ܣ��ڳ����ںͷּ���������������Ҫ����;
					if (it->isFinMission == 1)//������Ѱ·ʧ�ܣ�(������)����Ѱ·;
					{
						//�������
						it->currRoute.clear();
						//cout << "Start position find path fail!\n";
					}
					else if (it->isFinMission == 3)//Ŀ�Ŀ�Ѱ·ʧ�ܣ�������������Ѱ·;
					{
						it->currPos.i = it->currPos.i;
						it->currPos.j = it->currPos.j + 1;
						it->isFinMission = 5;		 //���뻺������־λ;
						//cout << "End position find path fail!\n";
					}
					else if (it->isFinMission == 5)//������Ѱ·ʧ�ܣ��ȴ���Ѱ·;
					{
						//this->failRCount++;			 //Ѱ·ʧ���ۼ�ͳ��;
						//cout << "Buffer find path fail!\n";
					}
					//����;��¼����λ��;
					//continue;
				}
			}
			//������ǰ��
			//�ж��Ƿ��ڴ�ͣ��?
			if (it->stay)continue;
			else if (it->currRoute.begin()->leave_slot - it->currRoute.begin()->enter_slot > 0)
				it->currRoute.begin()->enter_slot++;//ͣ���Ļ�;
			else
			{
				++maRC;									   //�ۼƻ����������;
				it->currRoute.pop_front();				   //ɾ����Ҫǰ����һ��
				it->currPos.i = it->currRoute.begin()->i;  //������ǰ��һ��
				it->currPos.j = it->currRoute.begin()->j;  //������ǰ��һ��

														   //����ǵ���Ŀ�ĵصĻ�
				if (it->currPos.i == it->destPort.i &&
					it->currPos.j == it->destPort.j && it->isFinMission == 2)
				{
					this->totalCount++;//ͳ��ֵ��������һ :�����
					//�Ƚ���Ŀ�ĵغͳ�����
					Point temp = it->destPort;
					it->destPort = it->startPort;
					it->startPort = temp;
					//�������
					it->currRoute.clear();
					it->isFinMission = 3;//״̬�����Ŀ�Ŀڳ���̬
				}
				//����Ƿ����˳����صĻ�
				else if (it->currPos.i == it->destPort.i &&
					it->currPos.j == it->destPort.j && it->isFinMission == 4)
				{
					//���ó����أ�ԭ���ĳ�����
					it->startPort = it->destPort;
					//��������Ŀ�ĵ�,�����ļ�ԭ��
					int destport = this->Destinations.front();//ȡ����Ŀ�ĵ�port ID
					this->Destinations.pop_front();
					it->destPort = desSet[destport];
					//�������
					it->currRoute.clear();
					it->isFinMission = 1;
				}
			}
		}
	}

	if (maRC > this->maxActiveRobots)this->maxActiveRobots = maRC;
	this->time_slot ++;	//����ʱ���ǰ��һ��
}


void VirtualEnvironment::PrintMap(unsigned char ** map, int real_row, int real_col)
{	//�����ͼ
	int i, j;
	for (i = 0; i<real_row; i++)
	{
		for (j = 0; j<real_col; j++)
		{
			if (map[i][j] == Tile_Open || map[i][j] == Tile_UP ||
				map[i][j] == Tile_DOWN || map[i][j] == Tile_LEFT
				|| map[i][j] == Tile_RIGHT || map[i][j] == Tile_UP_LEFT
				|| map[i][j] == Tile_UP_RIGHT || map[i][j] == Tile_DOWN_LEFT
				|| map[i][j] == Tile_DOWN_RIGHT) {
				printf("��");
			}
			else if (map[i][j] == Tile_Lock) {
				printf("��");
			}
			else if (map[i][j] == Tile_Start) {
				printf("��");
			}
			else if (map[i][j] == Tile_End) {
				printf("��");
			}
			else if (map[i][j] == Tile_Path) {
				printf("��");
			}
		}
		printf("\n");
	}
	printf("\n\n");
}

//a. ����
void VirtualEnvironment::CreateDest_centralized_uniform()
{
	int maxnumDestination = (map_row - 1) * (map_col - 2) * 2;
	if (this->numDestination > maxnumDestination)
	{
		cout << "\nYou can't exceed Maximum Destination number \nalready set to default : " << maxnumDestination << "  .\n\n";
		this->numDestination = maxnumDestination;
	}
	if (this->numDestination < 4)
	{
		cout << "\nToo small number of Destinations  \nalready set to default : " << 4 << "  .\n\n";
		this->numDestination = 4;
	}

	this->desSet = new Point[this->numDestination];
	this->pitSet = new Point[this->numDestination];
	this->bufSet = new Point[this->numDestination];

	int sqRow = ((map_row - 1) * 2);
	int sqCol = map_col - 2;//���� ������;

	int cx0 = sqRow / 2;
	int cy0 = sqCol / 2;

	int cx1 = cx0;
	int cy1 = cy0 + 1;

	int cx2 = cx0 + 1;
	int cy2 = cy0 + 1;

	int cx3 = cx0 + 1;
	int cy3 = cy0;

	int rowCount[4] = { 0 };//ÿ������ͳ��;
	int rowLimits = (map_col - 2) / 2;//ÿ���������ֵ;
	for (int t = 0; t < this->numDestination; ++t)
	{
		if (t % 4 == 0)//�׸�;
		{
			int x = cx0;
			int y = cy0 - rowCount[0];
			//����������
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//����������
			desSet[t].j = y * 3;//��άͼ�ϵ��������;

			rowCount[0]++;
			if (rowCount[0] >= rowLimits)
			{
				--cx0;
				rowCount[0] = 0;
			}
		}
		if (t % 4 == 1)//1��;
		{
			int x = cx1;
			int y = cy1 + rowCount[1];
			//����������
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//����������
			desSet[t].j = y * 3;//��άͼ�ϵ��������;

			rowCount[1]++;
			if (rowCount[1] >= rowLimits)
			{
				--cx1;
				rowCount[1] = 0;
			}
		}
		if (t % 4 == 2)//2��;
		{
			int x = cx2;
			int y = cy2 + rowCount[2];
			//����������
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//����������
			desSet[t].j = y * 3;//��άͼ�ϵ��������;

			rowCount[2]++;
			if (rowCount[2] >= rowLimits)
			{
				++cx2;
				rowCount[2] = 0;
			}
		}
		if (t % 4 == 3)//�׸�;
		{
			int x = cx3;
			int y = cy3 - rowCount[3];
			//����������
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//����������
			desSet[t].j = y * 3;//��άͼ�ϵ��������;

			rowCount[3]++;
			if (rowCount[3] >= rowLimits)
			{
				++cx3;
				rowCount[3] = 0;
			}
		}
	}

	//�Էּ�ڣ��������ĸ�ֵ;
	for (int t = 0; t < numDestination; ++t)
	{
		pitSet[t].i = desSet[t].i;
		pitSet[t].j = desSet[t].j - 1;

		bufSet[t].i = desSet[t].i;
		bufSet[t].j = desSet[t].j + 1;
	}
}

//b. ����
//����Ŀ�ĵ��Լ� �ּ�ڣ�����������;
void VirtualEnvironment::CreateDest_global_uniform()
{
	int maxnumDestination = (map_row - 1) * (map_col - 2) * 2;
	if (this->numDestination > maxnumDestination)
	{
		cout << "\nYou can't exceed Maximum Destination number \nalready set to default : " << maxnumDestination << "  .\n\n";
		this->numDestination = maxnumDestination;
	}

	this->desSet = new Point[this->numDestination];
	this->pitSet = new Point[this->numDestination];
	this->bufSet = new Point[this->numDestination];

	int numPerCol = this->numDestination / (map_col - 2);//ÿ��Ӧ���е���Ŀ;
	if (numPerCol < 1)
	{
		int colInteval = (map_col - 2) / this->numDestination;
		for (int t = 0; t < this->numDestination; ++t)
		{
			//����������
			int relRow = (map_col - 2) / 2;
			if (relRow % 2 == 0)desSet[t].i = relRow / 2 + relRow + 1;
			else desSet[t].i = (relRow - 1) / 2 + relRow + 1;
			//����������
			desSet[t].j = ((colInteval * t) % (map_col - 2) + 1) * 3;//��άͼ�ϵ��������;
		}
	}
	else
	{
		int rowInteval;
		if (this->numDestination % (map_col - 2) == 0)//ÿ��Ӧ���е���Ŀ,����������
			rowInteval = ((map_row - 1) * 2) / (numPerCol + 1);
		else rowInteval = ((map_row - 1) * 2) / (numPerCol + 2);

		int before = numPerCol * (map_col - 2);
		int left = this->numDestination - before;
		int numCount = 0;
		for (int t = 0; t < (map_col - 2); ++t)
		{
			for (int k = 1; k <= numPerCol; ++k)
			{
				//����������
				int relRow = (rowInteval * k);
				if (relRow % 2 == 0)desSet[numCount].i = relRow / 2 + relRow + 1;
				else desSet[numCount].i = (relRow - 1) / 2 + relRow + 1;
				//����������
				desSet[numCount++].j = (t + 1) * 3;//��άͼ�ϵ��������;
			}
		}
		for (int t = 0; t < left; ++t)
		{
			//����������
			int relRow = (rowInteval * (numPerCol + 1));
			if (relRow % 2 == 0)desSet[numCount].i = relRow / 2 + relRow + 1;
			else desSet[numCount].i = (relRow - 1) / 2 + relRow + 1;
			//����������
			desSet[numCount++].j = (t + 1) * 3;//��άͼ�ϵ��������;
		}
	}
	//�Էּ�ڣ��������ĸ�ֵ;
	for (int t = 0; t < numDestination; ++t)
	{
		pitSet[t].i = desSet[t].i;
		pitSet[t].j = desSet[t].j - 1;

		bufSet[t].i = desSet[t].i;
		bufSet[t].j = desSet[t].j + 1;
	}
}

//c. ����
void VirtualEnvironment::CreateDest_near_outport()
{
	int maxnumDestination = (map_row - 1) * (map_col - 2) * 2;
	if (this->numDestination > maxnumDestination)
	{
		cout << "\nYou can't exceed Maximum Destination number \nalready set to default : " << maxnumDestination << "  .\n\n";
		this->numDestination = maxnumDestination;
	}
	if (this->numDestination < 4)
	{
		cout << "\nToo small number of Destinations  \nalready set to default : " << 4 << "  .\n\n";
		this->numDestination = 4;
	}

	this->desSet = new Point[this->numDestination];
	this->pitSet = new Point[this->numDestination];
	this->bufSet = new Point[this->numDestination];

	int sqRow = ((map_row - 1) * 2);
	int sqCol = map_col - 2;//���� ������;

	int cx0 = 1;
	int cy0 = 1;

	int cx1 = 1;
	int cy1 = sqCol;

	int cx2 = sqRow;
	int cy2 = sqCol;

	int cx3 = sqRow;
	int cy3 = 1;

	int rowCount[4] = { 0 };//ÿ������ͳ��;
	int rowLimits = (map_col - 2) / 2;//ÿ���������ֵ;
	for (int t = 0; t < this->numDestination; ++t)
	{
		if (t % 4 == 0)//�׸�;
		{
			int x = cx0;
			int y = cy0 + rowCount[0];
			//����������
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//����������
			desSet[t].j = y * 3;//��άͼ�ϵ��������;

			rowCount[0]++;
			if (rowCount[0] >= rowLimits)
			{
				++cx0;
				rowCount[0] = 0;
			}
		}
		if (t % 4 == 1)//1��;
		{
			int x = cx1;
			int y = cy1 - rowCount[1];
			//����������
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//����������
			desSet[t].j = y * 3;//��άͼ�ϵ��������;

			rowCount[1]++;
			if (rowCount[1] >= rowLimits)
			{
				++cx1;
				rowCount[1] = 0;
			}
		}
		if (t % 4 == 2)//2��;
		{
			int x = cx2;
			int y = cy2 - rowCount[2];
			//����������
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//����������
			desSet[t].j = y * 3;//��άͼ�ϵ��������;

			rowCount[2]++;
			if (rowCount[2] >= rowLimits)
			{
				--cx2;
				rowCount[2] = 0;
			}
		}
		if (t % 4 == 3)//�׸�;
		{
			int x = cx3;
			int y = cy3 + rowCount[3];
			//����������
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//����������
			desSet[t].j = y * 3;//��άͼ�ϵ��������;

			rowCount[3]++;
			if (rowCount[3] >= rowLimits)
			{
				--cx3;
				rowCount[3] = 0;
			}
		}
	}

	//�Էּ�ڣ��������ĸ�ֵ;
	for (int t = 0; t < numDestination; ++t)
	{
		pitSet[t].i = desSet[t].i;
		pitSet[t].j = desSet[t].j - 1;

		bufSet[t].i = desSet[t].i;
		bufSet[t].j = desSet[t].j + 1;
	}
}

//d. ����
bool isDAxi(list<Point> & axi, int & x, int & y)
{
	if (axi.size() == 0)return false;
	list<Point>::iterator it = axi.begin();
	for (; it != axi.end(); ++it)
	{
		if (it->i == x && it->j == y)return true;
	}
	return false;
}
//d. ����
void VirtualEnvironment::CreateDest_random()
{
	int maxnumDestination = (map_row - 1) * (map_col - 2) * 2;
	if (this->numDestination > maxnumDestination)
	{
		cout << "\nYou can't exceed Maximum Destination number \nalready set to default : " << maxnumDestination << "  .\n\n";
		this->numDestination = maxnumDestination;
	}
	if (this->numDestination < 4)
	{
		cout << "\nToo small number of Destinations  \nalready set to default : " << 4 << "  .\n\n";
		this->numDestination = 4;
	}

	this->desSet = new Point[this->numDestination];
	this->pitSet = new Point[this->numDestination];
	this->bufSet = new Point[this->numDestination];

	int x = 0, y = 0;
	int sqRow = ((map_row - 1) * 2);
	int sqCol = map_col - 2;//���� ������;
	list<Point> axi;
	for (int t = 0; t < this->numDestination; ++t)
	{
		x = rand() % sqRow + 1;
		y = rand() % sqCol + 1;
		while (isDAxi(axi, x, y))
		{
			x = rand() % sqRow + 1;
			y = rand() % sqCol + 1;
		}
		axi.push_back(Point(x, y));
		//����������
		if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
		else desSet[t].i = (x - 1) / 2 + x;
		//����������
		desSet[t].j = y * 3;//��άͼ�ϵ��������;
	}
	//�Էּ�ڣ��������ĸ�ֵ;
	for (int t = 0; t < numDestination; ++t)
	{
		pitSet[t].i = desSet[t].i;
		pitSet[t].j = desSet[t].j - 1;

		bufSet[t].i = desSet[t].i;
		bufSet[t].j = desSet[t].j + 1;
	}
}

//����Ĭ�ϳ�����λ��
void VirtualEnvironment::CreateOutPort()
{
	//�˴��㷨����δ������⣬����Ĭ�ϰ���ȫ��������Χ�������ϵ�ԭ����ó�����
	int maxnumOutPort = 2 * map_col + 2 * (map_row - 2);
	if (this->numOutPort > maxnumOutPort)
	{
		cout << "\nYou can't exceed Maximum Output Port number \nalready set to default : " << maxnumOutPort << "  .\n\n";
		this->numOutPort = maxnumOutPort;
	}

	this->outPortSet = new Point[this->numOutPort];				//��̬�����洢�ռ�

	//������Ĭ��Ϊ4�������
	outPortSet[0].i = 0; outPortSet[0].j = 0;				//��һ��ê��λ��
	outPortSet[1].i = 0; outPortSet[1].j = (map_col - 1) * 3;
	outPortSet[2].i = (map_row - 1) * 3; outPortSet[2].j = 0;
	outPortSet[3].i = (map_row - 1) * 3; outPortSet[3].j = (map_col - 1) * 3;


	int longEdgePerPort = double(numOutPort - 4) / (map_row + map_col - 4) * (map_col - 2) / 2;//ÿ���᳤�߶˿�����
	int shortEdgePerPort = double(numOutPort - 4) / (map_row + map_col - 4) * (map_row - 2) / 2;//ÿ�����̱߶˿�����
	int edgeLastPort = numOutPort - 4 - 2 * (longEdgePerPort + shortEdgePerPort);//ÿ��ʣ��˿�����			//ÿ����ʣ��˿�����
	int edgePorts[4] = { longEdgePerPort, shortEdgePerPort, longEdgePerPort, shortEdgePerPort };
	for (int k = 0; k < 4; ++k, --edgeLastPort)
	{
		if (edgeLastPort > 0)edgePorts[k] += 1;
		else break;
	}
	int count = 4;
	int inteval = 1;
	//k == 0,�˱�Ŵ��Ϻ��Ϊ0����ʱ���ŵ�3
	if(map_col / (edgePorts[0] + 1) > 1)inteval = map_col / (edgePorts[0] + 1);
	for (int m = 1; m <= edgePorts[0]; ++m)
	{
		outPortSet[count].i = 0;	//0��
		outPortSet[count++].j = m * inteval * 3;//�����ʵ��ͼ����
	}
	//k == 1
	inteval = 1;	//reset this
	if (map_row / (edgePorts[1] + 1) > 1)inteval = map_row / (edgePorts[1] + 1);
	for (int m = 1; m <= edgePorts[1]; ++m)
	{
		outPortSet[count].i = m * inteval * 3;	//0��
		outPortSet[count++].j = (map_col - 1) * 3;//�����ʵ��ͼ����
	}
	//k == 2
	inteval = 1;	//reset this
	if (map_col / (edgePorts[2] + 1) > 1)inteval = map_col / (edgePorts[2] + 1);
	for (int m = 1; m <= edgePorts[2]; ++m)
	{
		outPortSet[count].i = (map_row - 1) * 3;	//�����
		outPortSet[count++].j = m * inteval * 3;//�����ʵ��ͼ����
	}
	//k == 3
	inteval = 1;	//reset this
	if (map_row / (edgePorts[3] + 1) > 1)inteval = map_row / (edgePorts[3] + 1);
	for (int m = 1; m <= edgePorts[3]; ++m)
	{
		outPortSet[count].i = m * inteval * 3;	//�����
		outPortSet[count++].j = 0;					//��ǰ0��
	}
}

void VirtualEnvironment::CreateRobot()
{
	int port = 1;
	int port_count = 1;
	int destport = 1;
	for (int k = 0; k < this->numRobot; k++)
	{
        port = k % this->numOutPort;	//Ӧ�����ĳ����˿�port ID
        port_count = k / this->numOutPort + 1;//�����ĳ����˿ڳ������Σ�Ҳ����ʱ���
		destport = this->Destinations.front();//ȡ����Ŀ�ĵ�port ID
		this->Destinations.pop_front();

		robot_list.push_back(Robot(k, outPortSet[port], desSet[destport], port_count));
	}
}

bool VirtualEnvironment::CheackCorrectness()
{
	int real_row = (map_row - 1) * 3 + 1;
	int real_col = (map_col - 1) * 3 + 1;
	//ÿ��ͳ��ǰ����;
	for (int i = 0; i < real_row; ++i)
		for (int j = 0; j < real_col; ++j)
			this->posMap[i][j].clear();

	vector <Robot> ::iterator rit = robot_list.begin();
	for (; rit != robot_list.end(); ++rit)
	{
		this->posMap[rit->currPos.i][rit->currPos.j].push_back(rit->Rid);
	}
	//ͳ���㷨��ȷ��;
	for (int i = 0; i < real_row; ++i)
		for (int j = 0; j < real_col; ++j)
		{
			if (posMap[i][j].size() > 1)//λ��ͳ�Ƴ�����������;
			{
				if (ResEntity_list[i][j].Rtype == 3 ||
					ResEntity_list[i][j].Rtype == 2 ||
					ResEntity_list[i][j].Rtype == 5 ||
					ResEntity_list[i][j].Rtype == 6)
				{
					cout << "** In time slot " << this->time_slot << ", Collision.. ";
					cout << "Position : (" << i << "," << j << ")  type: " << int(ResEntity_list[i][j].Rtype) << endl;
					return false;//������·����Դ������!
				}
			}
		}

	return true;	//Ĭ�Ϸ�����ȷ;
}
