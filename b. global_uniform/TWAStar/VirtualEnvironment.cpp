#include "StdAfx.h"
#include "VirtualEnvironment.h"


VirtualEnvironment::VirtualEnvironment(void)
{
	//载入目的地文件;
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
	//需要在析构函数里删除数据,进行清理步骤;

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
	delete[] this->outPortSet;      //出货口集合
	this->Destinations.clear();		//载入目的地集合
}

//计算时间差函数;
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

//实验主循环
void VirtualEnvironment::RunSimulation()
{
#ifdef Linx
	timespec start, end;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
#endif
	/* 提示信息 */
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


void VirtualEnvironment::CreateEnvSLSD()			//产生模拟一的场景
{
	//生成指定大小的资源地图
	//产生指定大小的虚拟地图
	int real_row = (map_row - 1) * 3 + 1;
	int real_col = (map_col - 1) * 3 + 1;

	this->real_map = new unsigned char * [real_row];//开辟地图空间
	this->posMap = new list <int> *[real_row];		//开辟地图空间
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
				this->real_map[i][j] = Tile_LEFT;	//←
			else if (i % 3 == 0 && i % 6 != 0 && j % 3 != 0)
				this->real_map[i][j] = Tile_RIGHT;	//→
			else if (j % 3 == 0 && j % 6 == 0 && i % 3 != 0)
				this->real_map[i][j] = Tile_DOWN;	//←
			else if (j % 3 == 0 && j % 6 != 0 && i % 3 != 0)
				this->real_map[i][j] = Tile_UP;	//↑

			else if (i % 3 == 0 && i % 6 == 0 && j % 3 == 0 && j % 6 != 0)
				this->real_map[i][j] = Tile_UP_LEFT;	//→
			else if (i % 3 == 0 && i % 6 == 0 && j % 3 == 0 && j % 6 == 0)
				this->real_map[i][j] = Tile_DOWN_LEFT;	//→
			else if (i % 3 == 0 && i % 6 != 0 && j % 3 == 0 && j % 6 != 0)
				this->real_map[i][j] = Tile_UP_RIGHT;	//→
			else if (i % 3 == 0 && i % 6 != 0 && j % 3 == 0 && j % 6 == 0)
				this->real_map[i][j] = Tile_DOWN_RIGHT;	//→
			else
			{
				this->real_map[i][j] = Tile_Lock;
				this->ResEntity_list[i][j].Rtype = 5;//未知不可行区域;
				continue;
			}
		}

	//PrintMap(this->real_map, real_row, real_col);

	this->CreateDest_global_uniform();	//b. 类型
	this->CreateOutPort();	//创建出货口函数
	this->CreateRobot();	//创建机器人函数

	for (int i = 0; i < numOutPort; i++)
	{
		//cout << outPortSet[i].i << " - " << outPortSet[i].j << endl;
		ResEntity_list[outPortSet[i].i][outPortSet[i].j].Rtype = 1;
	}

	for (int i = 0; i < numDestination; i++)
	{
		//cout << desSet[i].i << " - " << desSet[i].j << "\n";
		ResEntity_list[desSet[i].i][desSet[i].j].Rtype = 6; //设置目的地类型为6;
		this->real_map[desSet[i].i][desSet[i].j] = Tile_Open;//设置缓冲区为open开放;

		ResEntity_list[desSet[i].i][desSet[i].j + 1].Rtype = 4; //设置缓冲区类型为4;
		this->real_map[desSet[i].i][desSet[i].j + 1] = Tile_Open;//设置缓冲区为open开放;

		ResEntity_list[desSet[i].i][desSet[i].j - 1].Rtype = 2; //设置分拣格口类型为2;
		this->real_map[desSet[i].i][desSet[i].j - 1] = Tile_Lock;//设置分拣格口为不可行（坑）;
	}
}


//在每个时间槽里都要做的事情
void VirtualEnvironment::StepRobot()
{
	CheackCorrectness();//检查算法正确性;

	//先清理无用时间槽,只有普通路径资源需要预约和清理;
	for (int i = 0; i < (map_row - 1) * 3 + 1; ++i)
		for (int j = 0; j < (map_col - 1) * 3 + 1; ++j)
			if (ResEntity_list[i][j].Rtype == 3 || ResEntity_list[i][j].Rtype == 6)
				this->ResEntity_list[i][j].clearResTable(this->time_slot);

	//进行机器人调度
	int maRC = 0;//最大活动机器人统计;
	vector <Robot>::iterator it = this->robot_list.begin();//机器人数组，下标表示其ID
	for (; it != this->robot_list.end(); ++it)
	{
		if (it->ignition_slot <= this->time_slot)		  //到达启动时间槽才会开始
		{
			if (it->stay || it->isFinMission == 1 || it->isFinMission == 3)
			{
				//cout << endl << it->Rid << " 号机器人在时间槽 " << this->time_slot << " 刷新搜索中 ...\n**********************************\n";
				if (it->getRoute(this->time_slot, this->ResEntity_list, this->real_map, (this->map_row - 1) * 3 + 1, (this->map_col - 1) * 3 + 1, this->bufSet, this->numDestination))
				{
					//寻路成功,更换行走标志位;
					if (it->isFinMission == 1)it->isFinMission = 2;
					else if (it->isFinMission == 3 || it->isFinMission == 5)
						it->isFinMission = 4;
					it->stay = false;//寻路成功就不停留了;
				}
				else//寻路失败，此快件放弃？此处等待策略
				{
					it->stay = true;	//寻路失败，需要不断寻路，不要前进;
					this->failRCount++;//寻路失败累计统计;
					//寻路失败，在出货口和分拣口有两种情况，需要讨论;
					if (it->isFinMission == 1)//出货口寻路失败，(换货物)，再寻路;
					{
						//重设参数
						it->currRoute.clear();
						//cout << "Start position find path fail!\n";
					}
					else if (it->isFinMission == 3)//目的口寻路失败，进缓冲区，再寻路;
					{
						it->currPos.i = it->currPos.i;
						it->currPos.j = it->currPos.j + 1;
						it->isFinMission = 5;		 //进入缓冲区标志位;
						//cout << "End position find path fail!\n";
					}
					else if (it->isFinMission == 5)//缓冲区寻路失败，等待再寻路;
					{
						//this->failRCount++;			 //寻路失败累计统计;
						//cout << "Buffer find path fail!\n";
					}
					//保留;记录出错位置;
					//continue;
				}
			}
			//机器人前进
			//判断是否在此停留?
			if (it->stay)continue;
			else if (it->currRoute.begin()->leave_slot - it->currRoute.begin()->enter_slot > 0)
				it->currRoute.begin()->enter_slot++;//停留的话;
			else
			{
				++maRC;									   //累计活动机器人数量;
				it->currRoute.pop_front();				   //删除需要前进的一步
				it->currPos.i = it->currRoute.begin()->i;  //机器人前进一步
				it->currPos.j = it->currRoute.begin()->j;  //机器人前进一步

														   //如果是到达目的地的话
				if (it->currPos.i == it->destPort.i &&
					it->currPos.j == it->destPort.j && it->isFinMission == 2)
				{
					this->totalCount++;//统计值完成又完成一 :★★★★
					//先交换目的地和出发地
					Point temp = it->destPort;
					it->destPort = it->startPort;
					it->startPort = temp;
					//重设参数
					it->currRoute.clear();
					it->isFinMission = 3;//状态变成在目的口出发态
				}
				//如果是返回了出发地的话
				else if (it->currPos.i == it->destPort.i &&
					it->currPos.j == it->destPort.j && it->isFinMission == 4)
				{
					//设置出发地，原本的出发地
					it->startPort = it->destPort;
					//重新设置目的地,载入文件原则
					int destport = this->Destinations.front();//取得其目的地port ID
					this->Destinations.pop_front();
					it->destPort = desSet[destport];
					//重设参数
					it->currRoute.clear();
					it->isFinMission = 1;
				}
			}
		}
	}

	if (maRC > this->maxActiveRobots)this->maxActiveRobots = maRC;
	this->time_slot ++;	//运行时间槽前进一步
}


void VirtualEnvironment::PrintMap(unsigned char ** map, int real_row, int real_col)
{	//输出地图
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
				printf("□");
			}
			else if (map[i][j] == Tile_Lock) {
				printf("■");
			}
			else if (map[i][j] == Tile_Start) {
				printf("☆");
			}
			else if (map[i][j] == Tile_End) {
				printf("★");
			}
			else if (map[i][j] == Tile_Path) {
				printf("●");
			}
		}
		printf("\n");
	}
	printf("\n\n");
}

//a. 类型
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
	int sqCol = map_col - 2;//方阵 行与列;

	int cx0 = sqRow / 2;
	int cy0 = sqCol / 2;

	int cx1 = cx0;
	int cy1 = cy0 + 1;

	int cx2 = cx0 + 1;
	int cy2 = cy0 + 1;

	int cx3 = cx0 + 1;
	int cy3 = cy0;

	int rowCount[4] = { 0 };//每区块行统计;
	int rowLimits = (map_col - 2) / 2;//每区块行最大值;
	for (int t = 0; t < this->numDestination; ++t)
	{
		if (t % 4 == 0)//首个;
		{
			int x = cx0;
			int y = cy0 - rowCount[0];
			//设置行坐标
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//设置列坐标
			desSet[t].j = y * 3;//二维图上的相对坐标;

			rowCount[0]++;
			if (rowCount[0] >= rowLimits)
			{
				--cx0;
				rowCount[0] = 0;
			}
		}
		if (t % 4 == 1)//1个;
		{
			int x = cx1;
			int y = cy1 + rowCount[1];
			//设置行坐标
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//设置列坐标
			desSet[t].j = y * 3;//二维图上的相对坐标;

			rowCount[1]++;
			if (rowCount[1] >= rowLimits)
			{
				--cx1;
				rowCount[1] = 0;
			}
		}
		if (t % 4 == 2)//2个;
		{
			int x = cx2;
			int y = cy2 + rowCount[2];
			//设置行坐标
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//设置列坐标
			desSet[t].j = y * 3;//二维图上的相对坐标;

			rowCount[2]++;
			if (rowCount[2] >= rowLimits)
			{
				++cx2;
				rowCount[2] = 0;
			}
		}
		if (t % 4 == 3)//首个;
		{
			int x = cx3;
			int y = cy3 - rowCount[3];
			//设置行坐标
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//设置列坐标
			desSet[t].j = y * 3;//二维图上的相对坐标;

			rowCount[3]++;
			if (rowCount[3] >= rowLimits)
			{
				++cx3;
				rowCount[3] = 0;
			}
		}
	}

	//对分拣口，缓冲区的赋值;
	for (int t = 0; t < numDestination; ++t)
	{
		pitSet[t].i = desSet[t].i;
		pitSet[t].j = desSet[t].j - 1;

		bufSet[t].i = desSet[t].i;
		bufSet[t].j = desSet[t].j + 1;
	}
}

//b. 类型
//产生目的地以及 分拣口，缓冲区函数;
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

	int numPerCol = this->numDestination / (map_col - 2);//每列应该有的数目;
	if (numPerCol < 1)
	{
		int colInteval = (map_col - 2) / this->numDestination;
		for (int t = 0; t < this->numDestination; ++t)
		{
			//设置行坐标
			int relRow = (map_col - 2) / 2;
			if (relRow % 2 == 0)desSet[t].i = relRow / 2 + relRow + 1;
			else desSet[t].i = (relRow - 1) / 2 + relRow + 1;
			//设置列坐标
			desSet[t].j = ((colInteval * t) % (map_col - 2) + 1) * 3;//二维图上的相对坐标;
		}
	}
	else
	{
		int rowInteval;
		if (this->numDestination % (map_col - 2) == 0)//每列应该有的数目,整数除尽了
			rowInteval = ((map_row - 1) * 2) / (numPerCol + 1);
		else rowInteval = ((map_row - 1) * 2) / (numPerCol + 2);

		int before = numPerCol * (map_col - 2);
		int left = this->numDestination - before;
		int numCount = 0;
		for (int t = 0; t < (map_col - 2); ++t)
		{
			for (int k = 1; k <= numPerCol; ++k)
			{
				//设置行坐标
				int relRow = (rowInteval * k);
				if (relRow % 2 == 0)desSet[numCount].i = relRow / 2 + relRow + 1;
				else desSet[numCount].i = (relRow - 1) / 2 + relRow + 1;
				//设置列坐标
				desSet[numCount++].j = (t + 1) * 3;//二维图上的相对坐标;
			}
		}
		for (int t = 0; t < left; ++t)
		{
			//设置行坐标
			int relRow = (rowInteval * (numPerCol + 1));
			if (relRow % 2 == 0)desSet[numCount].i = relRow / 2 + relRow + 1;
			else desSet[numCount].i = (relRow - 1) / 2 + relRow + 1;
			//设置列坐标
			desSet[numCount++].j = (t + 1) * 3;//二维图上的相对坐标;
		}
	}
	//对分拣口，缓冲区的赋值;
	for (int t = 0; t < numDestination; ++t)
	{
		pitSet[t].i = desSet[t].i;
		pitSet[t].j = desSet[t].j - 1;

		bufSet[t].i = desSet[t].i;
		bufSet[t].j = desSet[t].j + 1;
	}
}

//c. 类型
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
	int sqCol = map_col - 2;//方阵 行与列;

	int cx0 = 1;
	int cy0 = 1;

	int cx1 = 1;
	int cy1 = sqCol;

	int cx2 = sqRow;
	int cy2 = sqCol;

	int cx3 = sqRow;
	int cy3 = 1;

	int rowCount[4] = { 0 };//每区块行统计;
	int rowLimits = (map_col - 2) / 2;//每区块行最大值;
	for (int t = 0; t < this->numDestination; ++t)
	{
		if (t % 4 == 0)//首个;
		{
			int x = cx0;
			int y = cy0 + rowCount[0];
			//设置行坐标
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//设置列坐标
			desSet[t].j = y * 3;//二维图上的相对坐标;

			rowCount[0]++;
			if (rowCount[0] >= rowLimits)
			{
				++cx0;
				rowCount[0] = 0;
			}
		}
		if (t % 4 == 1)//1个;
		{
			int x = cx1;
			int y = cy1 - rowCount[1];
			//设置行坐标
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//设置列坐标
			desSet[t].j = y * 3;//二维图上的相对坐标;

			rowCount[1]++;
			if (rowCount[1] >= rowLimits)
			{
				++cx1;
				rowCount[1] = 0;
			}
		}
		if (t % 4 == 2)//2个;
		{
			int x = cx2;
			int y = cy2 - rowCount[2];
			//设置行坐标
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//设置列坐标
			desSet[t].j = y * 3;//二维图上的相对坐标;

			rowCount[2]++;
			if (rowCount[2] >= rowLimits)
			{
				--cx2;
				rowCount[2] = 0;
			}
		}
		if (t % 4 == 3)//首个;
		{
			int x = cx3;
			int y = cy3 + rowCount[3];
			//设置行坐标
			if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
			else desSet[t].i = (x - 1) / 2 + x;
			//设置列坐标
			desSet[t].j = y * 3;//二维图上的相对坐标;

			rowCount[3]++;
			if (rowCount[3] >= rowLimits)
			{
				--cx3;
				rowCount[3] = 0;
			}
		}
	}

	//对分拣口，缓冲区的赋值;
	for (int t = 0; t < numDestination; ++t)
	{
		pitSet[t].i = desSet[t].i;
		pitSet[t].j = desSet[t].j - 1;

		bufSet[t].i = desSet[t].i;
		bufSet[t].j = desSet[t].j + 1;
	}
}

//d. 类型
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
//d. 类型
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
	int sqCol = map_col - 2;//方阵 行与列;
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
		//设置行坐标
		if (x % 2 == 0)desSet[t].i = x / 2 + x - 1;
		else desSet[t].i = (x - 1) / 2 + x;
		//设置列坐标
		desSet[t].j = y * 3;//二维图上的相对坐标;
	}
	//对分拣口，缓冲区的赋值;
	for (int t = 0; t < numDestination; ++t)
	{
		pitSet[t].i = desSet[t].i;
		pitSet[t].j = desSet[t].j - 1;

		bufSet[t].i = desSet[t].i;
		bufSet[t].j = desSet[t].j + 1;
	}
}

//创建默认出货口位置
void VirtualEnvironment::CreateOutPort()
{
	//此处算法存在未解决问题，我们默认按照全放在最外围四条边上的原则放置出货口
	int maxnumOutPort = 2 * map_col + 2 * (map_row - 2);
	if (this->numOutPort > maxnumOutPort)
	{
		cout << "\nYou can't exceed Maximum Output Port number \nalready set to default : " << maxnumOutPort << "  .\n\n";
		this->numOutPort = maxnumOutPort;
	}

	this->outPortSet = new Point[this->numOutPort];				//动态创建存储空间

	//出货口默认为4的情况下
	outPortSet[0].i = 0; outPortSet[0].j = 0;				//第一个锚定位置
	outPortSet[1].i = 0; outPortSet[1].j = (map_col - 1) * 3;
	outPortSet[2].i = (map_row - 1) * 3; outPortSet[2].j = 0;
	outPortSet[3].i = (map_row - 1) * 3; outPortSet[3].j = (map_col - 1) * 3;


	int longEdgePerPort = double(numOutPort - 4) / (map_row + map_col - 4) * (map_col - 2) / 2;//每条横长边端口数量
	int shortEdgePerPort = double(numOutPort - 4) / (map_row + map_col - 4) * (map_row - 2) / 2;//每条竖短边端口数量
	int edgeLastPort = numOutPort - 4 - 2 * (longEdgePerPort + shortEdgePerPort);//每边剩余端口容量			//每条边剩余端口数量
	int edgePorts[4] = { longEdgePerPort, shortEdgePerPort, longEdgePerPort, shortEdgePerPort };
	for (int k = 0; k < 4; ++k, --edgeLastPort)
	{
		if (edgeLastPort > 0)edgePorts[k] += 1;
		else break;
	}
	int count = 4;
	int inteval = 1;
	//k == 0,此编号从上横边为0，逆时针编号到3
	if(map_col / (edgePorts[0] + 1) > 1)inteval = map_col / (edgePorts[0] + 1);
	for (int m = 1; m <= edgePorts[0]; ++m)
	{
		outPortSet[count].i = 0;	//0行
		outPortSet[count++].j = m * inteval * 3;//相对真实地图坐标
	}
	//k == 1
	inteval = 1;	//reset this
	if (map_row / (edgePorts[1] + 1) > 1)inteval = map_row / (edgePorts[1] + 1);
	for (int m = 1; m <= edgePorts[1]; ++m)
	{
		outPortSet[count].i = m * inteval * 3;	//0行
		outPortSet[count++].j = (map_col - 1) * 3;//相对真实地图坐标
	}
	//k == 2
	inteval = 1;	//reset this
	if (map_col / (edgePorts[2] + 1) > 1)inteval = map_col / (edgePorts[2] + 1);
	for (int m = 1; m <= edgePorts[2]; ++m)
	{
		outPortSet[count].i = (map_row - 1) * 3;	//最底行
		outPortSet[count++].j = m * inteval * 3;//相对真实地图坐标
	}
	//k == 3
	inteval = 1;	//reset this
	if (map_row / (edgePorts[3] + 1) > 1)inteval = map_row / (edgePorts[3] + 1);
	for (int m = 1; m <= edgePorts[3]; ++m)
	{
		outPortSet[count].i = m * inteval * 3;	//最底行
		outPortSet[count++].j = 0;					//最前0列
	}
}

void VirtualEnvironment::CreateRobot()
{
	int port = 1;
	int port_count = 1;
	int destport = 1;
	for (int k = 0; k < this->numRobot; k++)
	{
        port = k % this->numOutPort;	//应归属的出发端口port ID
        port_count = k / this->numOutPort + 1;//归属的出发端口出发批次，也就是时间槽
		destport = this->Destinations.front();//取得其目的地port ID
		this->Destinations.pop_front();

		robot_list.push_back(Robot(k, outPortSet[port], desSet[destport], port_count));
	}
}

bool VirtualEnvironment::CheackCorrectness()
{
	int real_row = (map_row - 1) * 3 + 1;
	int real_col = (map_col - 1) * 3 + 1;
	//每次统计前清零;
	for (int i = 0; i < real_row; ++i)
		for (int j = 0; j < real_col; ++j)
			this->posMap[i][j].clear();

	vector <Robot> ::iterator rit = robot_list.begin();
	for (; rit != robot_list.end(); ++rit)
	{
		this->posMap[rit->currPos.i][rit->currPos.j].push_back(rit->Rid);
	}
	//统计算法正确性;
	for (int i = 0; i < real_row; ++i)
		for (int j = 0; j < real_col; ++j)
		{
			if (posMap[i][j].size() > 1)//位置统计超过正常容量;
			{
				if (ResEntity_list[i][j].Rtype == 3 ||
					ResEntity_list[i][j].Rtype == 2 ||
					ResEntity_list[i][j].Rtype == 5 ||
					ResEntity_list[i][j].Rtype == 6)
				{
					cout << "** In time slot " << this->time_slot << ", Collision.. ";
					cout << "Position : (" << i << "," << j << ")  type: " << int(ResEntity_list[i][j].Rtype) << endl;
					return false;//并且是路径资源，报错!
				}
			}
		}

	return true;	//默认返回正确;
}
