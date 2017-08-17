// GenerateExpressDestinations.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "RealDest.h"


int main()
{
	int numDestination = 2;	//目的地数目

    for(; numDestination <= 342; ++numDestination)
    {
        RealDest rd;
        rd.GenerateReal(numDestination);
    }








	//下面是生成随机分布快件的
	//ofstream fout("Destinations.txt");
	//int * arr = new int[numDestination];
	//for (int i = 0; i < numDestination; ++i)arr[i] = 0;

	///* Linux下运行
	//struct timespec tp;
	//clock_gettime(CLOCK_THREAD_CPUTIME_ID,&tp);
	//srand(tp.tv_nsec);
	//destport = rand() % this->numDestination;//随机其目的地port ID
	////destport = this->numDestination - it->Rid % this->numDestination - 1;
	//*/

	///* Windows下运行 */
	//int destport = 0;	//暂存当前目的地格口ID
	//LARGE_INTEGER nFrequency;


	//for (int i = 0; i < 100000; ++i)
	//{
	//	if (::QueryPerformanceFrequency(&nFrequency))
	//	{
	//		LARGE_INTEGER nStartCounter;
	//		::QueryPerformanceCounter(&nStartCounter);
	//		::srand((unsigned)nStartCounter.LowPart);
	//		destport = rand() % numDestination;//随机其目的地port ID
	//		arr[destport]++;
	//		fout << destport << endl;
	//	}
	//}
	//for (int i = 0; i < numDestination; ++i)cout<<i<<" : "<<arr[i]<<endl;

	//

	//fout.close();
    return 0;
}

