// GenerateExpressDestinations.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "RealDest.h"


int main()
{
	int numDestination = 2;	//Ŀ�ĵ���Ŀ

    for(; numDestination <= 342; ++numDestination)
    {
        RealDest rd;
        rd.GenerateReal(numDestination);
    }








	//��������������ֲ������
	//ofstream fout("Destinations.txt");
	//int * arr = new int[numDestination];
	//for (int i = 0; i < numDestination; ++i)arr[i] = 0;

	///* Linux������
	//struct timespec tp;
	//clock_gettime(CLOCK_THREAD_CPUTIME_ID,&tp);
	//srand(tp.tv_nsec);
	//destport = rand() % this->numDestination;//�����Ŀ�ĵ�port ID
	////destport = this->numDestination - it->Rid % this->numDestination - 1;
	//*/

	///* Windows������ */
	//int destport = 0;	//�ݴ浱ǰĿ�ĵظ��ID
	//LARGE_INTEGER nFrequency;


	//for (int i = 0; i < 100000; ++i)
	//{
	//	if (::QueryPerformanceFrequency(&nFrequency))
	//	{
	//		LARGE_INTEGER nStartCounter;
	//		::QueryPerformanceCounter(&nStartCounter);
	//		::srand((unsigned)nStartCounter.LowPart);
	//		destport = rand() % numDestination;//�����Ŀ�ĵ�port ID
	//		arr[destport]++;
	//		fout << destport << endl;
	//	}
	//}
	//for (int i = 0; i < numDestination; ++i)cout<<i<<" : "<<arr[i]<<endl;

	//

	//fout.close();
    return 0;
}

