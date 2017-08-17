#include "stdafx.h"
#include "RealDest.h"


RealDest::RealDest()
{
}


RealDest::~RealDest()
{
}

//random shuffle function
void random(int a[], int n)
{
    int index, tmp, i;
    struct timespec tp;

    clock_gettime(CLOCK_THREAD_CPUTIME_ID,&tp);
    srand(tp.tv_nsec);


    for(i=n-1; i>0; i--)
    {
        index=rand()%i;
        tmp=a[i];
        a[i]=a[index];
        a[index]=tmp;
    }
}


void RealDest::GenerateReal(int numDestination)
{
	double * cityFre;
	cityFre = new double[numDestination];
	for (int i = 0; i < numDestination; ++i)cityFre[i] = 0;
	int rank;	//����ID
	double cf;	//����Ƶ��
	double maxcf =0.0;

    string wfname = "DestinationsReal_" + to_string(numDestination);
    wfname += ".txt";

	ifstream fin("RealDesti_342.txt");
	ofstream fout(wfname);
	string str1;
	//fin >> str1 >> str1;
	fin >> rank >> maxcf;
	cityFre[rank] = maxcf;
	int i = 1;
	while (i++ < numDestination && fin >> rank >> cf)
        cityFre[rank] = cf;

    cout << "\nreal city fre ..\n*******************************\n";
	for (int i = 0; i < numDestination; ++i)cout << i<<" : "<<cityFre[i] << endl;


	int * arrCF = new int[numDestination];
	for (int i = 0; i < numDestination; ++i)arrCF[i] = 0;

	/* Linux������*/
	struct timespec tp;
    double destport = 0.0;

	//destport = this->numDestination - it->Rid % this->numDestination - 1;
    for (int i = 0; i < 249650; ++i)
    {
        clock_gettime(CLOCK_THREAD_CPUTIME_ID,&tp);
        srand(tp.tv_nsec);
        //destport = rand() % this->numDestination;//�����Ŀ�ĵ�port ID
        destport = maxcf * ( double(rand()) / (RAND_MAX) );//������Ǹ��ٷֱ���ֵ
        //cout << destport << endl;
        //Sleep(1);
        int shuffle_num = 0;
        int shuffleArr[500] = {0};
        for (int i = 0; i < numDestination; ++i)
        {
            if (cityFre[i] >= destport)
            {
                //fout << i << endl;	//�洢��ID
                arrCF[i]++;			    //ͳ��Ƶ��
                shuffleArr[shuffle_num++] = i;
            }
        }

        random(shuffleArr, shuffle_num);
        for (int i = 0; i < shuffle_num; ++i)
            fout << shuffleArr[i] << endl;	//�洢��ID

    }


	/* Windows������
	double destport = 0.0;	//�ݴ浱ǰĿ�ĵظ��IDƵ��ֵ
	LARGE_INTEGER nFrequency;
	for (int i = 0; i < 185538; ++i)
	{
		if (::QueryPerformanceFrequency(&nFrequency))
		{
			LARGE_INTEGER nStartCounter;
			::QueryPerformanceCounter(&nStartCounter);
			::srand((unsigned)nStartCounter.LowPart);
			destport = maxcf * ( double(rand()) / (RAND_MAX) );//������Ǹ��ٷֱ���ֵ
			//cout << destport << endl;
			Sleep(1);
			for (int i = 0; i < numDestination; ++i)
			{
				if (cityFre[i] >= destport)
				{
					fout << i << endl;	//�洢��ID
					arrCF[i]++;			//ͳ��Ƶ��
				}
			}
		}
	}
	*/
    cout << "\ngenerate city fre ..\n*******************************\n";
	for (int i = 0; i < numDestination; ++i)cout << i << " : " << arrCF[i] << endl;


	fin.close();
	fout.close();
}
