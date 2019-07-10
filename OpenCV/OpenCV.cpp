// OpenCV.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "OpenCV.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// The one and only application object

CWinApp theApp;

using namespace std;


#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;






#include "ImageSurf.h"
#include "Imagejoint.h"


void ShowImage(std::string FileName)
{
	// ����һ��ͼƬ��С��è��
	Mat img = imread(FileName);
	// ����һ����Ϊ "С��è"����
	namedWindow("С��è");
	// �ڴ�������ʾС��è
	imshow("С��è", img);
	// �ȴ�6000 ms�󴰿��Զ��ر�
	waitKey(6000);
}


int main()
{
    int nRetCode = 0;

    HMODULE hModule = ::GetModuleHandle(nullptr);

    if (hModule != nullptr)
    {
        // initialize MFC and print and error on failure
        if (!AfxWinInit(hModule, nullptr, ::GetCommandLine(), 0))
        {
            // TODO: change error code to suit your needs
            wprintf(L"Fatal Error: MFC initialization failed\n");
            nRetCode = 1;
        }
        else
        {
            // TODO: code your application's behavior here.
        }
    }
    else
    {
        // TODO: change error code to suit your needs
        wprintf(L"Fatal Error: GetModuleHandle failed\n");
        nRetCode = 1;
    }

	CImagejoint ij;
	vector<string> FNs;
	FNs.clear();
	for (int i = 0; i < 5; i++)
	{
		FNs.push_back("E:\\0.DEL\\1\\img_0_"+std::to_string(i+7)+"_100.bmp");
	}

	ij.JointH(FNs);

	CImageSurf is;
	is.Surf("E:\\0.DEL\\2\\img_0_7_100.bmp", "E:\\0.DEL\\2\\img_0_8_100.bmp");
	//is.Surf3("E:\\0.DEL\\2\\2.bmp", "E:\\0.DEL\\2\\1.bmp");

	//ShowImage("E:\\0.DEL\\1\\img_Merge_5.bmp");

    return nRetCode;
}
