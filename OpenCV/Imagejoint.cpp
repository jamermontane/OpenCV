#include "stdafx.h"
#include "Imagejoint.h"


CImagejoint::CImagejoint()
{
}


CImagejoint::~CImagejoint()
{
}


CImagejoint::CImagejoint(int Cols, int Rows)
{
	InitMatrix(Cols, Rows);
}

void CImagejoint::InitMatrix(int Cols, int Rows)
{

}


void CImagejoint::JointH(vector<string> FNs)
{
	Mat combine, combine1, combine2;


	vector<string>::iterator FN = FNs.begin();
	Mat a = imread(*FN);
	FN++;
	while (FN != FNs.end())
	{
		Mat b = imread(*FN);
		hconcat(a, b,combine);
		a = combine;
		FN++;
	}

	imwrite( "joint.bmp",combine);
	//imshow("Combine", combine);
	waitKey(0);
}

