#pragma once


#include <iostream>
#include <core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

#include <string>
#include <vector>

class CImagejoint
{
private:

	vector<vector<string>> _ImageMatrix;

private:
	void InitMatrix(int Cols, int Rows);

public:
	CImagejoint();
	CImagejoint(int Cols, int Rows);
	~CImagejoint();
	
public:
	void JointH(vector<string> FNs);

};

