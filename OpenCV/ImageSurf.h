#pragma once


#include "highgui/highgui.hpp"    
#include "opencv2/nonfree/nonfree.hpp"    
#include "opencv2/legacy/legacy.hpp"   
#include <iostream>  

using namespace cv;
using namespace std;




typedef struct
{
	Point2f left_top;
	Point2f left_bottom;
	Point2f right_top;
	Point2f right_bottom;
}four_corners_t;


class CImageSurf
{

private:
	four_corners_t _corners;

private:
	void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst);
	void CalcCorners(const Mat& H, const Mat& src);

	

public:
	CImageSurf();
	~CImageSurf();

public:
	void Surf(std::string FN1, std::string FN2);

	void Surf2(std::string FN1, std::string FN2);

	void Surf3(std::string FN1, std::string FN2);

};

