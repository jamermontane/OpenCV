#include "stdafx.h"
#include "ImageSurf.h"


CImageSurf::CImageSurf()
{
}


CImageSurf::~CImageSurf()
{
}


void CImageSurf::CalcCorners(const Mat& H, const Mat& src)
{
	double v2[] = { 0, 0, 1 };//左上角
	double v1[3];//变换后的坐标值
	Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

	V1 = H * V2;
	//左上角(0,0,1)
	cout << "V2: " << V2 << endl;
	cout << "V1: " << V1 << endl;
	_corners.left_top.x = v1[0] / v1[2];
	_corners.left_top.y = v1[1] / v1[2];

	//左下角(0,src.rows,1)
	v2[0] = 0;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	_corners.left_bottom.x = v1[0] / v1[2];
	_corners.left_bottom.y = v1[1] / v1[2];

	//右上角(src.cols,0,1)
	v2[0] = src.cols;
	v2[1] = 0;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	_corners.right_top.x = v1[0] / v1[2];
	_corners.right_top.y = v1[1] / v1[2];

	//右下角(src.cols,src.rows,1)
	v2[0] = src.cols;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	_corners.right_bottom.x = v1[0] / v1[2];
	_corners.right_bottom.y = v1[1] / v1[2];

}



void CImageSurf::OptimizeSeam(Mat& img1, Mat& trans, Mat& dst)
{
	int start = MIN(_corners.left_top.x, _corners.left_bottom.x);//开始位置，即重叠区域的左边界  

	double processWidth = img1.cols - start;//重叠区域的宽度  
	int rows = dst.rows;
	int cols = img1.cols; //注意，是列数*通道数
	double alpha = 1;//img1中像素的权重  

	for (int i = 0; i < rows; i++)
	{
		uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
		uchar* t = trans.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);
		for (int j = 0;/*start;*/ j < cols; j++)
		{
			//如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 1;
			}
			else
			{
				//img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好  
				alpha = (processWidth - (j - start)) / processWidth;
			}

			d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
			d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
			d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

		}
	}

}

void CImageSurf::Surf(std::string FN1, std::string FN2)
{
	Mat image01 = imread(FN1, 1);    //右图
	Mat image02 = imread(FN2, 1);    //左图
	imshow("p2", image01);
	imshow("p1", image02);
	// 
	   //灰度图转换  
	Mat image1, image2;
	cvtColor(image01, image1, CV_RGB2GRAY);
	cvtColor(image02, image2, CV_RGB2GRAY);
	// 
	// 
	   //提取特征点    
	SurfFeatureDetector Detector(2000);
	vector<KeyPoint> keyPoint1, keyPoint2;
	Detector.detect(image1, keyPoint1);
	Detector.detect(image2, keyPoint2);
	// // 
	   //特征点描述，为下边的特征点匹配做准备    
	SurfDescriptorExtractor Descriptor;
	Mat imageDesc1, imageDesc2;
	Descriptor.compute(image1, keyPoint1, imageDesc1);
	Descriptor.compute(image2, keyPoint2, imageDesc2);

	FlannBasedMatcher matcher;
	vector<vector<DMatch> > matchePoints;
	vector<DMatch> GoodMatchePoints;

	vector<Mat> train_desc(1, imageDesc1);
	matcher.add(train_desc);
	matcher.train();

	matcher.knnMatch(imageDesc2, matchePoints, 2);
	cout << "total match points: " << matchePoints.size() << endl;

	// Lowe's algorithm,获取优秀匹配点
	for (int i = 0; i < matchePoints.size(); i++)
	{
		if (matchePoints[i][0].distance < 0.4 * matchePoints[i][1].distance)
		{
			GoodMatchePoints.push_back(matchePoints[i][0]);
		}
	}

	Mat first_match;
	drawMatches(image02, keyPoint2, image01, keyPoint1, GoodMatchePoints, first_match);
	imshow("first_match ", first_match);

	vector<Point2f> imagePoints1, imagePoints2;

	for (int i = 0; i < GoodMatchePoints.size(); i++)
	{
		imagePoints2.push_back(keyPoint2[GoodMatchePoints[i].queryIdx].pt);
		imagePoints1.push_back(keyPoint1[GoodMatchePoints[i].trainIdx].pt);
	}



	//获取图像1到图像2的投影映射矩阵 尺寸为3*3  
	Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
	////也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差  
	//Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);  
	cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵      

												//计算配准图的四个顶点坐标
	CalcCorners(homo, image01);
	cout << "left_top:" << _corners.left_top << endl;
	cout << "left_bottom:" << _corners.left_bottom << endl;
	cout << "right_top:" << _corners.right_top << endl;
	cout << "right_bottom:" << _corners.right_bottom << endl;

	//图像配准  
	Mat imageTransform1, imageTransform2;
	warpPerspective(image01, imageTransform1, homo, Size(MAX(_corners.right_top.x, _corners.right_bottom.x), image02.rows));
	//warpPerspective(image01, imageTransform2, adjustMat*homo, Size(image02.cols*1.3, image02.rows*1.8));
	imshow("直接经过透视矩阵变换", imageTransform1);
	imwrite("trans1.jpg", imageTransform1);


	//创建拼接后的图,需提前计算图的大小
	int dst_width = /*image01.cols + image02.cols;*/imageTransform1.cols;  //取最右点的长度为拼接图的长度
	int dst_height = image02.rows;

	Mat dst(dst_height, dst_width, CV_8UC3);
	dst.setTo(0);

	imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	image02.copyTo(dst(Rect(0, 0, image02.cols, image02.rows)));

	imshow("b_dst", dst);


	OptimizeSeam(image02, imageTransform1, dst);


	imshow("dst", dst);
	imwrite("dst.jpg", dst);

	waitKey();

}


float fDistance(Point2f p1, Point2f p2)
{
	float ftmp = (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
	ftmp = sqrt((float)ftmp);
	return ftmp;
}

void CImageSurf::Surf2(std::string FN1, std::string FN2)
{
	Mat img_1 = imread(FN1, 0);
	Mat img_2 = imread(FN2, 0);
	////添加于连铸图像
	//img_1 = img_1(Rect(20,0,img_1.cols-40,img_1.rows));
	//img_2 = img_2(Rect(20,0,img_1.cols-40,img_1.rows));
	//    cv::Canny(img_1,img_1,100,200);
	//    cv::Canny(img_2,img_2,100,200);
	if (!img_1.data || !img_2.data)
	{
		std::cout << " --(!) Error reading images " << std::endl; return;
	}
	//-- Step 1: 使用SURF识别出特征点
	int minHessian = 400;
	SurfFeatureDetector detector(minHessian);
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	detector.detect(img_1, keypoints_1);
	detector.detect(img_2, keypoints_2);
	//-- Step 2: 描述SURF特征
	SurfDescriptorExtractor extractor;
	Mat descriptors_1, descriptors_2;
	extractor.compute(img_1, keypoints_1, descriptors_1);
	extractor.compute(img_2, keypoints_2, descriptors_2);
	//-- Step 3: 匹配
	FlannBasedMatcher matcher;//BFMatcher为强制匹配
	std::vector< DMatch > matches;
	matcher.match(descriptors_1, descriptors_2, matches);
	//取最大最小距离
	double max_dist = 0; double min_dist = 100;
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	std::vector< DMatch > good_matches;
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (matches[i].distance <= 3 * min_dist)//这里的阈值选择了3倍的min_dist
		{
			good_matches.push_back(matches[i]);
		}
	}
	//画出"good match"
	Mat img_matches;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//-- Localize the object from img_1 in img_2
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;
	for (int i = 0; i < (int)good_matches.size(); i++)
	{
		obj.push_back(keypoints_1[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_2[good_matches[i].trainIdx].pt);
	}
	//直接调用ransac,计算单应矩阵
	Mat H = findHomography(obj, scene, CV_RANSAC);
	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = Point(0, 0);
	obj_corners[1] = Point(img_1.cols, 0);
	obj_corners[2] = Point(img_1.cols, img_1.rows);
	obj_corners[3] = Point(0, img_1.rows);
	std::vector<Point2f> scene_corners(4);
	perspectiveTransform(obj_corners, scene_corners, H);
	//计算内点外点
	std::vector<Point2f> scene_test(obj.size());
	perspectiveTransform(obj, scene_test, H);
	for (int i = 0; i < scene_test.size(); i++)
	{
		printf("%d is %f \n", i + 1, fDistance(scene[i], scene_test[i]));
	}

	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	Point2f offset((float)img_1.cols, 0);
	line(img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar(0, 255, 0), 4);
	//-- Show detected matches
	imshow("Good Matches & Object detection", img_matches);
	waitKey(0);
}


void CImageSurf::Surf3(std::string FN1, std::string FN2)
{
	
	Mat img_1;
	Mat img_2;
	Mat img_raw_1 = imread(FN1);
	Mat img_raw_2 = imread(FN2);
	cvtColor(img_raw_1, img_1, CV_BGR2GRAY);
	cvtColor(img_raw_2, img_2, CV_BGR2GRAY);
	//-- Step 1: 使用SURF识别出特征点
	int minHessian = 400;
	SurfFeatureDetector detector(minHessian);
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	detector.detect(img_1, keypoints_1);
	detector.detect(img_2, keypoints_2);
	//-- Step 2: 描述SURF特征
	SurfDescriptorExtractor extractor;
	Mat descriptors_1, descriptors_2;
	extractor.compute(img_1, keypoints_1, descriptors_1);
	extractor.compute(img_2, keypoints_2, descriptors_2);
	//-- Step 3: 匹配
	FlannBasedMatcher matcher;//BFMatcher为强制匹配
	std::vector< DMatch > matches;
	matcher.match(descriptors_1, descriptors_2, matches);
	//取最大最小距离
	double max_dist = 0; double min_dist = 100;
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	std::vector< DMatch > good_matches;
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (matches[i].distance <= 3 * min_dist)//这里的阈值选择了3倍的min_dist
		{
			good_matches.push_back(matches[i]);
		}
	}
	//-- Localize the object from img_1 in img_2
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;
	for (int i = 0; i < (int)good_matches.size(); i++)
	{
		//这里采用“帧向拼接图像中添加的方法”，因此左边的是scene,右边的是obj
		scene.push_back(keypoints_1[good_matches[i].queryIdx].pt);
		obj.push_back(keypoints_2[good_matches[i].trainIdx].pt);
	}
	//直接调用ransac,计算单应矩阵
	Mat H = findHomography(obj, scene, CV_RANSAC);
	//图像对准
	Mat result;
	Mat resultback; //保存的是新帧经过单应矩阵变换以后的图像
	warpPerspective(img_raw_2, result, H, Size(2 * img_2.cols, img_2.rows));
	result.copyTo(resultback);
	Mat half(result, cv::Rect(0, 0, img_2.cols, img_2.rows));
	img_raw_1.copyTo(half);
	imshow("ajust", result);
	//渐入渐出融合
	Mat result_linerblend = result.clone();
	double dblend = 0.0;
	int ioffset = img_2.cols - 100;
	for (int i = 0; i < 100; i++)
	{
		result_linerblend.col(ioffset + i) = result.col(ioffset + i)*(1 - dblend) + resultback.col(ioffset + i)*dblend;
		dblend = dblend + 0.01;
	}
	imshow("result_linerblend", result_linerblend);
	//最大值法融合
	Mat result_maxvalue = result.clone();
	for (int i = 0; i < img_2.rows; i++)
	{
		for (int j = 0; j < 100; j++)
		{
			int iresult = result.at<Vec3b>(i, ioffset + j)[0] + result.at<Vec3b>(i, ioffset + j)[1] + result.at<Vec3b>(i, ioffset + j)[2];
			int iresultback = resultback.at<Vec3b>(i, ioffset + j)[0] + resultback.at<Vec3b>(i, ioffset + j)[1] + resultback.at<Vec3b>(i, ioffset + j)[2];
			if (iresultback > iresult)
			{
				result_maxvalue.at<Vec3b>(i, ioffset + j) = resultback.at<Vec3b>(i, ioffset + j);
			}
		}
	}
	imshow("result_maxvalue", result_maxvalue);
	//带阈值的加权平滑处理
	Mat result_advance = result.clone();
	for (int i = 0; i < img_2.rows; i++)
	{
		for (int j = 0; j < 33; j++)
		{
			int iimg1 = result.at<Vec3b>(i, ioffset + j)[0] + result.at<Vec3b>(i, ioffset + j)[1] + result.at<Vec3b>(i, ioffset + j)[2];
			//int iimg2= resultback.at<Vec3b>(i,ioffset+j)[0]+ resultback.at<Vec3b>(i,ioffset+j)[1]+ resultback.at<Vec3b>(i,ioffset+j)[2];
			int ilinerblend = result_linerblend.at<Vec3b>(i, ioffset + j)[0] + result_linerblend.at<Vec3b>(i, ioffset + j)[1] + result_linerblend.at<Vec3b>(i, ioffset + j)[2];
			if (abs(iimg1 - ilinerblend) < 3)
			{
				result_advance.at<Vec3b>(i, ioffset + j) = result_linerblend.at<Vec3b>(i, ioffset + j);
			}
		}
	}
	for (int i = 0; i < img_2.rows; i++)
	{
		for (int j = 33; j < 66; j++)
		{
			int iimg1 = result.at<Vec3b>(i, ioffset + j)[0] + result.at<Vec3b>(i, ioffset + j)[1] + result.at<Vec3b>(i, ioffset + j)[2];
			int iimg2 = resultback.at<Vec3b>(i, ioffset + j)[0] + resultback.at<Vec3b>(i, ioffset + j)[1] + resultback.at<Vec3b>(i, ioffset + j)[2];
			int ilinerblend = result_linerblend.at<Vec3b>(i, ioffset + j)[0] + result_linerblend.at<Vec3b>(i, ioffset + j)[1] + result_linerblend.at<Vec3b>(i, ioffset + j)[2];
			if (abs(max(iimg1, iimg2) - ilinerblend) < 3)
			{
				result_advance.at<Vec3b>(i, ioffset + j) = result_linerblend.at<Vec3b>(i, ioffset + j);
			}
			else if (iimg2 > iimg1)
			{
				result_advance.at<Vec3b>(i, ioffset + j) = resultback.at<Vec3b>(i, ioffset + j);
			}
		}
	}
	for (int i = 0; i < img_2.rows; i++)
	{
		for (int j = 66; j < 100; j++)
		{
			//int iimg1= result.at<Vec3b>(i,ioffset+j)[0]+ result.at<Vec3b>(i,ioffset+j)[1]+ result.at<Vec3b>(i,ioffset+j)[2];
			int iimg2 = resultback.at<Vec3b>(i, ioffset + j)[0] + resultback.at<Vec3b>(i, ioffset + j)[1] + resultback.at<Vec3b>(i, ioffset + j)[2];
			int ilinerblend = result_linerblend.at<Vec3b>(i, ioffset + j)[0] + result_linerblend.at<Vec3b>(i, ioffset + j)[1] + result_linerblend.at<Vec3b>(i, ioffset + j)[2];
			if (abs(iimg2 - ilinerblend) < 3)
			{
				result_advance.at<Vec3b>(i, ioffset + j) = result_linerblend.at<Vec3b>(i, ioffset + j);
			}
			else
			{
				result_advance.at<Vec3b>(i, ioffset + j) = resultback.at<Vec3b>(i, ioffset + j);
			}
		}
	}
	imshow("result_advance", result_advance);
	waitKey(0);
}