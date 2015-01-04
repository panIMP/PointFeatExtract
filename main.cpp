#include "imgIO.h"
#include "opencv2/highgui/highgui.hpp"
#include "interestPointLocate.h"
#include "opencv2/nonfree/features2d.hpp"

#include "error.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <string.h>
#include <time.h>
#include "debugMarco.h"


using namespace std;


#ifdef _MAIN_TEST_
int main()
{
	// ============================================================================================================================
	// left image
	cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/juanbidao/src/juanbidao1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/gongjian/1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp1/view0.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/grap/img1.ppm"), cv::IMREAD_COLOR);

	// right image
	cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/juanbidao/yingdongxuanzhuan/juanbidao22.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/gongjian/7.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp1/view0.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/grap/img4.ppm"), cv::IMREAD_COLOR);

	ProjectMat realMat;
	realMat.m1 = 1;
	realMat.m2 = 0;
	realMat.m3 = 0;
	realMat.m4 = 0;
	realMat.m5 = 1;
	realMat.m6 = 0;
	cout << "realMat: " << endl;
	cout << realMat.m1 << "\t" << realMat.m2 << "\t" << realMat.m3 << endl;
	cout << realMat.m4 << "\t" << realMat.m5 << "\t" << realMat.m6 << endl;

#ifdef _MODULATE_DATA_
	double angle1 = 0.0;
	double angle2 = 30.0;
	double angle3 = 60.0;
	double dx = 0;
	double dy = dx;
	double d = sqrt(dx * dx + dy * dy);
	double scale = 1;
	double a1, a2, a3, a4;
	cv::Mat matT(2, 3, CV_64FC1);
	cv::Mat matTUni(2, 3, CV_64FC1);
	cv::Mat matA(2, 2, CV_64FC1);
	cv::Mat matAUni(2, 2, CV_64FC1);
	cv::Mat matU(2, 2, CV_64FC1);
	cv::Mat matUUni(2, 2, CV_64FC1);
	cv::Mat matS(2, 2, CV_64FC1);
	cv::Mat matSUni(2, 2, CV_64FC1);
	cv::Mat matVT(2, 2, CV_64FC1);
	cv::Mat matVTUni(2, 2, CV_64FC1);

	// affine transform
	matRColor = scaleImg(matRColor, scale);
	rotateImg(matRColor, matRColor, angle3);
	matRColor = affineImg(matRColor, angle2);
	rotateImg(matRColor, matRColor, angle1);
	matRColor = moveImg(matRColor, dx, dy);

	cv::imshow("joke", matRColor);
	cv::imwrite("joke.png", matRColor);
#endif

	// ==============================================================================================================================
	cv::Mat_<unsigned char> matL;
	cv::cvtColor(matLColor, matL, CV_BGR2GRAY);
	unsigned char* p_imgL = matL.data;
	unsigned short wL = matL.cols;
	unsigned short hL = matL.rows;
	gaussin(p_imgL, wL, hL);

	cv::Mat_<unsigned char> matR;
	cv::cvtColor(matRColor, matR, CV_BGR2GRAY);
	unsigned char* p_imgR = matR.data;
	unsigned short wR = matR.cols;
	unsigned short hR = matR.rows;
	gaussin(p_imgR, wR, hR);

	// ==============================================================================================================================
	// create the integrate image of the left image
	unsigned int* p_integImgL = createIntegImg(p_imgL, wL, hL);

	// create the det(Hessin) images of different octaves and layers of the left image
	hesMat* p_detHesImgPyrL = (hesMat*)calloc_check(LAYER_NUM * wL * hL, sizeof(hesMat));
	createDetHesImgPyr(p_detHesImgPyrL, p_integImgL, LAYER_NUM, wL, hL);

	// locate the interest points in the pyramid of the left image
	InterestPoint* p_pointsL = (InterestPoint*)calloc_check(wL * hL, sizeof(InterestPoint));
	unsigned char* p_markImgL = (unsigned char*)calloc_check(wL * hL, sizeof(unsigned char));
	unsigned int pointNumL = getPointsLocations(p_pointsL, p_markImgL, p_imgL, p_detHesImgPyrL, LAYER_NUM, 200.0, wL, hL);

	// =============
	// create the integrate image of the right image
	unsigned int* p_integImgR = createIntegImg(p_imgR, wR, hR);

	// create the det(Hessin) images of different octaves and layers of the right image
	hesMat* p_detHesImgPyrR = (hesMat*)calloc_check(LAYER_NUM * wR * hR, sizeof(hesMat));
	createDetHesImgPyr(p_detHesImgPyrR, p_integImgR, LAYER_NUM, wR, hR);

	// locate the interest points in the pyramid of the right image
	InterestPoint* p_pointsR = (InterestPoint*)calloc_check(wR * hR, sizeof(InterestPoint));
	unsigned char* p_markImgR = (unsigned char*)calloc_check(wR * hR, sizeof(unsigned char));
	unsigned int pointNumR = getPointsLocations(p_pointsR, p_markImgR, p_imgR, p_detHesImgPyrR, LAYER_NUM, 200.0, wR, hR);

	// show compare point location result
	drawRect(matLColor, p_pointsL, pointNumL, 15, 2, cv::Scalar(255, 255, 255));
	drawRect(matRColor, p_pointsR, pointNumR, 15, 2, cv::Scalar(255, 255, 255));

#ifndef _ONLY_LOCATION_
	// ==============================================================================================================================
	double dR = (double)(wR * hR) / (double)pointNumR;
	unsigned short r = sqrt(dR);
	double dThresh = dR / 4;
	
	wipeOutBoudaryPixel(p_pointsL, &pointNumL, r, wL, hL);
	cout << "left image interest point number: " << pointNumL << endl;
	getPointsFeats(p_pointsL, pointNumL, p_imgL, r, wL);

	wipeOutBoudaryPixel(p_pointsR, &pointNumR, r, wR, hR);
	cout << "right image interest point number: " << pointNumR << endl;
	getPointsFeats(p_pointsR, pointNumR, p_imgR, r, wR);

	// ==============================================================================================================================
	// match the two images
	unsigned int pairNum = 0;
	PointPair* p_pairs = (PointPair*)calloc_check(max(pointNumL, pointNumR), sizeof(PointPair));
	ProjectMat suitMat = matchInterestPoints(p_pointsL, pointNumL, p_pointsR, pointNumR, p_pairs, &pairNum, dThresh, wL, hL, wR, hR);
	showMatchResult(matLColor, matRColor, realMat, suitMat, p_pointsL, pointNumL, p_pairs, pairNum, dThresh, 10);

	// ===============================================================================================================================
	// free memory
	free(p_integImgL);
	free(p_detHesImgPyrL);
	free(p_markImgL);
	free(p_pointsL);

	free(p_integImgR);
	free(p_detHesImgPyrR);
	free(p_markImgR);
	free(p_pointsR);

	free(p_pairs);

#endif

	getchar();
	cv::waitKey();
	return 0;
}
#endif


#ifdef _DEBUG_OPENCV_
int main()
{
	int i = 0;
	unsigned long long totalTime1 = 0;
	unsigned long long totalTime2 = 0;
	// detecting keypoints
	cv::SurfFeatureDetector detector;
	vector<cv::KeyPoint> keypoints;
	cv::SurfDescriptorExtractor extractor;
	cv::Mat descriptors;
	cv::Mat img;

	for (int i = 1; i <= 27; ++i)
	{
		stringstream str;
		str << i;

		img = cv::imread(("G:/xiangmu/Pictures/gongjian/" + str.str() + ".bmp"), CV_LOAD_IMAGE_GRAYSCALE);

		clock_t start1 = clock();
		detector.detect(img, keypoints);
		clock_t end1 = clock();
		clock_t time1 = end1 - start1;
		cout << "time1: " << time1;

		clock_t start2 = clock();
		extractor.compute(img, keypoints, descriptors);
		clock_t end2 = clock();
		clock_t time2 = end2 - start2;
		cout << "time2: " << time2 << endl;

		totalTime1 += time1;
		totalTime2 += time2;
	}

	cout << "even time1: " << totalTime1 / 27 << endl;
	cout << "even time2: " << totalTime2 / 27 << endl;
	cv::waitKey();
	getchar();

	return 0;
}
#endif


#ifdef _GET_IMG_VAL_
int main()
{
	cv::Mat_<unsigned char> mat1 = cv::imread(string("G:/xiangmu/Pictures/gongjian/1.bmp"), cv::IMREAD_GRAYSCALE);
	//cv::Mat_<unsigned char> mat1;
	//cv::cvtColor(mat1Color, mat1, CV_BGR2GRAY);
	cv::imshow("mat1Src", mat1);
	gaussin(mat1.data, mat1.cols, mat1.rows);
	
	unsigned char thresh = otsu(mat1.data, mat1.cols, mat1.rows);
	cv::Mat mat2(mat1.rows, mat1.cols, CV_8UC1);
	mat1.copyTo(mat2);
	cout << mat2.size() << endl;
	binary(mat2.data, thresh, 255, mat1.cols, mat1.rows);
	cv::imshow("mat1Binary", mat2);
	cv::imwrite("mat1Binary.bmp", mat2);
	cout << mat2.channels() << endl;
	
	cv::Mat mat3(mat1.rows, mat1.cols, CV_8UC1);
	mat2.copyTo(mat3);
	erode(mat2.data, mat3.data, mat3.cols, mat3.rows);
	cv::imshow("mat1Erode", mat3);

	subtract(mat2.data, mat3.data, mat1.cols, mat1.rows);
	cv::imshow("mat1Edge", mat2);

	cv::waitKey();
	//printImageVal("I1E0V0.txt", mat1.data, mat1.cols, mat1.rows, 1);
	//cv::imwrite("G:/xiangmu/Pictures/juanbidao/yingdongxuanzhuan/juanbidaoBinaryEqu.bmp", mat1);

	/*cv::Mat_<cv::Vec3b> mat2Color = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp1/view0.png"), cv::IMREAD_COLOR);
	cv::Mat_<unsigned char> mat2;
	cv::cvtColor(mat2Color, mat2, CV_BGR2GRAY);
	cv::equalizeHist(mat2, mat2);
	printImageVal("I1E1V0.txt", mat2.data, mat2.cols, mat2.rows, 1);
	cv::imwrite("G:/xiangmu/Pictures/Dolls/Illum1/Exp1/view0Equalize.png", mat2);

	cv::Mat_<cv::Vec3b> mat3Color = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp2/view0.png"), cv::IMREAD_COLOR);
	cv::Mat_<unsigned char> mat3;
	cv::cvtColor(mat3Color, mat3, CV_BGR2GRAY);
	printImageVal("I1E2V0.txt", mat3.data, mat3.cols, mat3.rows, 1);
	cv::imwrite("G:/xiangmu/Pictures/Dolls/Illum1/Exp2/view0Eqalize.png", mat3);

	cv::Mat_<cv::Vec3b> mat4Color = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum2/Exp1/view0.png"), cv::IMREAD_COLOR);
	cv::Mat_<unsigned char> mat4;
	cv::cvtColor(mat4Color, mat4, CV_BGR2GRAY);
	printImageVal("I2E1V0.txt", mat4.data, mat4.cols, mat4.rows, 1);
	cv::imwrite("G:/xiangmu/Pictures/Dolls/Illum2/Exp1/view0Equalize.png", mat4);*/

	return 0;
}
#endif
