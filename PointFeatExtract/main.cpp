#include "imgIO.h"
#include "opencv2/highgui/highgui.hpp"
#include "interestPointLocate.h"
#include "opencv2/nonfree/features2d.hpp"

#include "Error\error.h"
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
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/juanbidao/src/juanbidao1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/gongjian/1.bmp"), cv::IMREAD_COLOR);
	cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp1/view0.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/grap/img1.ppm"), cv::IMREAD_COLOR);

	// right image
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/juanbidao/yingdongxuanzhuan/juanbidao10.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/gongjian/7.bmp"), cv::IMREAD_COLOR);
	cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp1/view6.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/grap/img3.ppm"), cv::IMREAD_COLOR);

	ProjectMat realMat;
	realMat.m1 = 1;
	realMat.m2 = 0;
	realMat.m3 = 0;
	realMat.m4 = 0;
	realMat.m5 = 1;
	realMat.m6 = 0;

#ifdef _MODULATE_DATA_
	double angle1 = 0.0;
	double angle2 = 30.0;
	double angle3 = 60.0;
	double dx = 0;
	double dy = dx;
	double scale = 1;

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

	// ==============================================================================================================================
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
	drawRect(matLColor, p_pointsL, pointNumL, 10, 2, cv::Scalar(255, 255, 255));
	drawRect(matRColor, p_pointsR, pointNumR, 10, 2, cv::Scalar(255, 255, 255));

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

	cv::waitKey();
	return 0;
}
#endif

