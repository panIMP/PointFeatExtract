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
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/jiong/pos/1.bmp"), cv::IMREAD_COLOR);
	cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/juanbidao/pos/juanbidao4.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/ykq/src/1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/gongjian/1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp0/view0.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/grap/img1.ppm"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/sift/box.pgm"), cv::IMREAD_COLOR);
	if (matLColor.empty())
		return -1;

	// right image
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/jiong/pos/3.bmp"), cv::IMREAD_COLOR);
	cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/juanbidao/qingxie/juanbidao9.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/ykq/pos/2.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/gongjian/7.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum3/Exp2/view0.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/grap/img3.ppm"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/sift/scene.pgm"), cv::IMREAD_COLOR);
	if (matRColor.empty())
		return -1;

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
	unsigned int areaL = wL * hL;

	cv::Mat_<unsigned char> matR;
	cv::cvtColor(matRColor, matR, CV_BGR2GRAY);
	unsigned char* p_imgR = matR.data;
	unsigned short wR = matR.cols;
	unsigned short hR = matR.rows;
	unsigned int areaR = wR * hR;

	cv::Mat_<unsigned char> t_matL(hL, wL);
	cv::Mat_<unsigned char> t_matR(hR, wR);
	memset(t_matL.data, SHOULDMARK, wL * hL);
	memset(t_matR.data, SHOULDMARK, wR * hR);

	cv::imshow("matL", matL);
	cv::imshow("matR", matR);
	
#ifdef _PRE_PROCESS_
	areaL = preProcess(0, matL.data, t_matL.data, SHOULDMARK, wL, hL);
	cv::imshow("t_matL", t_matL);

	areaR = preProcess(0, matR.data, t_matR.data, SHOULDMARK, wR, hR);
	cv::imshow("t_matR", t_matR);
#endif

	// ==============================================================================================================================
	// create the integrate image of the left image
	unsigned int* p_integImgL = createIntegImg(p_imgL, wL, hL);

	// create the det(Hessin) images of different octaves and layers of the left image
	hesMat* p_detHesImgPyrL = (hesMat*)calloc_check(wL * hL, sizeof(hesMat));
	createDetHesImgPyr(p_detHesImgPyrL, p_integImgL, t_matL.data, LAYER_NUM, wL, hL);

	// locate the interest points in the pyramid of the left image
	InterestPoint* p_pointsL = (InterestPoint*)calloc_check(wL * hL, sizeof(InterestPoint));
	unsigned int pointNumL = getPointsLocations(p_pointsL, t_matL.data, p_imgL, p_detHesImgPyrL, LAYER_NUM, 400.0, wL, hL);

	// ==============================================================================================================================
	// create the integrate image of the right image
	unsigned int* p_integImgR = createIntegImg(p_imgR, wR, hR);

	// create the det(Hessin) images of different octaves and layers of the right image
	hesMat* p_detHesImgPyrR = (hesMat*)calloc_check(wR * hR, sizeof(hesMat));
	createDetHesImgPyr(p_detHesImgPyrR, p_integImgR, t_matR.data, LAYER_NUM, wR, hR);

	// locate the interest points in the pyramid of the right image
	InterestPoint* p_pointsR = (InterestPoint*)calloc_check(wR * hR, sizeof(InterestPoint));
	unsigned int pointNumR = getPointsLocations(p_pointsR, t_matR.data, p_imgR, p_detHesImgPyrR, LAYER_NUM, 400.0, wR, hR);

	// ==============================================================================================================================
	double dR = (double)(areaR) / (double)pointNumR;
	unsigned short r = sqrt(dR) / 2;
	r = 12;
	double dThresh = r * r / 4;

	cout << "left image interest point number: " << pointNumL << endl;
	getPointsFeats(p_pointsL, pointNumL, p_imgL, r, wL);

	cout << "right image interest point number: " << pointNumR << endl;
	getPointsFeats(p_pointsR, pointNumR, p_imgR, r, wR);

	// show compare point location result
	cv::cvtColor(matL, matLColor, cv::COLOR_GRAY2BGR);
	cv::cvtColor(matR, matRColor, cv::COLOR_GRAY2BGR);
	drawRect(matLColor, p_pointsL, pointNumL, 1, cv::Scalar(255, 255, 255));
	drawRect(matRColor, p_pointsR, pointNumR, 1, cv::Scalar(255, 255, 255));

	// ==============================================================================================================================
	// match the two images
	unsigned int pairNum = 0;
	PointPair* p_pairs = (PointPair*)calloc_check(max(pointNumL, pointNumR), sizeof(PointPair));
	ProjectMat suitMat = matchInterestPoints(p_pointsL, pointNumL, p_pointsR, pointNumR, p_pairs, &pairNum, dThresh, wL, hL, wR, hR);
	showMatchResult(matLColor, matRColor, realMat, suitMat, p_pointsL, pointNumL, p_pairs, pairNum, dThresh, 7);

	// ===============================================================================================================================
	// free memory
	free(p_integImgL);
	free(p_detHesImgPyrL);
	free(p_pointsL);

	free(p_integImgR);
	free(p_detHesImgPyrR);
	free(p_pointsR);

	free(p_pairs);

	cv::waitKey();
	return 0;
}
#endif



#ifdef _BINARY_
int main()
{
	// left image
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/jiong/pos/1.bmp"), cv::IMREAD_COLOR);
	cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/juanbidao/guangzhao/juanbidao5.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/gongjian/1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/ykq/src/1.bmp"), cv::IMREAD_COLOR);

	cv::Mat_<unsigned char> matL;
	cv::cvtColor(matLColor, matL, CV_BGR2GRAY);
	unsigned char* p_imgL = matL.data;
	unsigned short w = matL.cols;
	unsigned short h = matL.rows;
	cv::Mat_<unsigned char> mat0(h, w);
	cv::Mat_<unsigned char> mat1(h, w);
	cv::Mat_<unsigned char> mat2(h, w);
	cv::Mat_<unsigned char> mat3(h, w);
	cv::Mat_<unsigned char> mat4(h, w);

	gaussin(p_imgL, w, h);

	matL.copyTo(mat0);
	matL.copyTo(mat1);
	matL.copyTo(mat2);
	matL.copyTo(mat3);
	matL.copyTo(mat4);

	cv::imshow("src", matL);
	cv::imwrite("C:/src.png", matL);

	cv::threshold(mat0, mat0, 0, 255, cv::THRESH_OTSU);
	cv::imshow("opencv global otsu", mat0);
	cv::imwrite("C:/opencv_global_otsu.png", mat0);

	otsuBinary(mat1.data, 255, w, h);
	cv::imshow("global otsu", mat1);
	cv::imwrite("C:/global_otsu.png", mat1);

	localOtsuBinary(mat2.data, 255, w, h, 4);
	cv::imshow("local otsu", mat2);
	cv::imwrite("C:/local_otsu.png", mat2);

	localOtsuRecurBinary(mat3.data, 255, w, h, 4);
	cv::imshow("local recur otsu", mat3);
	cv::imwrite("C:/local_recur_otsu.png", mat3);

	binary(mat4.data, threshByFirstVally(mat4.data, w, h), 255, w, h);
	cv::imshow("thresh by triangular", mat4);
	cv::imwrite("thresh_by_triangular.png", mat4);
	
	setBoundaryZero(mat4.data, w, h);
	cv::Mat_<unsigned char> mat4Edge(w, h);
	mat4.copyTo(mat4Edge);
	elate(mat4Edge.data, mat4.data, w, h);
	subtract(mat4.data, mat4Edge.data, w, h);
	cv::imshow("edge", mat4);
	cv::imwrite("C:/edge.png", mat4);

	Contour contours;
	contours.p_coords = (Coord*)malloc_check(mat4.cols * mat4.rows * sizeof(Coord));
	int contourNum = markOutContour(mat4.data, &contours, mat4.cols, mat4.rows);
	markMaxOutContour(mat4.data, &contours, contourNum, mat4.cols, mat4.rows);
	cv::imshow("biggestEdge", mat4);
	cv::imwrite("C:/biggestEdge.png", mat4);

	fillRegion(mat4.data, w, h, 255);
	//cv::floodFill(mat4, cv::Point(347, 118), 255);
	cv::imshow("fill", mat4);
	cv::imwrite("C:/fill.png", mat4);

	equHist(matL.data, mat4.data, w, h);
	cv::imshow("equalized", matL);
	cv::imwrite("C:/equalized1.png", matL);

	cv::waitKey(0);
	return 0;
}

#endif

