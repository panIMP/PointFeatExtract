#include "imgIO.h"
#include "opencv2/highgui/highgui.hpp"
#include "interestPointLocate.h"
#include "opencv2/nonfree/features2d.hpp"

#include <error.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <string.h>
#include <time.h>

using namespace std;

/*int main()
{
    cv::Mat_<uchar> mat0 = cv::imread(string("G:/xiangmu/Dolls/Illum1/Exp1/view0.png"), cv::IMREAD_GRAYSCALE);
    cv::Mat_<uchar> mat1 = cv::imread(string("G:/xiangmu/Dolls/Illum1/Exp1/view1.png"), cv::IMREAD_GRAYSCALE);
    cv::Mat_<uchar> mat2 = cv::imread(string("G:/xiangmu/Dolls/Illum1/Exp1/view2.png"), cv::IMREAD_GRAYSCALE);
    cv::Mat_<uchar> mat3 = cv::imread(string("G:/xiangmu/Dolls/Illum1/Exp1/view3.png"), cv::IMREAD_GRAYSCALE);
    cv::Mat_<uchar> mat4 = cv::imread(string("G:/xiangmu/Dolls/Illum1/Exp1/view4.png"), cv::IMREAD_GRAYSCALE);
    cv::Mat_<uchar> mat5 = cv::imread(string("G:/xiangmu/Dolls/Illum1/Exp1/view5.png"), cv::IMREAD_GRAYSCALE);
    cv::Mat_<uchar> mat6 = cv::imread(string("G:/xiangmu/Dolls/Illum1/Exp1/view5.png"), cv::IMREAD_GRAYSCALE);

    cv::Mat_<uchar> mats[7] = {mat0, mat1, mat2, mat3, m}

    mergeMats()

    return 0;
}*/

/*int main()
{
    cv::Mat img2 = cv::imread("G:/xiangmu/Dolls/Illum1/Exp1/view1.png", CV_LOAD_IMAGE_GRAYSCALE);
    rotateImg(img2, img2, 30, 1);
    cv::imshow("1", img2);
    cv::waitKey(0);
    return 0;
}*/

/*
int main()
{
    cv::Mat img1 = cv::imread("G:/xiangmu/Dolls/Illum1/Exp/view3.png", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat img2 = cv::imread("G:/xiangmu/Dolls/Illum1/Exp/view3.png", CV_LOAD_IMAGE_GRAYSCALE);
//    cv::GaussianBlur(img2, img2, img2.size(),3);

//    rotateImg(img1, img1, 0, 0.4);
//    int dh = (img1.rows - img2.rows) / 2;
//    int dw = (img1.cols - img2.cols) / 2;
//    if (dh > 0 && dw > 0)
//    {
//        cv::copyMakeBorder(img2, img2, dh, dh, dw, dw, cv::BORDER_CONSTANT);
//    }

    if(img1.empty() || img2.empty())
    {
        printf("Can't read one of the images\n");
        return -1;
    }

    // detecting keypoints
    cv::SiftFeatureDetector detector(959);
    vector<cv::KeyPoint> keypoints1, keypoints2;
    detector.detect(img1, keypoints1);
    detector.detect(img2, keypoints2);
    cout << "left image interest point number: " << keypoints1.size() << endl;
    cout << "right image interest point number: " << keypoints2.size() << endl;

    // computing descriptors
    cv::SiftDescriptorExtractor extractor;
    cv::Mat descriptors1, descriptors2;
    extractor.compute(img1, keypoints1, descriptors1);
    extractor.compute(img2, keypoints2, descriptors2);

    // matching descriptors
    cv::BruteForceMatcher<cv::L2<float> > matcher;
    vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    std::vector<int> joke(matches.size());
    vector<cv::DMatch>::iterator begin = matches.begin();
    for(; begin != matches.end(); ++begin)
    {
        joke[begin->trainIdx] = 1;
    }
    int total = 0;
    for (int i = 0; i < matches.size(); ++i)
    {
        total += joke[i];
    }
    cout << "matched number: " << total << endl;

    // drawing the results
    cv::namedWindow("matches", 1);
    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
    cv::imshow("matches", img_matches);
    cv::waitKey(0);

    return 0;
}
*/

int main()
{
    // start algorithm
    string algStartStr("Algorithm starts!\n");
    string algEndStr("Algorithm ends!\n");
    string seperatorStr("=====================================\n\n");
    cout << algStartStr;
    cout << seperatorStr;

    // =======================================================================================
    // left image
    DEBUG_PRINT_SIMPLIFIED("\n\nLeft image processing starts!\n\n");
    cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp1/view0test.bmp"), cv::IMREAD_COLOR);
    //rotateImg(matLColor, matLColor, 0, 0.7);
    //cv::GaussianBlur(matRColor, matRColor, matRColor.size(),3);
    cv::Mat_<unsigned char> matL;
    cv::cvtColor(matLColor, matL, CV_BGR2GRAY);
    unsigned char* p_imgL = matL.data;
    unsigned short wL = matL.cols;
    unsigned short hL = matL.rows;

    // create the integrate image of the left image
    unsigned int* p_integImgL = createIntegImg(p_imgL, wL, hL);

    // create the det(Hessin) images of different octaves and layers of the left image
    double* p_detHesImgPyrL = (double*)calloc_check(OCTAVE_NUM * LAYER_NUM * wL * hL, sizeof(double));
    MaxDetHesOctave maxValL = createDetHesImgPyr(p_detHesImgPyrL, p_integImgL, OCTAVE_NUM, LAYER_NUM, wL, hL);

    // locate the interest points in the pyramid of the left image
    InterestPoint* p_pointsL = (InterestPoint*)calloc_check(wL * hL, sizeof(InterestPoint));
    unsigned char* p_markImgL = (unsigned char*)calloc_check(wL * hL, sizeof(unsigned char));
    PointNum numL = getPointsLocations(p_pointsL, p_markImgL, p_detHesImgPyrL, maxValL, OCTAVE_NUM, LAYER_NUM, wL, hL);
    unsigned int pointNumL = numL.calcNum;
    cout << "left image interest point number: " << numL.realNum << endl;

    // =======================================================================================
    // right image
    DEBUG_PRINT_SIMPLIFIED("\n\nRight image processing starts!\n\n");
    cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("G:/xiangmu/Pictures/Dolls/Illum1/Exp1/view1.png"), cv::IMREAD_COLOR);
    cv::Mat_<unsigned char> matR;
    cv::cvtColor(matRColor, matR, CV_BGR2GRAY);
    unsigned char* p_imgR = matR.data;
    unsigned short wR = matR.cols;
    unsigned short hR = matR.rows;

    // create the integrate image of the right image
    unsigned int* p_integImgR = createIntegImg(p_imgR, wR, hR);

    // create the det(Hessin) images of different octaves and layers of the right image
    double* p_detHesImgPyrR = (double*)calloc_check(OCTAVE_NUM * LAYER_NUM * wR * hR, sizeof(double));
    MaxDetHesOctave maxValR = createDetHesImgPyr(p_detHesImgPyrR, p_integImgR, OCTAVE_NUM, LAYER_NUM, wR, hR);

    // locate the interest points in the pyramid of the right image
    InterestPoint* p_pointsR = (InterestPoint*)calloc_check(wR * hR, sizeof(InterestPoint));
    unsigned char* p_markImgR = (unsigned char*)calloc_check(wR * hR, sizeof(unsigned char));
    PointNum numR = getPointsLocations(p_pointsR, p_markImgR, p_detHesImgPyrR, maxValR, OCTAVE_NUM, LAYER_NUM, wR, hR);
    unsigned int pointNumR = numR.calcNum;
    cout << "right image interest point number: " << numR.realNum << endl;

    // =======================================================================================
    // left image and right image get feats based on common scale
    unsigned short rL = sqrt(((double)(wL * hL)) / (numL.realNum));
    unsigned short rR = sqrt(((double)(wR * hR)) / (numR.realNum));
    unsigned short r = min(rL, rR);
    getPointsFeats(p_pointsL, p_markImgL, pointNumL, p_imgL, wL, r);
    getPointsFeats(p_pointsR, p_markImgR, pointNumR, p_imgR, wR, r);

    // show left image
    drawRect(matLColor, p_pointsL, pointNumL, 1, 2, cv::Scalar(255, 255, 0));
    cv::imshow("left marked image", matLColor);
    //cv::imwrite("leftMarked.png", matLColor);

    // show right image
    drawRect(matRColor, p_pointsR, pointNumR, 1, 2, cv::Scalar(255, 255, 0));
    cv::imshow("right marked image", matRColor);
    //cv::imwrite("rightMarked.png", matRColor);

    // =======================================================================================
    // match the two images
    unsigned int pairNum = 0;
    PointPair* p_pairs = matchInterestPoints(p_pointsL, pointNumL, p_pointsR, pointNumR, &pairNum, wL, hL, wR, hR);
    cout << "Matched pairs number: " << pairNum << endl;

    // show match result
    showMatchResult(matLColor, matRColor, p_pairs, pairNum, string("1.1.01"));

    // =======================================================================================
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

    // =======================================================================================
    // end
    cout << algEndStr;

    cv::waitKey();
    return 0;
}
