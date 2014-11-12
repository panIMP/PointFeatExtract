#include <iostream>
#include <string>

#include "imgIO.h"
#include "opencv2/highgui/highgui.hpp"
#include "interestPointLocate.h"
#include "opencv2/nonfree/features2d.hpp"

#include <time.h>

using namespace std;


int main()
{
    // start algorithm
    string algStartStr("Algorithm starts!\n");
    string algEndStr("Algorithm ends!\n");
    string seperatorStr("=====================================\n\n");
    cout << algStartStr;
    cout << seperatorStr;

    // left image ============================================================================
    cv::Mat_<unsigned char> matL = cv::imread(string("/home/hupan/destop/Dolls/Illum2/Exp0/view0.png"), cv::IMREAD_GRAYSCALE);
    unsigned char* p_imgL = matL.data;
    unsigned short wL = matL.cols;
    unsigned short hL = matL.rows;

    // create the integrate image of the left image
    unsigned int* p_integImgL = createIntegImg(p_imgL, wL, hL);

    // create the det(Hessin) images of different octaves and layers of the left image
    double* p_detHesImgPyrL = createDetHesImgPyr(p_integImgL, OCTAVE_NUM, LAYER_NUM, wL, hL);

    // locate the interest points in the pyramid of the left image
    unsigned int pointNumL = 0;
    interestPoint* p_pointsL = getInterestPoints(&pointNumL, p_imgL, p_detHesImgPyrL, OCTAVE_NUM, LAYER_NUM, wL, hL);

    // show left image
    drawRect(p_imgL, p_pointsL, pointNumL, 2, wL);
    cv::imshow("left marked image", matL);

    // free memory
    free(p_integImgL);
    free(p_detHesImgPyrL);

    // right image ===========================================================================
    cv::Mat_<unsigned char> matR = cv::imread(string("/home/hupan/destop/Dolls/Illum2/Exp0/view5.png"), cv::IMREAD_GRAYSCALE);
    unsigned char* p_imgR = matR.data;
    unsigned short wR = matR.cols;
    unsigned short hR = matR.rows;

    // create the integrate image of the right image
    unsigned int* p_integImgR = createIntegImg(p_imgR, wR, hR);

    // create the det(Hessin) images of different octaves and layers of the right image
    double* p_detHesImgPyrR = createDetHesImgPyr(p_integImgR, OCTAVE_NUM, LAYER_NUM, wR, hR);

    // locate the interest points in the pyramid of the left image
    unsigned int pointNumR = 0;
    interestPoint* p_pointsR = getInterestPoints(&pointNumR, p_imgR, p_detHesImgPyrR, OCTAVE_NUM, LAYER_NUM, wR, hR);

    // show right image
    drawRect(p_imgR, p_pointsR, pointNumR, 2, wR);
    cv::imshow("right marked image", matR);

    // free memory
    free(p_integImgR);
    free(p_detHesImgPyrR);

    // match the two images ==================================================================
    unsigned int matchNum = 0;
    pointPair* p_matchedPair = matchPointsFeats(p_pointsL, pointNumL, p_pointsR, pointNumR, &matchNum);




    // free memory
    free(p_pointsL);
    free(p_pointsR);

    // merge the input images into one image
    cv::Mat_<unsigned char> imgArr[2] = {matL, matR};
    cv::Mat_<unsigned char> mergedImg = mergeImgs(imgArr, 2, horizontal);
    cv::imshow("merged initial images", mergedImg);

    cv::waitKey();
    cout << algEndStr;

    return 0;
}

