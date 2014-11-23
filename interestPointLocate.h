#ifndef INTERESTPOINTLOCATE_H
#define INTERESTPOINTLOCATE_H

#include "imgMath.h"
#include "opencv2/core/core.hpp"

// specific parameter data of the det(Hessin) pyramid construction ========================
// number of octaves
#define OCTAVE_NUM  3
// number of layers
#define LAYER_NUM   4
// number of filters(xx, yy, xy)
#define FILT_NUM  3
// number of factor parameters
#define PARAM_NUM   25
// maximum number of boxes
#define MAX_BOX_NUM 4


// filter box parameter structure
typedef struct P_FILTER_BOX
{
    int tLX; // x coordinate of the top left corner of the box
    int tLY; // y coordinate of the top left corner of the box
    int bRX; // x coordinate of the bottom right corner of the box
    int bRY; // y coordinate of the bottom right corner of the box
    int s;   // area(pixel number) of the first box
    int wei; // weight of the first box
} FilterBox;

// filter parameter structure -- one filter is like xx, or yy, or xy
typedef struct FILTER
{
    // filter box number
    int boxNum;

    // parameters of the boxes
    FilterBox box[MAX_BOX_NUM];
} Filt;

// structs that contains filter four corner coordinate pointers in the image
typedef struct FILTER_CORNER_PTRS
{
    const Filt* p_filt;

    // pointers of the four corners of each box
    unsigned int* p_boxTLCorn[MAX_BOX_NUM];
    unsigned int* p_boxTRCorn[MAX_BOX_NUM];
    unsigned int* p_boxBLCorn[MAX_BOX_NUM];
    unsigned int* p_boxBRCorn[MAX_BOX_NUM];
} FiltCornPtrs;

typedef double featElemType;
#define FEAT_NUM 3
#define FEAT_MAX 1E20
#define FEAT_MIN -1E20
#define FEAT_OFFSET 1E-20

typedef struct HU_MATRIX
{
    // feat[0]: log(n20)
    // feat[1]: log(n02)
    // feat[2]: log(abs(n11))
    featElemType feat[FEAT_NUM];
}huMat;

typedef struct INTEREST_POINT
{
    coord c;
    huMat mat;
}interestPoint;

typedef struct POINT_PAIR
{
    coord pL;
    coord pR;
}pointPair;

/*The 6 unknown coefficients for projection matrix*/
/*
    |	m1	m2	m3	|		|	x	|		|	x^	|
    |				|		|		|		|		|
    |	m4	m5	m6	|	*	| 	y	|	= 	|	y^	|
    |				|		|		|		|		|
    |	0	0	1	|		|	1	|		|	1	|
*/
typedef struct PROJECT_MATRIX
{
    double m1    ; double m2    ; double m3    ;
    double m4    ; double m5    ; double m6    ;
    double m7 = 0; double m8 = 0; double m9 = 1;
}projectMat;

// draw the rectangle around the interest point
void drawRect(unsigned char *p_img, const interestPoint *p_points, unsigned short pointNum, unsigned short r, unsigned short w);

// init the pointers of the four corners of each box of certain filter
void initFiltPtrs(unsigned int *p_integImg, FiltCornPtrs* p_filtCornPtrss, const Filt *p_filts[], unsigned short w);

// increase the pointers of the four corners of each box of certain filter by certain value
void incFiltPtrs(FiltCornPtrs* p_filtCornPtrss, unsigned short val);

// calculate the det(Hessin) response of one pixel
void calcDetHes(const FiltCornPtrs* p_filtCornPtrss, double* p_detHesImg);

// create the det(Hessin) image of the input integral image at one specific octave and layer
void createDetHesImg(unsigned int *p_integImg, double* p_detHesImg, unsigned short octOrder, unsigned short layOrder, unsigned short w, unsigned short h);

// create the det(Hessin) image pyramid of certain number of octaves and layers
// returns pointers to the image pyramid, first pixel of the image in the first layer of the first octave
double* createDetHesImgPyr(unsigned int *p_integImg, unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h);

// find the interest point
// returns 0: current pixel is not the regional maximum point
// returns 1: current pixel is the regional maximum point
int isRegionMaximum(const double* p_in, unsigned short w, unsigned short h);

// get the interest points location
// returns the number of the founded interst points
int getPointsLocations(interestPoint *p_points, const double *p_detHesImgPyr, unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h);

// calculate the feature of one interest point
void calcFeat(cv::Mat& mat, interestPoint* p_point, const unsigned char* p_img, const coord* p_coord, int neighPointNum, unsigned short w);

// calculate the features of all the located interest points
void getPointsFeats(interestPoint* p_points, unsigned int pointNum, const unsigned char* p_img, unsigned short w, unsigned short r);

// locate the interest points through the det(Hessin) image pyramid
// returns the pointer to the head of the array of interest points
interestPoint* getInterestPoints(unsigned int* p_pointNum, const unsigned char* p_img, const double *p_detHesImgPyr, unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h);

// calculate the mahala distance of two feature
// returns the feature distance
double calcFeatDistance(const interestPoint* p_pointL, const interestPoint* p_pointR);

// get the most similar(nearest) point of current point
const interestPoint* getNearestPoint(const interestPoint* p_pointCur, const interestPoint* p_pointsRef, unsigned int pointNumRef);

// rough match based on mutual-minimum-distance
// returns the matched pair number
int roughMatch(const interestPoint* p_pointsL, int pointNumL, const interestPoint* p_pointsR, int pointNumR, pointPair* p_pairs);

// match the interest points of two images
pointPair* matchInterestPoints(const interestPoint* p_pointsL, int pointNumL, const interestPoint* p_pointsR, int pointNumR, unsigned int* p_pairNum);

// using ransac to get the best projection matrix based on the coarse matching pairs
projectMat getProjMatByRansac(const pointPair* p_pairs, unsigned int pairNum);

// draw the link of matched points of two images
void showMatchResult(const cv::Mat& matL, const cv::Mat& matR, const pointPair* p_pairs, unsigned int pairNum);

#endif // FPLOCATE_H
