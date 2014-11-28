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

#define DETHES_MAX 1E20
#define DETHES_MIN -1E20

// maximum det(Hes) value recorded of all layers in one octave
typedef struct MAX_DETHES_LAYERS
{
    double maxPerLay[LAYER_NUM];
} MaxDetHesLayer;

// maximum det(Hes) value recorded of all octaves
typedef struct MAX_DETHES_OCTAVES
{
    MaxDetHesLayer maxPerOct[OCTAVE_NUM];
} MaxDetHesOctave;

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
#define FEAT_NUM 4
#define FEAT_MAX 1E100
#define FEAT_MIN -1E100
#define FEAT_OFFSET 1E-100

typedef struct HU_MATRIX
{
    featElemType feat[FEAT_NUM];
}HuMat;

typedef struct INTEREST_POINT
{
    coord c;
    HuMat mat;
}InterestPoint;

typedef struct POINT_NUM
{
    unsigned int realNum; // interest points that has been detected by surf
    unsigned int calcNum; // interest points that will not be calculated outside the image when calculating features
}PointNum;

typedef struct POINT_PAIR
{
    coord pL;
    coord pR;
}PointPair;

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
}ProjectMat;

// draw the rectangle around the interest point
void drawRect(cv::Mat& mat, const InterestPoint *p_points, unsigned short pointNum, unsigned short step,
              unsigned short r, cv::Scalar color);

// draw the rectangle around the interest point
void drawRect(unsigned char *p_img, const InterestPoint *p_points, unsigned short pointNum,
              unsigned short r, unsigned short w, unsigned char pixVal);

// init the pointers of the four corners of each box of certain filter
void initFiltPtrs(unsigned int *p_integImg, FiltCornPtrs* p_filtCornPtrss, const Filt *p_filts[], unsigned short w);

// increase the pointers of the four corners of each box of certain filter by certain value
void incFiltPtrs(FiltCornPtrs* p_filtCornPtrss, unsigned short val);

// calculate the det(Hessin) response of one pixel
void calcDetHes(const FiltCornPtrs* p_filtCornPtrss, double* p_detHesImg);

// create the det(Hessin) image of the input integral image at one specific octave and layer
void createDetHesImg(unsigned int *p_integImg, double* p_detHesImg, double *p_maxPerLay,
                     unsigned short octOrder, unsigned short layOrder, unsigned short w, unsigned short h);

// create the det(Hessin) image pyramid of certain number of octaves and layers
// returns pointers to the image pyramid, first pixel of the image in the first layer of the first octave
MaxDetHesOctave createDetHesImgPyr(double *p_detHesImgPyr, unsigned int *p_integImg,
                        unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h);

// find the interest point
// returns 0: current pixel is not the regional maximum point
// returns 1: current pixel is the regional maximum point
int isRegionMaximum(const double* p_in, unsigned short w, unsigned short h);


// get the interest points location
// returns the number of the founded interst points
PointNum getPointsLocations(InterestPoint* p_points, unsigned char* p_markImg,
                       const double* p_detHesImgPyr, MaxDetHesOctave maxVal,
                       unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h);

// calculate the feature of one interest point
void calcFeat(InterestPoint* p_point, const unsigned char* p_img, const unsigned char* p_markImg,
              const coord* p_coordHuMat, const coord* p_coordNeigh,
              int neighPointNumHuMat, int neightPointNumNeigh, unsigned short w);

// calculate the features of all the located interest points
void getPointsFeats(InterestPoint* p_points, const unsigned char* p_markImg, unsigned int pointNum,
                    const unsigned char* p_img, unsigned short w, unsigned short r);

// locate the interest points through the det(Hessin) image pyramid
// returns the pointer to the head of the array of interest points
InterestPoint* getInterestPoints(unsigned int* p_pointNum, const unsigned char* p_img,
                                 const double *p_detHesImgPyr, MaxDetHesOctave maxVal,
                                 unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h);

// calculate the mahala distance of two feature
// returns the feature distance
double calcFeatDistance(const InterestPoint* p_pointL, const InterestPoint* p_pointR);

// get the most similar(nearest) point of current point
const InterestPoint* getNearestPoint(const InterestPoint* p_pointCur, const InterestPoint* p_pointsRef,
                                     unsigned int pointNumRef);

// normalize all the feats
void normalizePointsFeats(InterestPoint* p_pointsL, unsigned int pointNumL,
                          InterestPoint* p_pointsR, unsigned int pointNumR);

// rough match based on mutual-minimum-distance
// returns the matched pair number
int roughMatch(const InterestPoint* p_pointsL, unsigned int pointNumL,
               const InterestPoint* p_pointsR, unsigned int pointNumR, PointPair* p_pairs);

// match the interest points of two images
PointPair* matchInterestPoints(InterestPoint *p_pointsL, int pointNumL,
                               InterestPoint *p_pointsR, int pointNumR, unsigned int* p_pairNum,
                               unsigned short wL, unsigned short hL, unsigned short wR, unsigned short hR);

// using ransac to get the best projection matrix based on the coarse matching pairs
ProjectMat getProjMatByRansac(const PointPair* p_pairs, unsigned int pairNum,
                              unsigned short wL, unsigned short hL,
                              unsigned short wR, unsigned short hR);

// draw the link of matched points of two images
void showMatchResult(const cv::Mat& matL, const cv::Mat& matR,
                     const PointPair* p_pairs, unsigned int pairNum, std::string testName);

#endif // FPLOCATE_H
