#ifndef INTERESTPOINTLOCATE_H
#define INTERESTPOINTLOCATE_H

#include "imgMath.h"

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

typedef struct HU_MATRIX
{
    double m0;
    double m1;
}huMat;

typedef struct INTEREST_POINT
{
    coord c;
    huMat mat;
}interestPoint;

typedef struct POINT_PAIR
{
    coord p1;
    coord p2;
}pointPair;

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
void calcFeat(interestPoint* p_point, const unsigned char* p_img, const coord* p_coord, int neighPointNum, unsigned short w, unsigned short h);

// calculate the features of all the located interest points
void getPointsFeats(interestPoint* p_points, unsigned int pointNum, const unsigned char* p_img, unsigned short w, unsigned short h, unsigned short r);

// locate the interest points through the det(Hessin) image pyramid
// returns the pointer to the head of the array of interest points
interestPoint* getInterestPoints(unsigned int* p_pointNum, const unsigned char* p_img, const double *p_detHesImgPyr, unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h);

// calculate the mahala distance of two feature
// returns the feature distance
double calcFeatDistance(interestPoint* p_pointL, interestPoint* p_pointR);

// match the interest points of two images
pointPair* matchPointsFeats(interestPoint* p_pointsL, int pointNumL, interestPoint* p_pointsR, int pointNumR, unsigned int* p_matchNum);

#endif // FPLOCATE_H
