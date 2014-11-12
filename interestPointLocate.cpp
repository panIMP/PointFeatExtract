#include "interestPointLocate.h"
#include <malloc.h>
#include <string.h>
#include "imgIO.h"
#include "imgMath.h"
#include <time.h>
#include <iostream>
using namespace std;


// filters parameters used to generate the Det(Hessin) pyramid for interest point location
// outer array is: octave number
// middle array is: layer number
// inner array is: filter number(xx, yy, xy) multiplies parameter number
// parameter from left to right are:
// start =============================
// box filter number,
// first box top left corner coordinate, first box bottom right corner coordinate,
// first box area, first box weight,
// second box top left corner coordinate, ...
// ...
// last box top left corner coordinate, ...
// ...
// correspording deata value of the simulated box filter of second order derivative of guassin.
// end ===============================
const Filt g_filts[OCTAVE_NUM][LAYER_NUM][FILT_NUM]=
{
    // 1st octave ===================================
    {
        // 1st layer -- box size: 9 * 9
        {
            // 1st filter -- xx
            {
                3, 0, 2,  2,  6, 15, 1,  3,  2, 5,   6, 15, -2,  6,  2,  8,  6, 15,  1,  0,  0,  0,  0,  0, 0,
            },
            // 2nd filter -- yy
            {
                3, 2, 0,  6,  2, 15, 1,  2,  3, 6,   5, 15, -2,  2,  6,  6,  8, 15,  1,  0,  0,  0,  0,  0, 0,
            },
            // 3rd filter -- xy
            {
                4, 1, 1,  3,  3,  9, 1,  5,  1, 7,   3,  9, -1,  1,  5,  3,  7,  9, -1,  5,  5,  7,  7,  9, 1,
            },
        },

        // 2nd layer -- box size: 15 * 15
        {
            {
                3, 0, 3,  4, 11, 45, 1,  5,  3,  9, 11, 45, -2, 10,  3, 14, 11, 45,  1,  0,  0,  0,  0,  0, 0,
            },

            {
                3, 3, 0, 11,  4, 45, 1,  3,  5, 11,  9, 45, -2,  3, 10, 11, 14, 45,  1,  0,  0,  0,  0,  0, 0,
            },

            {
                4, 2, 2,  6,  6, 25, 1,  8,  2, 12,  6, 25, -1,  2,  8,  6, 12, 25, -1,  8,  8, 12, 12, 25, 1,
            },
        },

        // 3rd layer -- box size: 21 * 21
        {
            {
                3, 0, 4,  6, 16, 91, 1,  7,  4, 13, 16, 91, -2, 14,  4, 20, 16, 91,  1,  0,  0,  0,  0,  0, 0,
            },

            {
                3, 4, 0, 16,  6, 91, 1,  4,  7, 16, 13, 91, -2,  4, 14, 16, 20, 91,  1,  0,  0,  0,  0,  0, 0,
            },

            {
                4, 3, 3,  9,  9, 49, 1, 11,  3, 17,  9, 49, -1,  3, 11,  9, 17, 49, -1, 11, 11, 17, 17, 49, 1,
            },
        },

        // 4th layer -- box size: 27 * 27
        {
            {
                3, 0, 5,  8, 21, 153, 1,  9,  5, 17, 21, 153, -2, 18,  5, 26, 21, 153,  1,  0,  0,  0,  0, 0,  0,
            },

            {
                3, 5, 0, 21,  8, 153, 1,  5,  9, 21, 17, 153, -2,  5, 18, 21, 26, 153,  1,  0,  0,  0,  0, 0,  0,
            },

            {
                4, 4, 4, 12, 12,  81, 1, 14,  4, 22, 12,  81, -1,  4, 14, 12, 22,  81, -1, 14, 14, 22, 22, 81, 1,
            },
        },
    },

    // 2nd octave ===================================
    {
        // 1st layer -- box size: 15 * 15
        {
            {
                3, 0, 3,  4, 11, 45, 1,  5,  3,  9, 11, 45, -2, 10,  3, 14, 11, 45,  1,  0,  0,  0,  0,  0, 0,
            },

            {
                3, 3, 0, 11,  4, 45, 1,  3,  5, 11,  9, 45, -2,  3, 10, 11, 14, 45,  1,  0,  0,  0,  0,  0, 0,
            },

            {
                4, 2, 2,  6,  6, 25, 1,  8,  2, 12,  6, 25, -1,  2,  8,  6, 12, 25, -1,  8,  8, 12, 12, 25, 1,
            },
        },

        // 2nd layer -- box size: 27 * 27
        {
            {
                3, 0, 5,  8, 21, 153, 1,  9,  5, 17, 21, 153, -2, 18,  5, 26, 21, 153,  1,  0,  0,  0,  0, 0,  0,
            },

            {
                3, 5, 0, 21,  8, 153, 1,  5,  9, 21, 17, 153, -2,  5, 18, 21, 26, 153,  1,  0,  0,  0,  0, 0,  0,
            },

            {
                4, 4, 4, 12, 12,  81, 1, 14,  4, 22, 12,  81, -1,  4, 14, 12, 22,  81, -1, 14, 14, 22, 22, 81, 1,
            },
        },

        // 3rd layer -- box size: 39 * 39
        {
            {
                3, 0, 7, 12, 31, 325, 1, 13,  7, 25, 31, 325, -2, 26,  7, 38, 31, 325,  1,  0,  0,  0,  0,   0, 0,
            },

            {
                3, 7, 0, 31, 12, 325, 1,  7, 13, 31, 25, 325, -2,  7, 26, 31, 28, 325,  1,  0,  0,  0,  0,   0, 0,
            },

            {
                4, 6, 6, 18, 18, 169, 1, 20,  6, 32, 18, 169, -1,  6, 20, 18, 32, 169, -1, 20, 20, 32, 32, 169, 1,
            },
        },

        // 4th layer -- box size: 51 * 51
        {
            {
                3, 0, 9, 16, 41, 561, 1, 17,  9, 33, 41, 561, -2, 34,  9, 50, 41, 561,  1,  0,  0,  0,  0, 0, 0
            },

            {
                3, 9, 0, 41, 16, 561, 1,  9, 17, 41, 33, 561, -2,  9, 34, 41, 50, 561,  1,  0,  0,  0,  0, 0, 0,
            },

            {
                4, 8, 8, 24, 24, 289, 1, 26,  8, 42, 24, 289, -1,  8, 26, 24, 42, 289, -1, 26, 26, 42, 42, 289, 1,
            },
        },
    },

    // 3rd octave ===================================
    {
        // 1st layer -- box size: 27 * 27
        {
            {
                3, 0, 5,  8, 21, 153, 1,  9,  5, 17, 21, 153, -2, 18,  5, 26, 21, 153,  1,  0,  0,  0,  0, 0, 0,
            },

            {
                3, 5, 0, 21,  8, 153, 1,  5,  9, 21, 17, 153, -2,  5, 18, 21, 26, 153,  1,  0,  0,  0,  0, 0, 0,
            },

            {
                4, 4, 4, 12, 12,  81, 1, 14,  4, 22, 12,  81, -1,  4, 14, 12, 22,  81, -1, 14, 14, 22, 22, 81, 1,
            },
        },

        // 2nd layer -- box size: 51 * 51
        {
            {
                3, 0, 9, 16, 41, 561, 1, 17,  9, 33, 41, 561, -2, 34,  9, 50, 41, 561,  1,  0,  0,  0,  0, 0, 0
            },

            {
                3, 9, 0, 41, 16, 561, 1,  9, 17, 41, 33, 561, -2,  9, 34, 41, 50, 561,  1,  0,  0,  0,  0, 0, 0,
            },

            {
                4, 8, 8, 24, 24, 289, 1, 26,  8, 42, 24, 289, -1,  8, 26, 24, 42, 289, -1, 26, 26, 42, 42, 289, 1,
            },
        },

        // 3rd layer -- box size: 75 * 75
        {
            {
                3, 0, 13, 24, 61, 1225, 1, 25, 13, 49, 61, 1225, -2, 50, 13, 74, 61, 1225,  1,  0,  0,  0,  0,   0, 0,
            },

            {
                3, 13, 0, 61, 24, 1225, 1, 13, 25, 61, 49, 1225, -2, 13, 50, 61, 74, 1225,  1,  0,  0,  0,  0,   0, 0,
            },

            {
                4, 12, 12, 36, 36, 625, 1, 38, 12, 62, 36,  625, -1, 12, 38, 36, 62,  625, -1, 38, 38, 62, 62, 625, 1,
            },
        },

        // 4th player -- box size: 99 * 99
        {
            {
                3, 0, 17, 32, 81, 2145, 1, 33, 17, 65, 81, 2145, -2, 66, 17, 98, 81, 2145,  1,  0,  0,  0,  0,   0, 0,
            },

            {
                3, 17, 0, 81, 32, 2145, 1, 17, 33, 81, 65, 2145, -2, 17, 66, 81, 98, 2145,  1,  0,  0,  0,  0,   0, 0,
            },

            {
                4, 16, 16, 48, 48, 1089, 1, 50,16, 82, 48, 1089, -1, 16, 50, 48, 82, 1089, -1, 50, 50, 82, 82, 1089, 1,
            },
        }
    },
};


// draw the rectangle around the interest point
void drawRect(unsigned char *p_img, const interestPoint *p_points, unsigned short pointNum, unsigned short r, unsigned short w)
{
    for (unsigned short p = 0; p < pointNum; ++p)
    {
        for (unsigned short y = p_points[p].c.y - r/2; y <= p_points[p].c.y + r/2; ++y)
        {
            for (unsigned short x = p_points[p].c.x - r/2; x <= p_points[p].c.x + r/2; ++x)
            {
                p_img[y * w + x] = 255;
            }
        }
    }
}

// init the pointers of the four corners of each box of certain filter
void initFiltPtrs(unsigned int *p_integImg, FiltCornPtrs* p_filtCornPtrss, const Filt* p_filts[], unsigned short w)
{
    for (unsigned short f = 0; f < FILT_NUM; ++f)
    {
        p_filtCornPtrss[f].p_filt = p_filts[f];
        for (unsigned short b = 0; b < p_filts[f]->boxNum; ++b)
        {
            p_filtCornPtrss[f].p_boxTLCorn[b] = p_integImg + p_filts[f]->box[b].tLY * w + p_filts[f]->box[b].tLX;
            p_filtCornPtrss[f].p_boxTRCorn[b] = p_integImg + p_filts[f]->box[b].tLY * w + p_filts[f]->box[b].bRX + 1;
            p_filtCornPtrss[f].p_boxBLCorn[b] = p_integImg + (p_filts[f]->box[b].bRY + 1) * w + p_filts[f]->box[b].tLX;
            p_filtCornPtrss[f].p_boxBRCorn[b] = p_integImg + (p_filts[f]->box[b].bRY + 1) * w + p_filts[f]->box[b].bRX + 1;
        }
    }
}

// increase the pointers of the four corners of each box of certain filter by certain value
void incFiltPtrs(FiltCornPtrs* p_filtCornPtrss, unsigned short val)
{
    for (unsigned short f = 0; f < FILT_NUM; ++f)
    {
        for (unsigned short b = 0; b < p_filtCornPtrss[f].p_filt->boxNum; ++b)
        {
            p_filtCornPtrss[f].p_boxTLCorn[b] += val;
            p_filtCornPtrss[f].p_boxTRCorn[b] += val;
            p_filtCornPtrss[f].p_boxBLCorn[b] += val;
            p_filtCornPtrss[f].p_boxBRCorn[b] += val;
        }
    }
}

// calculate the det(Hessin) response of one pixel
void calcDetHes(const FiltCornPtrs *p_filtCornPtrss, double* p_detHesImg)
{
    double deriVal[FILT_NUM] = {0.0, 0.0, 0.0};

    unsigned short boxNum;
    FiltCornPtrs ptrs;
    for (unsigned short f = 0; f < FILT_NUM; ++f)
    {
        ptrs = p_filtCornPtrss[f];
        boxNum = ptrs.p_filt->boxNum;
        unsigned int val = 0;
        for (unsigned short b = 0; b < boxNum; ++b)
        {
            val = *(ptrs.p_boxBRCorn[b]) + *(ptrs.p_boxTLCorn[b]) - *(ptrs.p_boxBLCorn[b]) - *(ptrs.p_boxTRCorn[b]);

            deriVal[f] += (double)val * (double)(ptrs.p_filt->box[b].wei) / (double)(ptrs.p_filt->box[b].s);
        }
    }

    *p_detHesImg = deriVal[0] * deriVal[1] - 0.81 * deriVal[2] * deriVal[2];
}

// create the det(Hessin) image of the input integral image at one specific octave and layer
void createDetHesImg(unsigned int *p_integImg, double* p_detHesImg, unsigned short octOrder, unsigned short layOrder, unsigned short w, unsigned short h)
{
    // fetch params for box-filtering
    const Filt* p_filt = &g_filts[octOrder][layOrder][0];

    // filtering pixel coordinate range
    unsigned short stX = (p_filt->box[1].tLX + p_filt->box[1].bRX) / 2 + 1;
    unsigned short stY = (p_filt->box[1].tLY + p_filt->box[1].bRY) / 2 + 1;
    unsigned short enX = w - stX;
    unsigned short enY = h - stY;

    // the filters of one layer
    const Filt* p_filts[FILT_NUM] = {p_filt, p_filt + 1, p_filt + 2};

    // pointers of the filter box corners
    FiltCornPtrs filtCornPtrss[FILT_NUM];
    initFiltPtrs(p_integImg, filtCornPtrss, p_filts, w);

    // iterate over the image
    FiltCornPtrs t_filtCornPtrss[FILT_NUM];
    double* t_p_detHesImg;
    for (unsigned short y = stY; y < enY; ++y, incFiltPtrs(filtCornPtrss, w))
    {
        t_p_detHesImg = p_detHesImg + y * w + stX;

        for (unsigned short f = 0; f < FILT_NUM; ++f)
            t_filtCornPtrss[f] = filtCornPtrss[f];

        for (unsigned short x = stX; x < enX; ++x, ++t_p_detHesImg, incFiltPtrs(t_filtCornPtrss, 1))
        {
            calcDetHes(t_filtCornPtrss, t_p_detHesImg);
        }
    }
}

// create the det(Hessin) image pyramid of certain number of octaves and layers
// returns pointers to the image pyramid, first pixel of the image in the first layer of the first octave
double* createDetHesImgPyr(unsigned int *p_integImg, unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h)
{
    double* p_detHesImgPyr = (double*)calloc(OCTAVE_NUM * LAYER_NUM * w * h, sizeof(double));

    // create pyramid
    for (unsigned short octOrder = 0; octOrder < octNum; ++octOrder)
    {
        for (unsigned short layOrder = 0; layOrder < layNum; ++layOrder)
        {
            createDetHesImg(p_integImg, p_detHesImgPyr, octOrder, layOrder, w, h);
            p_detHesImgPyr += w * h;
        }
    }

    return (p_detHesImgPyr - w * h * OCTAVE_NUM * LAYER_NUM);
}

// find the interest point
// returns 0: current pixel is not the regional maximum point
// returns 1: current pixel is the regional maximum point
int isRegionMaximum(const double *p_in, unsigned short w, unsigned short h)
{
    double i00, i01, i02;
    double i10, i11, i12;
    double i20, i21, i22;
    unsigned int totalSize = w * h;
    double val = *p_in;

    // current layer of image
    i00=*(p_in-w-1);    i01=*(p_in    -w);    i02=*(p_in-w+1);
    i10=*(p_in  -1);    i11=*(p_in      );    i12=*(p_in  +1);
    i20=*(p_in+w-1);    i21=*(p_in    +w);    i22=*(p_in+w+1);

    if (i11 <= i00 || i11 <= i01 || i11 <= i02
            || i11 <= i10 || i11 <= i12
            || i11 <= i20 || i11 <= i21 || i11 <= i22)
        return 0;

    // upper layer of image
    const double* p_inUpper = p_in + totalSize;

    i00=*(p_inUpper-w-1);    i01=*(p_inUpper-w);    i02=*(p_inUpper-w+1);
    i10=*(p_inUpper  -1);    i11=*(p_inUpper  );    i12=*(p_inUpper  +1);
    i20=*(p_inUpper+w-1);    i21=*(p_inUpper+w);    i22=*(p_inUpper+w+1);

    if (val <= i00 || val <= i01 || val <= i02
            || val <= i10 || val <= i11 || val <= i12
            || val <= i20 || val <= i21 || val <= i22)
        return 0;

    // lower layer of image
    const double* p_inLower = p_in - totalSize;

    i00=*(p_inLower-w-1);    i01=*(p_inLower-w);    i02=*(p_inLower-w+1);
    i10=*(p_inLower  -1);    i11=*(p_inLower  );    i12=*(p_inLower  +1);
    i20=*(p_inLower+w-1);    i21=*(p_inLower+w);    i22=*(p_inLower+w+1);

    if (val <= i00 || val <= i01 || val <= i02
            || val <= i10 || val <= i11 || val <= i12
            || val <= i20 || val <= i21 || val <= i22)
        return 0;

    // current pixel is the maximum of 26 neighbour pixels
    return 1;
}


// get the interest points location
// returns the number of the founded interst points
int getPointsLocations(interestPoint* p_points, const double* p_detHesImgPyr, unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h)
{
    unsigned int totalSize = w * h;
    unsigned int filtSize = w * (h - 2) - 2;
    interestPoint* p_out = p_points;
    unsigned char* p_markImg = (unsigned char*)calloc(totalSize, sizeof(unsigned char));

    for (unsigned short octOrder = 0; octOrder < octNum; ++octOrder)
    {
        for (unsigned short layOrder = 1; layOrder < layNum - 1; ++layOrder)
        {
            const double* p_detHesImg = p_detHesImgPyr + (octOrder * layNum + layOrder) * totalSize;

            for (unsigned int c = 0; c < filtSize; ++c, ++p_detHesImg)
            {
                if (isRegionMaximum(p_detHesImg, w, h))
                {
                    // mark the interest points first
                    p_markImg[c + w + 1] = 1;
                }
            }
        }
    }

    // store the pixels into interst point arrays
    for (unsigned int i = 0; i < totalSize; ++i)
    {
        if (p_markImg[i] == 1)
        {
            p_out->c.y = i / w;
            p_out->c.x = i - p_out->c.y * w;
            p_out++;
        }
    }

    // free memory
    free(p_markImg);

    return p_out - p_points;
}

// calculate the feature of one interest point
void calcFeat(interestPoint* p_point, const unsigned char* p_img, const coord* p_coord, int neighPointNum, unsigned short w, unsigned short h)
{
    unsigned short xCenter = p_point->c.x;
    unsigned short yCenter = p_point->c.y;
    const coord* p_coordEnd = p_coord + neighPointNum;

    unsigned int sum = 0;
    unsigned int sumX = 0;
    unsigned int sumY = 0;
    unsigned int sumXY = 0;
    unsigned int sumX2 = 0;
    unsigned int sumY2 = 0;
    unsigned int x,y,val,valX, valY;

    for (; p_coord != p_coordEnd; ++p_coord)
    {
        x = p_coord->x + xCenter;
        y = p_coord->y + yCenter;
        val = p_img[y * w + x];
        valX = val * x;
        valY = val * y;

        sum += val;
        sumX += valX;
        sumY += valY;
        sumX2 += valX * x;
        sumY2 += valY * y;
        sumXY += valX * y;
    }

    double xEven = (double)sumX / (double)sum;
    double yEven = (double)sumY / (double)sum;
    double u00 = sum;
    double u20 = sumX2 - 2 * xEven * sumX + xEven * xEven * u00;
    double u02 = sumY2 - 2 * yEven * sumY + yEven * yEven * u00;
    //double u11 = sumXY - xEven * sumY - yEven * sumX + xEven * yEven * u00;
    double n20 = u20 / (u00 * u00);
    double n02 = u02 / (u00 * u00);
    //double n11 = u11 / (u00 * u00);

    p_point->mat.m0 = n20;
    p_point->mat.m1 = n02;
}

// calculate the features of all the located interest points
void getPointsFeats(interestPoint* p_points, unsigned int pointNum, const unsigned char *p_img, unsigned short w, unsigned short h, unsigned short r)
{
    unsigned int neighPointNum = 0;
    coord* p_coord = calcDiskTmplArray(r, &neighPointNum);

    interestPoint* p_pointsEnd = p_points + pointNum;

    for (; p_points != p_pointsEnd; ++p_points)
    {
        calcFeat(p_points, p_img, p_coord, neighPointNum, w, h);
    }

    free(p_coord);
    p_coord = NULL;
}

// locate the interest points through the det(Hessin) image pyramid
// returns the pointer to the head of the array of interest points
interestPoint *getInterestPoints(unsigned int* p_pointNum, const unsigned char *p_img, const double* p_detHesImgPyr, unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h)
{
    interestPoint* p_points = (interestPoint*)calloc(w * h, sizeof(interestPoint));

    // step 1: get the interest points
    *p_pointNum = getPointsLocations(p_points, p_detHesImgPyr, octNum, layNum, w, h);

    // step 2: calculate the feature of the interest points
    getPointsFeats(p_points, *p_pointNum, p_img, w, h, 5);

    return p_points;
}

// calculate the mahala distance of two feature
// returns the feature distance
double calcFeatDistance(interestPoint* p_pointL, interestPoint* p_pointR)
{
    double distM0 = p_pointL->mat.m0 - p_pointR->mat.m0;
    double distM1 = p_pointL->mat.m1 - p_pointR->mat.m1;

    double dist = distM0 * distM0 + distM1 * distM1;

    return dist;
}

// match the interest points of two images
// returns the matched pairs of interest points
pointPair *matchPointsFeats(interestPoint* p_pointsL, int pointNumL, interestPoint* p_pointsR, int pointNumR, unsigned int* p_matchNum)
{
    unsigned int maxMatchedPairNum = max(pointNumL, pointNumR);
    pointPair* p_matchedPair = (pointPair*)calloc(maxMatchedPairNum, sizeof(pointPair));
    pointPair* p_curMatchedPair = p_matchedPair;

    interestPoint* p_pointsLEnd = p_pointsL + pointNumL;
    interestPoint* p_pointsREnd = p_pointsR + pointNumR;

    for (; p_pointsL != p_pointsLEnd; ++p_pointsL)
    {
        double minDist = 1000.0;
        interestPoint* p_matchedR = NULL;

        for (; p_pointsR != p_pointsREnd; ++p_pointsR)
        {
            double dist = calcFeatDistance(p_pointsL, p_pointsR);
            if (dist < minDist)
            {
                p_matchedR = p_pointsR;
                minDist = dist;
            }
        }

        if (p_matchedR != NULL)
        {
            p_curMatchedPair->p1 = p_pointsL->c;
            p_curMatchedPair->p2 = p_matchedR->c;
            p_curMatchedPair++;
        }
    }

    *p_matchNum = p_curMatchedPair - p_matchedPair;

    return p_matchedPair;
}






































