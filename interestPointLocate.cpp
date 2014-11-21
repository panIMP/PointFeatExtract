#include "interestPointLocate.h"
#include "error.h"
#include <malloc.h>
#include <string.h>
#include "imgIO.h"
#include "imgMath.h"
#include <time.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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
    if (p_img == NULL || p_points == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of image pointer or interest points pointer");
        exit(-1);
    }

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
    if (p_integImg == NULL || p_filtCornPtrss == NULL || p_filts == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of integral image pointer or filter corner pointers or filter template pointer");
        exit(-1);
    }

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
    if (p_filtCornPtrss == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of filter corner pointers");
        exit(-1);
    }

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
    if (p_filtCornPtrss == NULL || p_detHesImg == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of filter corner pointers or Det(Hessin) image pointer");
        exit(-1);
    }

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
void createDetHesImg(unsigned int *p_integImg, double* p_detHesImg,
                     unsigned short octOrder, unsigned short layOrder, unsigned short w, unsigned short h)
{
    if (p_integImg == NULL || p_detHesImg == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of integral image pointer or Det(Hessin) image");
        exit(-1);
    }

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
    if (p_integImg == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of the pointer of integral image");
        exit(-1);
    }

    double* p_detHesImgPyr = (double*)calloc_check(OCTAVE_NUM * LAYER_NUM * w * h, sizeof(double));

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
    if (p_in == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of image pointer");
        exit(-1);
    }

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
int getPointsLocations(interestPoint* p_points, const double* p_detHesImgPyr,
                       unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h)
{
    if (p_points == NULL || p_detHesImgPyr == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of interest pointe set pointer or Det(Hessin) pointer");
        exit(-1);
    }

    unsigned int totalSize = w * h;
    unsigned int filtSize = w * (h - 2) - 2;
    interestPoint* p_out = p_points;
    unsigned char* p_markImg = (unsigned char*)calloc_check(totalSize, sizeof(unsigned char));

    for (unsigned short octOrder = 2; octOrder < octNum; ++octOrder)
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
void calcFeat(interestPoint* p_point, const unsigned char* p_img, const coord* p_coord, int neighPointNum, unsigned short w)
{
    if (p_point == NULL || p_img == NULL || p_coord == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of interest pointer set pointer or image pointer or array pointer");
        exit(-1);
    }

    unsigned short xCenter = p_point->c.x;
    unsigned short yCenter = p_point->c.y;
    const coord* p_coordEnd = p_coord + neighPointNum;

    unsigned long long sum = 0;
    unsigned long long sumX = 0;
    unsigned long long sumY = 0;
    unsigned long long sumXY = 0;
    unsigned long long sumX2 = 0;
    unsigned long long sumY2 = 0;
    unsigned long long x,y,val,valX, valY;

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

    double xEven = ((double)sumX) / ((double)sum + 1e-5);
    double yEven = ((double)sumY) / ((double)sum + 1e-5);
    double u00 = sum;
    double u20 = sumX2 - 2 * xEven * sumX + xEven * xEven * u00;
    double u02 = sumY2 - 2 * yEven * sumY + yEven * yEven * u00;
    double u11 = sumXY - xEven * sumY - yEven * sumX + xEven * yEven * u00;
    double u00Square = u00 * u00;
    double n20 = (u20 + 1e-5) / (u00Square + 1e-5);
    double n02 = (u02 + 1e-5) / (u00Square + 1e-5);
    double n11 = (u11 + 1e-5) / (u00Square + 1e-5);

    p_point->mat.feat[0] = log(n20);
    p_point->mat.feat[1] = log(n02);
    p_point->mat.feat[2] = log(abs(n11));
}

// calculate the features of all the located interest points
void getPointsFeats(interestPoint* p_points, unsigned int pointNum, const unsigned char *p_img, unsigned short w, unsigned short r)
{
    if (p_points == NULL || p_img == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of points set pointer");
        exit(-1);
    }

    unsigned int neighPointNum = 0;
    coord* p_coord = calcDiskTmplArray(r, &neighPointNum);

    interestPoint* p_pointsEnd = p_points + pointNum;
    interestPoint* p_pointsCur = p_points;

    // the sum of all feat and the the sample deta -- for normalization
    /*featElemType featSum[FEAT_NUM] = {0.0};
    featElemType featDeta[FEAT_NUM] = {0.0};
    featElemType featEven[FEAT_NUM] = {0.0};*/

    // get the points feat
    unsigned int curPointSeq = 1;
    for (; p_pointsCur != p_pointsEnd; ++p_pointsCur, ++curPointSeq)
    {
        calcFeat(p_pointsCur, p_img, p_coord, neighPointNum, w);

        /*for (int f = 0; f < FEAT_NUM; ++f)
        {
            featSum[f] += curMat.feat[f];
        }*/

        DEBUG_PRINT_SIMPLIFIED("Point%4d: (%4d, %4d), Feat: (%lf, %lf, %lf)\n", curPointSeq, p_pointsCur->c.x, p_pointsCur->c.y,
                               p_pointsCur->mat.feat[0], p_pointsCur->mat.feat[1], p_pointsCur->mat.feat[2]);
    }
    /*for (int f = 0; f < FEAT_NUM; ++f)
    {
        featEven[f] = featSum[f] / pointNum;

        featElemType featDistSquareSum = 0.0;
        for (p_pointsCur = p_points; p_pointsCur != p_pointsEnd; ++p_pointsCur)
        {
            featDistSquareSum += pow((p_pointsCur->mat.feat[f] - featEven[f]), 2);
        }

        featDeta[f] = featDistSquareSum / (pointNum - 1);
    }

    // normalize the points feat
    curPointSeq = 1;
    for (p_pointsCur = p_points; p_pointsCur != p_pointsEnd; ++p_pointsCur, ++curPointSeq)
    {
        for (int f = 0; f < FEAT_NUM; ++f)
        {
            p_pointsCur->mat.feat[f] = (p_pointsCur->mat.feat[f] - featEven[f]) / featDeta[f];
        }

        DEBUG_PRINT_SIMPLIFIED("Point%4d: (%4d, %4d), Feat: (%lf, %lf, %lf)\n", curPointSeq, p_pointsCur->c.x,
        p_pointsCur->c.y, p_pointsCur->mat.feat[0], p_pointsCur->mat.feat[1], p_pointsCur->mat.feat[2]);
    }*/

    free(p_coord);
    p_coord = NULL;
}

// locate the interest points through the det(Hessin) image pyramid
// returns the pointer to the head of the array of interest points
interestPoint *getInterestPoints(unsigned int* p_pointNum, const unsigned char *p_img, const double* p_detHesImgPyr,
                                 unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h)
{
    if (p_pointNum == NULL || p_img == NULL || p_detHesImgPyr == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    interestPoint* p_points = (interestPoint*)calloc_check(w * h, sizeof(interestPoint));

    // step 1: get the interest points
    *p_pointNum = getPointsLocations(p_points, p_detHesImgPyr, octNum, layNum, w, h);

    // step 2: calculate the feature of the interest points
    getPointsFeats(p_points, *p_pointNum, p_img, w, 6);

    return p_points;
}

// calculate the mahala distance of two feature
// returns the feature distance
double calcFeatDistance(const interestPoint *p_pointL, const interestPoint *p_pointR)
{
    if (p_pointL == NULL || p_pointR == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    double dist = 0.0;
    double tempDist = 0.0;
    for (int f = 0; f < FEAT_NUM; ++f)
    {
        tempDist = p_pointL->mat.feat[f] - p_pointR->mat.feat[f];
        dist += tempDist * tempDist;
    }

    return dist;
}


// get the most similar(nearest) point of current point
const interestPoint* getNearestPoint(const interestPoint *p_pointCur, const interestPoint *p_pointsRef, unsigned int pointNumRef)
{
    if (p_pointsRef == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    if (p_pointCur == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    const interestPoint* p_pointsRefNearest = NULL;
    const interestPoint* p_pointsRefEnd = p_pointsRef + pointNumRef;
    double minDist = 10000.0;

    for (; p_pointsRef != p_pointsRefEnd; ++p_pointsRef)
    {
        double dist = calcFeatDistance(p_pointCur, p_pointsRef);
        if (dist < minDist)
        {
            p_pointsRefNearest = p_pointsRef;
            minDist = dist;
        }
    }

    return p_pointsRefNearest;
}

// rough match based on mutual-minimum-distance
// returns the matched pair number
int roughMatch(const interestPoint* p_pointsL, int pointNumL, const interestPoint* p_pointsR, int pointNumR, pointPair* p_pairs)
{
    if (p_pointsL == NULL || p_pointsR == NULL || p_pairs == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    pointPair* p_pairsCur = p_pairs;

    const interestPoint* p_pointsLCur = p_pointsL;
    const interestPoint* p_pointsLStart = p_pointsL;
    const interestPoint* p_pointsLEnd = p_pointsL + pointNumL;

    const interestPoint* p_pointsRStart = p_pointsR;

    for (; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur)
    {
        const interestPoint* p_matchedR = getNearestPoint(p_pointsLCur, p_pointsRStart, pointNumR);
        if (p_matchedR == NULL)
        {
            DEBUG_PRINT_DETAILED("Left image Point %ld has no nearest Right image point", p_pointsLCur - p_pointsLStart + 1);
        }
        else
        {
            const interestPoint* p_matchedL = getNearestPoint(p_matchedR, p_pointsLStart, pointNumL);
            if (p_matchedL == NULL)
            {
                DEBUG_PRINT_DETAILED("Right image Point %ld has no nearest Left image point", p_matchedR - p_pointsRStart + 1);
            }
            else
            {
                if (p_matchedL == p_pointsLCur)
                {
                    p_pairsCur->pL = p_matchedL->c;
                    p_pairsCur->pR = p_matchedR->c;
                    p_pairsCur++;
                }
            }
        }
   }

    return (p_pairsCur - p_pairs);
}


// using ransac to get the best projection matrix based on the coarse matching pairs
projectMat getProjMatByRansac(const pointPair *p_pairs, unsigned int pairNum)
{
    if (p_pairs == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    // threshold for distance between expectedly matched point position and actually matched point position
    // > distThresh -- inner point
    // < distThresh -- outer point
    double distThresh = 250;

    // threshold for ending the iteration
    // > innerPointNumThresh -- correct enough projection matrix coefficiency has been found
    // < innerPointNumThresh -- not yet
    unsigned int maxInnerPointNum = 0;

    // projection matrix coefficiency
    projectMat mat, suitMat;

    unsigned int iterateNum = 2000*pairNum;
    for (unsigned int i = 0; i < iterateNum; ++i)
    {
        unsigned int start1 = rand() % pairNum;
        unsigned int start2 = rand() % pairNum;
        unsigned int start3 = rand() % pairNum;
        const pointPair* p_pair1 = p_pairs + start1;
        const pointPair* p_pair2 = p_pairs + start2;
        const pointPair* p_pair3 = p_pairs + start3;

        // The 6 unknown coefficients for projection matrix
        //
        //
        // | m1	m2	m3 |		|	x	|		|	x^	|
        // |		   |		|		|		|		|
        // | m4	m5	m6 |	*	| 	y	|	= 	|	y^	|
        // |		   |		|		|		|		|
        // | 0	0	1  |		|	1	|		|	1	|
        //
        // ==>
        //
        // m1 * x + m2 * y + m3 = x^   ==>   x * m1 + y * m2 + 1 * m3 = x^
        //
        // ==>
        //
        // | x1 y1 1 |      | m1 |     | x1^ |
        // |         |      |    |     |
        // | x2 y2 1 |  *   | m2 |  =  | x2^ |
        // |         |      |    |     |
        // | x3 y3 1 |      | m3 |     | x3^ |
        //
        // ==> m4 * x + m5 * y + m6 = y^ is the same as above
        //
        // (x1, y1), (x2, y2), (x3, y3) stands for three left points and so do (x1^, y1^), (x2^, y2^) ...
        //
        // pick three roughly matched pairs to calculate th pairNum - 2e m1, m2, m3....
        // matA stands for:
        // | x1 y1 1 |
        // |         |
        // | x2 y2 1 |
        // |         |
        // | x3 y3 1 |
        // matB stands for:
        // | x1^ |      | y1^ |
        // |     |      |     |
        // | x2^ |  OR  | y2^ |
        // |     |      |     |
        // | x3^ |      | y3^ |
        // mat123 and mat456 stands for m1,m2,m3 etc...
        // matA * mat123 = matB OR matA * mat456 = matB
        cv::Mat_<double> matA = (cv::Mat_<double>(3, 3) << p_pair1->pL.x, p_pair1->pL.y, 1,
                                 p_pair2->pL.x, p_pair2->pL.y, 1, p_pair3->pL.x, p_pair3->pL.y, 1);
        cv::Mat_<double> matB = (cv::Mat_<double>(3, 1) << p_pair1->pR.x, p_pair2->pR.x, p_pair3->pR.x);
        cv::Mat_<double> mat123;

        // if mat123 are solved out, continue to solve mat456, or discard it
        if (cv::solve(matA, matB, mat123, cv::DECOMP_NORMAL | cv::DECOMP_SVD))
        {
            cv::Mat_<double> matB = (cv::Mat_<double>(3, 1) << p_pair1->pR.y, p_pair2->pR.y, p_pair3->pR.y);
            cv::Mat_<double> mat456;

            // if mat456 are solved out, continue calculate the inner point num
            if (cv::solve(matA, matB, mat456, cv::DECOMP_NORMAL | cv::DECOMP_SVD))
            {
                // the coefficients have been solved out
                mat.m1 = mat123.at<double>(0, 0);
                mat.m2 = mat123.at<double>(1, 0);
                mat.m3 = mat123.at<double>(2, 0);
                mat.m4 = mat456.at<double>(0, 0);
                mat.m5 = mat456.at<double>(1, 0);
                mat.m6 = mat456.at<double>(2, 0);

                // calculate the inner point number under current coefficients
                const pointPair* p_pairCur = p_pairs;
                const pointPair* p_pairEnd = p_pairs + pairNum;
                unsigned int innerPointNum = 0;

                for (; p_pairCur != p_pairEnd; ++p_pairCur)
                {
                    double expectedRx = mat.m1 * p_pairCur->pL.x + mat.m2 * p_pairCur->pL.y + mat.m3;
                    double expectedRy = mat.m4 * p_pairCur->pL.x + mat.m5 * p_pairCur->pL.y + mat.m6;
                    double dist = pow((expectedRx - p_pairCur->pR.x), 2) + pow((expectedRy - p_pairCur->pR.y), 2);
                    if (dist < distThresh)
                    {
                        ++innerPointNum;
                    }
                }

                // only record the coefficients that generates maximum inner point num
                if (innerPointNum > maxInnerPointNum)
                {
                    maxInnerPointNum = innerPointNum;
                    suitMat = mat;
                }
            }
        }
    }

    return suitMat;
}

// match the interest points of two images
// returns the matched pairs of interest points
pointPair *matchInterestPoints(const interestPoint *p_pointsL, int pointNumL,
                               const interestPoint *p_pointsR, int pointNumR, unsigned int* p_pairNum)
{
    if (p_pointsL == NULL || p_pointsR == NULL || p_pairNum == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    pointPair* p_pairs = (pointPair*)calloc_check(max(pointNumL, pointNumR), sizeof(pointPair));

    // step 1: rough match based on mutual-minimum-distance
    *p_pairNum = roughMatch(p_pointsL, pointNumL, p_pointsR, pointNumR, p_pairs);

    // step 2: get the projection match based on ransac
    projectMat mat = getProjMatByRansac(p_pairs, *p_pairNum);

    // step 3: rectify the match based on ransac
    pointPair* p_pairsEnd = p_pairs + *p_pairNum;
    pointPair* p_pairsCur = p_pairs;
    for (; p_pairsCur != p_pairsEnd; ++p_pairsCur)
    {
        p_pairsCur->pR.x = p_pairsCur->pL.x * mat.m1 + p_pairsCur->pL.y * mat.m2 + mat.m3;
        p_pairsCur->pR.y = p_pairsCur->pL.x * mat.m4 + p_pairsCur->pL.y * mat.m5 + mat.m6;
    }

    return p_pairs;
}

// draw the link of matched points of two images
void showMatchResult(const cv::Mat& matL, const cv::Mat& matR, const pointPair* p_pairs, unsigned int pairNum)
{
    if (p_pairs == NULL)
    {
        DEBUG_PRINT_DETAILED("NULL input of pointers");
        exit(-1);
    }

    // merge the input images into one image
    cv::Mat_<unsigned char> matArr[2] = {matL, matR};
    cv::Mat_<unsigned char> mergedMat = mergeMats(matArr, 2, horizontal);

    unsigned short wL = matL.cols;

    // draw the lines
    const pointPair* p_pairsEnd = p_pairs + pairNum;
    for (; p_pairs != p_pairsEnd; ++p_pairs)
    {
        cv::line(mergedMat, cv::Point(p_pairs->pL.x, p_pairs->pL.y),
                 cv::Point(p_pairs->pR.x + wL , p_pairs->pR.y), cv::Scalar(255, 0, 0));
    }

    cv::imshow("merged initial images", mergedMat);
}




































