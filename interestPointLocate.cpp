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
void drawRect(cv::Mat& mat, const InterestPoint *p_points, unsigned short pointNum,
              unsigned short step, unsigned short r, cv::Scalar color)
{
    if (p_points == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of image pointer or interest points pointer");
        exit(-1);
    }

    const InterestPoint* p_pointsCur = p_points;
    const InterestPoint* p_pointsEnd = p_points + pointNum;
    for (; p_pointsCur < p_pointsEnd; p_pointsCur+=step)
    {
        cv::Point center(p_pointsCur->c.x, p_pointsCur->c.y);
        cv::circle(mat, center, r, color);
    }
}

// draw the rectangle around the interest point
void drawRect(unsigned char *p_img, const InterestPoint *p_points, unsigned short pointNum,
              unsigned short r, unsigned short w, unsigned char pixVal)
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
                p_img[y * w + x] = pixVal;
            }
        }
    }
}

// init the pointers of the four corners of each box of certain filter
void initFiltPtrs(unsigned int *p_integImg, FiltCornPtrs* p_filtCornPtrss, const Filt* p_filts[], unsigned short w)
{
    if (p_integImg == NULL || p_filtCornPtrss == NULL || p_filts == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of integral image pointer\
                             or filter corner pointers or filter template pointer");
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
void createDetHesImg(unsigned int *p_integImg, double* p_detHesImg, double* p_maxPerLay,
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
    double maxPerLay = FEAT_MIN;
    for (unsigned short y = stY; y < enY; ++y, incFiltPtrs(filtCornPtrss, w))
    {
        t_p_detHesImg = p_detHesImg + y * w + stX;

        for (unsigned short f = 0; f < FILT_NUM; ++f)
            t_filtCornPtrss[f] = filtCornPtrss[f];

        for (unsigned short x = stX; x < enX; ++x, ++t_p_detHesImg, incFiltPtrs(t_filtCornPtrss, 1))
        {
            calcDetHes(t_filtCornPtrss, t_p_detHesImg);

            if (maxPerLay < *t_p_detHesImg)
                maxPerLay = *t_p_detHesImg;
        }
    }

    *p_maxPerLay = maxPerLay;
}

// create the det(Hessin) image pyramid of certain number of octaves and layers
// returns pointers to the image pyramid, first pixel of the image in the first layer of the first octave
MaxDetHesOctave createDetHesImgPyr(double* p_detHesImgPyr, unsigned int *p_integImg,
                                   unsigned short octNum, unsigned short layNum,
                                   unsigned short w, unsigned short h)
{
    if (p_integImg == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of the pointer of integral image");
        exit(-1);
    }

    MaxDetHesOctave maxVal;
    unsigned int totalSize = w * h;

    // create pyramid
    for (unsigned short octOrder = 0; octOrder < octNum; ++octOrder)
    {
        double* t_p_detHesImgPyr = p_detHesImgPyr + octOrder * LAYER_NUM * w * h;
        for (unsigned short layOrder = 0; layOrder < layNum; ++layOrder)
        {
            maxVal.maxPerOct[octOrder].maxPerLay[layOrder] = DETHES_MIN;

            double* p_maxPerLay = &(maxVal.maxPerOct[octOrder].maxPerLay[layOrder]);
            createDetHesImg(p_integImg, t_p_detHesImgPyr, p_maxPerLay, octOrder, layOrder, w, h);

            t_p_detHesImgPyr += totalSize;
        }
    }

    return maxVal;
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
PointNum getPointsLocations(InterestPoint* p_points, unsigned char* p_markImg,
                       const double* p_detHesImgPyr, MaxDetHesOctave maxVal,
                       unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h)
{
    if (p_points == NULL || p_detHesImgPyr == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of interest pointe set pointer or Det(Hessin) pointer");
        exit(-1);
    }

    PointNum num = {0, 0};
    unsigned int totalSize = w * h;
    unsigned int filtSize = w * (h - 1) - 1;
    InterestPoint* p_out = p_points;

    /*for (unsigned short octOrder = 1; octOrder < octNum; ++octOrder)
    {
        for (unsigned short layOrder = 0; layOrder < layNum - 1; ++layOrder)
        {
            double detHesThresh = maxVal.maxPerOct[octOrder].maxPerLay[layOrder] * 0.05;
            const double* p_detHesImg = p_detHesImgPyr + (octOrder * layNum + layOrder) * totalSize;

            for (unsigned int c = w + 1; c < filtSize; ++c, ++p_detHesImg)
            {
                //if (isRegionMaximum(p_detHesImg, w, h))
                if (*p_detHesImg > detHesThresh && isRegionMaximum(p_detHesImg, w, h))
                {
                    // mark the interest points first
                    p_markImg[c] = 1;
                }
            }
        }
    }*/

    unsigned short sequNum = octNum * layNum;
    for (unsigned short sequOrder = 1; sequOrder < sequNum - 1; ++sequOrder)
    {
        unsigned short octOrder = sequOrder / layNum;
        unsigned short layOrder = sequOrder - octOrder * OCTAVE_NUM;
        double detHesThresh = maxVal.maxPerOct[octOrder].maxPerLay[layOrder] * 0.1;
        const double* p_detHesImg = p_detHesImgPyr + sequOrder * totalSize;

        for (unsigned int c = w + 1; c < filtSize; ++c, ++p_detHesImg)
        {
            //if (isRegionMaximum(p_detHesImg, w, h))
            if (*p_detHesImg > detHesThresh && isRegionMaximum(p_detHesImg, w, h))
            {
                // mark the interest points first
                p_markImg[c] = 1;
            }
        }
    }


    // store the pixels into interst point arrays
    unsigned int realNum = 0;
    for (unsigned int i = 0; i < totalSize; ++i)
    {
        if (p_markImg[i] == 1)
        {
            realNum++;
        }
    }
    num.realNum = realNum;

    unsigned short r = sqrt(((double)(w * h)) / (double)(realNum));
    unsigned xEnd = w - r;
    unsigned yEnd = h - r;
    for (unsigned int i = 0; i < totalSize; ++i)
    {
        if (p_markImg[i] == 1)
        {
            unsigned short y = i / w;
            unsigned short x = i - y * w;
            if (y <= r || y >= yEnd || x <= r || x >= xEnd)
            {
                p_markImg[i] = 0;
                continue;
            }

            p_out->c.x = x;
            p_out->c.y = y;
            p_out++;
        }
    }
    num.calcNum = p_out - p_points;

    return num;
}


// calculate the feature of one interest point
void calcFeat(InterestPoint* p_point, const unsigned char* p_img, const unsigned char* p_markImg,
              const coord* p_coordHuMat, const coord* p_coordNeigh,
              int neighPointNumHuMat, int neightPointNumNeigh, unsigned short w)
{
    if (p_point == NULL || p_img == NULL || p_coordHuMat == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of interest pointer set pointer or image pointer or array pointer");
        exit(-1);
    }

    unsigned short xCenter = p_point->c.x;
    unsigned short yCenter = p_point->c.y;

    //Feat 1: hu-mat feat
    const coord* p_coordHuMatEnd = p_coordHuMat + neighPointNumHuMat;
    unsigned long long sum = 0;
    unsigned long long sumX = 0;
    unsigned long long sumY = 0;
    unsigned long long sumX2 = 0;
    unsigned long long sumY2 = 0;
    unsigned long long sumXY = 0;
    unsigned long long sumX3 = 0;
    unsigned long long sumY3 = 0;
    unsigned long long sumX2Y = 0;
    unsigned long long sumXY2 = 0;
    unsigned long long x,y,x2,y2,xy,x3,y3,x2y,xy2,val;
    for (; p_coordHuMat != p_coordHuMatEnd; ++p_coordHuMat)
    {
        x = p_coordHuMat->x + xCenter;
        y = p_coordHuMat->y + yCenter;
        xy = x * y;
        x2 = x * x;
        x3 = x2 * x;
        y2 = y * y;
        y3 = y2 * y;
        x2y = x2 * y;
        xy2 = x * y2;

        val = p_img[y * w + x];

        sum += val;
        sumX += (val * x);
        sumY += (val * y);
        sumX2 += (val * x2);
        sumY2 += (val * y2);
        sumXY += (val * xy) ;
        sumX3 += (val * x3);
        sumY3 += (val * y3);
        sumX2Y += (val * x2y);
        sumXY2 += (val * xy2);
    }
    double xEven = ((double)sumX) / ((double)sum + FEAT_OFFSET);
    double yEven = ((double)sumY) / ((double)sum + FEAT_OFFSET);
    double xEven2 = xEven * xEven;
    double yEven2 = yEven * yEven;
    double xEvenyEvne = xEven * yEven;
    double xEven3 = xEven2 * xEven;
    double yEven3 = yEven2 * yEven;
    double xEven2yEven = xEven2 * yEven;
    double xEvenyEven2 = xEven * yEven2;
    double u00 = sum;
    double u20 = sumX2 - 2 * xEven * sumX + xEven * xEven * u00;
    double u02 = sumY2 - 2 * yEven * sumY + yEven * yEven * u00;
    double u11 = sumXY - xEven * sumY - yEven * sumX + xEven * yEven * u00;
    double u30 = sumX3 - 3 * sumX2 * xEven + 3 * sumX * xEven2 - xEven3 * u00;
    double u03 = sumY3 - 3 * sumY2 * yEven + 3 * sumY * yEven2 - yEven3 * u00;
    double u21 = sumX2Y - 2 * sumXY * xEven + sumY * xEven2 - sumX2 * yEven + 2 * sumX * xEvenyEvne - xEven2yEven * u00;
    double u12 = sumXY2 - 2 * sumXY * yEven + sumX * yEven2 - sumY2 * xEven + 2 * sumY * xEvenyEvne - xEvenyEven2 * u00;

    //Feat 1: hu-mat feat
    const coord* p_coordNeighEnd = p_coordNeigh + neightPointNumNeigh;
    double neigh = 0.0;
    for (; p_coordNeigh != p_coordNeighEnd; ++p_coordNeigh)
    {
        if (p_coordNeigh->x == 0 && p_coordNeigh->y == 0)
        {
            x = 0;
        }
        x = p_coordNeigh->x + xCenter;
        y = p_coordNeigh->y + yCenter;

        if (p_markImg[y * w + x] == 1)
        {
            double dist = p_coordNeigh->x * p_coordNeigh->x + p_coordNeigh->y * p_coordNeigh->y;
            dist = dist / ((double)(w * w));
            neigh += pow(2, -sqrt(dist));
        }
    }


    // =========== Hu ===============
/*    double u00Square = u00 * u00 + FEAT_OFFSET;
    double u0025 = pow(u00, 2.5) + FEAT_OFFSET;
    double n20 = (u20 + FEAT_OFFSET) / u00Square;
    double n02 = (u02 + FEAT_OFFSET) / u00Square;
    double n11 = (u11 + FEAT_OFFSET) / u00Square;
    double n30 = (u30 + FEAT_OFFSET) / u0025;
    double n03 = (u03 + FEAT_OFFSET) / u0025;
    double n21 = (u21 + FEAT_OFFSET) / u0025;
    double n12 = (u12 + FEAT_OFFSET) / u0025;

    double t1 = n30 + n12;
    double t2 = n21 + n03;
    double t3 = n30 - 3 * n12;
    double t4 = 3 * n21 - n03;

    double t12 = pow(t1, 2);
    double t22 = pow(t2, 2);
    double t32 = pow(t3, 2);
    double t42 = pow(t4, 2);

    double m1 = n20 + n02;
    double m2 = pow((n20 - n02), 2) + 4 * n11 * n11;
    double m3 = t32 + t42;
    double m4 = t12 + t22;
    double m5 = t3 * t1 * (t12 - 3 * t22) + t4 * t2 *(3 * t12 - t22);
    double m6 = (n20 - n02) * (t12 - t22) + 4 * n11 * t1 * t2;
    double m7 = t4 * t1 * (t12 - 3 * t22) - t3 * t2 * (3 * t12 - t22);

    p_point->mat.feat[0] = m1;
    p_point->mat.feat[1] = m2;
    p_point->mat.feat[2] = m3;
    p_point->mat.feat[3] = m4;
    p_point->mat.feat[4] = m5;
    p_point->mat.feat[5] = m6;
    p_point->mat.feat[6] = m7;
*/

    // ========== Modified Hu1 ==============
    double u00Square = u00 * u00 + FEAT_OFFSET;
    double u00Cubic = pow(u00, 3) + FEAT_OFFSET;
    double u00div = u00 + FEAT_OFFSET;
    double n20 = (u20 + FEAT_OFFSET) / u00div;
    double n02 = (u02 + FEAT_OFFSET) / u00div;
    double n11 = (u11 + FEAT_OFFSET) / u00div;
    double n30 = (u30 + FEAT_OFFSET) / u00div;
    double n03 = (u03 + FEAT_OFFSET) / u00div;
    double n21 = (u21 + FEAT_OFFSET) / u00div;
    double n12 = (u12 + FEAT_OFFSET) / u00div;

    double m1 = n20 + n02;
    double m2 = n21 + n12;
    double m3 = n30 + n03;
    double m4 = 2 * n11;

    p_point->mat.feat[0] = m1;
    p_point->mat.feat[1] = m2;
    p_point->mat.feat[2] = m3;
    p_point->mat.feat[3] = m4;
//    p_point->mat.feat[4] = neigh;
//    p_point->mat.feat[5] = p_img[yCenter * w + xCenter];


/*
    // ========== Modified Hu2 ==============
    double u00Square = u00 * u00 + FEAT_OFFSET;
    double u0025 = pow(u00, 2.5) + FEAT_OFFSET;
    double n20 = (u20 + FEAT_OFFSET) / u00Square;
    double n02 = (u02 + FEAT_OFFSET) / u00Square;
    double n11 = (u11 + FEAT_OFFSET) / u00Square;
    double n30 = (u30 + FEAT_OFFSET) / u0025;
    double n03 = (u03 + FEAT_OFFSET) / u0025;
    double n21 = (u21 + FEAT_OFFSET) / u0025;
    double n12 = (u12 + FEAT_OFFSET) / u0025;

    double t1 = n30 + n12;
    double t2 = n21 + n03;
    double t3 = n30 - 3 * n12;
    double t4 = 3 * n21 - n03;

    double t12 = pow(t1, 2);
    double t22 = pow(t2, 2);
    double t32 = pow(t3, 2);
    double t42 = pow(t4, 2);

    double m1 = n20 + n02;
    double m2 = pow((n20 - n02), 2) + 4 * n11 * n11;
    double m3 = t32 + t42;
    double m4 = t12 + t22;
    double m5 = t3 * t1 * (t12 - 3 * t22) + t4 * t2 *(3 * t12 - t22);
    double m6 = (n20 - n02) * (t12 - t22) + 4 * n11 * t1 * t2;
    double m7 = t4 * t1 * (t12 - 3 * t22) - t3 * t2 * (3 * t12 - t22);

    double f1 = sqrt(m2) / (m1 + FEAT_OFFSET);
    double f2 = sqrt(m4 / (m3 + FEAT_OFFSET));
    double f3 = m3 * u00 / (m1 * m2 + FEAT_OFFSET);
    double f4 = m3 / (m1 * m1 * m1 + FEAT_OFFSET);
    double f5 = m4 / (m1 * m1 * m1 + FEAT_OFFSET);

    p_point->mat.feat[0] = f1;
    p_point->mat.feat[1] = f2;
    p_point->mat.feat[2] = f3;
    p_point->mat.feat[3] = f4;
    p_point->mat.feat[4] = f5;
*/


    // ========== Modified Hu3 ==============
/*    double u00Square = u00 * u00 + FEAT_OFFSET;
    double u0025 = pow(u00, 2.5) + FEAT_OFFSET;
    double n20 = (u20 + FEAT_OFFSET) / u00Square;
    double n02 = (u02 + FEAT_OFFSET) / u00Square;
    double n11 = (u11 + FEAT_OFFSET) / u00Square;
    double n30 = (u30 + FEAT_OFFSET) / u0025;
    double n03 = (u03 + FEAT_OFFSET) / u0025;
    double n21 = (u21 + FEAT_OFFSET) / u0025;
    double n12 = (u12 + FEAT_OFFSET) / u0025;

    double t1 = n30 + n12;
    double t2 = n21 + n03;
    double t3 = n30 - 3 * n12;
    double t4 = 3 * n21 - n03;

    double t12 = pow(t1, 2);
    double t22 = pow(t2, 2);
    double t32 = pow(t3, 2);
    double t42 = pow(t4, 2);

    double m1 = n20 + n02;
    double m2 = pow((n20 - n02), 2) + 4 * n11 * n11;
    double m3 = t32 + t42;
    double m4 = t12 + t22;
    double m5 = t3 * t1 * (t12 - 3 * t22) + t4 * t2 *(3 * t12 - t22);
    double m6 = (n20 - n02) * (t12 - t22) + 4 * n11 * t1 * t2;
    double m7 = t4 * t1 * (t12 - 3 * t22) - t3 * t2 * (3 * t12 - t22);

    double f1 = m4 / m3;
    double f2 = m7 / m5;
    double f3 = m5 / (m4 * m3);

    p_point->mat.feat[0] = f1;
    p_point->mat.feat[1] = f2;
    p_point->mat.feat[2] = f3;*/

}

// calculate the features of all the located interest points
void getPointsFeats(InterestPoint* p_points, const unsigned char *p_markImg, unsigned int pointNum,
                    const unsigned char *p_img, unsigned short w, unsigned short r)
{
    if (p_points == NULL || p_img == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of points set pointer");
        exit(-1);
    }

    unsigned int neighPointNumHuMat = 0;
    coord* p_coordHuMat = calcDiskTmplArray(r, &neighPointNumHuMat);
    unsigned int neighPointNumNeigh = 0;
    coord* p_coordNeigh = calcDiskTmplArray(r, &neighPointNumNeigh);

    InterestPoint* p_pointsEnd = p_points + pointNum;
    InterestPoint* p_pointsCur = p_points;

    // get the points feat
    unsigned int curPointSeq = 1;
    for (; p_pointsCur != p_pointsEnd; ++p_pointsCur, ++curPointSeq)
    {
        calcFeat(p_pointsCur, p_img, p_markImg, p_coordHuMat, p_coordNeigh, neighPointNumHuMat, neighPointNumNeigh, w);

        DEBUG_PRINT_SIMPLIFIED("Point%4d: (%4d, %4d), Feat: (", curPointSeq, p_pointsCur->c.x, p_pointsCur->c.y);
        for (int f = 0; f < FEAT_NUM; ++f)
        {
            DEBUG_PRINT_SIMPLIFIED("%lf ", p_pointsCur->mat.feat[f]);
        }
        DEBUG_PRINT_SIMPLIFIED(")\n");
    }

    free(p_coordHuMat);
    free(p_coordNeigh);
    p_coordHuMat = NULL;
}

// locate the interest points through the det(Hessin) image pyramid
// returns the pointer to the head of the array of interest points
InterestPoint *getInterestPoints(unsigned int* p_pointNum, const unsigned char *p_img,
                                 const double* p_detHesImgPyr, MaxDetHesOctave maxVal,
                                 unsigned short octNum, unsigned short layNum, unsigned short w, unsigned short h)
{
    if (p_pointNum == NULL || p_img == NULL || p_detHesImgPyr == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    InterestPoint* p_points = (InterestPoint*)calloc_check(w * h, sizeof(InterestPoint));
    unsigned char* p_markImg = (unsigned char*)calloc_check(w * h, sizeof(unsigned char));

    // step 1: get the interest points
    PointNum num;
    num = getPointsLocations(p_points, p_markImg, p_detHesImgPyr, maxVal, octNum, layNum, w, h);
    *p_pointNum = num.calcNum;

    // step 2: calculate the feature of the interest points
    clock_t start = clock();
    //getPointsFeats(p_points, p_markImg, num.calcNum, p_img, w, sqrt(((double)(w * h)) / (num.realNum)));
    getPointsFeats(p_points, p_markImg, num.calcNum, p_img, w, 15);
    clock_t end1 = clock();
    printf("feat time consume: %ld\n", (end1 - start));

    // free memory
    free(p_markImg);

    return p_points;
}

// calculate the mahala distance of two feature
// returns the feature distance
double calcFeatDistance(const InterestPoint *p_pointL, const InterestPoint *p_pointR)
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

//    cv::Mat_<double> matL(FEAT_NUM, 1);
//    for (int f = 0; f < FEAT_NUM; ++f)
//    {
//        matL.at<double>(f, 1) = p_pointL->mat.feat[f];
//    }

//    cv::Mat_<double> matR(FEAT_NUM, 1);
//    for (int f = 0; f < FEAT_NUM; ++f)
//    {
//        matR.at<double>(f, 1) = p_pointR->mat.feat[f];
//    }

    return dist;
}


// get the most similar(nearest) point of current point
const InterestPoint* getNearestPoint(const InterestPoint *p_pointCur, const InterestPoint *p_pointsRef, unsigned int pointNumRef)
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

    const InterestPoint* p_pointsRefNearest = NULL;
    const InterestPoint* p_pointsRefEnd = p_pointsRef + pointNumRef;
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

// normalize all the feats
void normalizePointsFeats(InterestPoint* p_pointsL, unsigned int pointNumL, InterestPoint* p_pointsR, unsigned int pointNumR)
{
    // get the max and min value of different channel of feats
    featElemType* p_featMax = (featElemType*)calloc_check(FEAT_NUM, sizeof(featElemType));
    featElemType* p_featMin = (featElemType*)calloc_check(FEAT_NUM, sizeof(featElemType));
    featElemType* p_featGap = (featElemType*)calloc_check(FEAT_NUM, sizeof(featElemType));

    featElemType* p_featMaxCur = p_featMax;
    featElemType* p_featMinCur = p_featMin;
    const featElemType* p_featMinEnd = p_featMin + FEAT_NUM;
    for (; p_featMinCur != p_featMinEnd; ++p_featMaxCur, ++p_featMinCur)
    {
        *p_featMaxCur = FEAT_MIN;
        *p_featMinCur = FEAT_MAX;
    }

    InterestPoint* p_pointsLCur = p_pointsL;
    const InterestPoint* p_pointsLEnd = p_pointsL + pointNumL;
    InterestPoint* p_pointsRCur = p_pointsR;
    const InterestPoint* p_pointsREnd = p_pointsR + pointNumR;

    // get the max and min value of different channel of feats
    unsigned int f = 0;
    p_featMaxCur = p_featMax;
    p_featMinCur = p_featMin;
    featElemType* p_featGapCur = p_featGap;
    for (; p_featMinCur != p_featMinEnd; ++p_featMaxCur, ++p_featMinCur, ++p_featGapCur, ++f)
    {
        for (p_pointsLCur = p_pointsL; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur)
        {
            featElemType val = p_pointsLCur->mat.feat[f];

            if (*p_featMaxCur < val)
                *p_featMaxCur = val;
            if (*p_featMinCur > val)
                *p_featMinCur = val;
        }

        for (p_pointsRCur = p_pointsR; p_pointsRCur != p_pointsREnd; ++p_pointsRCur)
        {
            featElemType val = p_pointsRCur->mat.feat[f];

            if (*p_featMaxCur < val)
                *p_featMaxCur = val;
            if (*p_featMinCur > val)
                *p_featMinCur = val;
        }

        *p_featGapCur = *p_featMaxCur - *p_featMinCur;
    }

    // normalize the feats
    unsigned int curPointSeq = 1;
    for (curPointSeq = 1, p_pointsLCur = p_pointsL; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur, ++curPointSeq)
    {
        p_featMinCur = p_featMin;
        p_featGapCur = p_featGap;
        for (f = 0; p_featMinCur != p_featMinEnd; ++p_featMinCur, ++p_featGapCur, ++f)
        {
            p_pointsLCur->mat.feat[f] = (p_pointsLCur->mat.feat[f] - *p_featMinCur) / (*p_featGapCur);
        }

        DEBUG_PRINT_SIMPLIFIED("L Point%4d: (%4d, %4d), Feat: (", curPointSeq, p_pointsLCur->c.x, p_pointsLCur->c.y);
        for (int f = 0; f < FEAT_NUM; ++f)
        {
            DEBUG_PRINT_SIMPLIFIED("%lf ", p_pointsLCur->mat.feat[f]);
        }
        DEBUG_PRINT_SIMPLIFIED(")\n");
    }
    for (curPointSeq = 1, p_pointsRCur = p_pointsR; p_pointsRCur != p_pointsREnd; ++p_pointsRCur, ++curPointSeq)
    {
        p_featMinCur = p_featMin;
        p_featGapCur = p_featGap;
        for (f = 0; p_featMinCur != p_featMinEnd; ++p_featMinCur, ++p_featGapCur, ++f)
        {
            p_pointsRCur->mat.feat[f] = (p_pointsRCur->mat.feat[f] - *p_featMinCur) / (*p_featGapCur);
        }

        DEBUG_PRINT_SIMPLIFIED("R Point%4d: (%4d, %4d), Feat: (", curPointSeq, p_pointsRCur->c.x, p_pointsRCur->c.y);
        for (int f = 0; f < FEAT_NUM; ++f)
        {
            DEBUG_PRINT_SIMPLIFIED("%lf ", p_pointsRCur->mat.feat[f]);
        }
        DEBUG_PRINT_SIMPLIFIED(")\n");
    }

}

// rough match based on mutual-minimum-distance
// returns the matched pair number
int roughMatch(const InterestPoint* p_pointsL, unsigned int pointNumL,
               const InterestPoint* p_pointsR, unsigned int pointNumR, PointPair* p_pairs)
{
    if (p_pointsL == NULL || p_pointsR == NULL || p_pairs == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    PointPair* p_pairsCur = p_pairs;

    const InterestPoint* p_pointsLCur = p_pointsL;
    const InterestPoint* p_pointsLStart = p_pointsL;
    const InterestPoint* p_pointsLEnd = p_pointsL + pointNumL;

    const InterestPoint* p_pointsRStart = p_pointsR;

    for (; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur)
    {
        const InterestPoint* p_matchedR = getNearestPoint(p_pointsLCur, p_pointsRStart, pointNumR);
        if (p_matchedR == NULL)
        {
            DEBUG_PRINT_DETAILED("Left image Point %ld has no nearest Right image point", p_pointsLCur - p_pointsLStart + 1);
        }
        else
        {
            const InterestPoint* p_matchedL = getNearestPoint(p_matchedR, p_pointsLStart, pointNumL);
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
ProjectMat getProjMatByRansac(const PointPair *p_pairs, unsigned int pairNum,
                              unsigned short wL, unsigned short hL, unsigned short wR, unsigned short hR)
{
    if (p_pairs == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    // threshold for distance between expectedly matched point position and actually matched point position
    // > distThresh -- inner point
    // < distThresh -- outer point
    double distThresh = min((double)(wL * hL), double(wR * hR)) / (double)(pairNum);
    //double distThresh = 50 * 50;

    // threshold for ending the iteration
    // > innerPointNumThresh -- correct enough projection matrix coefficiency has been found
    // < innerPointNumThresh -- not yet
    unsigned int maxInnerPointNum = 0;

    // projection matrix coefficiency
    ProjectMat curMat, suitMat;

    unsigned int iterateNum = 100 * pairNum;
    cv::Point2f srcTri[3];
    cv::Point2f dstTri[3];
    cv::Mat mat(2, 3, CV_32FC1);

    for (unsigned int i = 0; i < iterateNum; ++i)
    {
        unsigned int start1 = rand() % pairNum;
        unsigned int start2 = rand() % pairNum;
        unsigned int start3 = rand() % pairNum;
        const PointPair* p_pair1 = p_pairs + start1;
        const PointPair* p_pair2 = p_pairs + start2;
        const PointPair* p_pair3 = p_pairs + start3;

        srcTri[0] = cv::Point2f(p_pair1->pL.x, p_pair1->pL.y);
        srcTri[1] = cv::Point2f(p_pair2->pL.x, p_pair2->pL.y);
        srcTri[2] = cv::Point2f(p_pair3->pL.x, p_pair3->pL.y);

        dstTri[0] = cv::Point2f(p_pair1->pR.x, p_pair1->pR.y);
        dstTri[1] = cv::Point2f(p_pair2->pR.x, p_pair2->pR.y);
        dstTri[2] = cv::Point2f(p_pair3->pR.x, p_pair3->pR.y);

        mat = cv::getAffineTransform(srcTri, dstTri);
        curMat.m1 = mat.at<double>(0, 0);
        curMat.m2 = mat.at<double>(0, 1);
        curMat.m3 = mat.at<double>(0, 2);
        curMat.m4 = mat.at<double>(1, 0);
        curMat.m5 = mat.at<double>(1, 1);
        curMat.m6 = mat.at<double>(1, 2);

        // calculate the inner point number under current coefficients
        const PointPair* p_pairCur = p_pairs;
        const PointPair* p_pairEnd = p_pairs + pairNum;
        unsigned int innerPointNum = 0;

        for (; p_pairCur != p_pairEnd; ++p_pairCur)
        {
            double expectedRx = curMat.m1 * p_pairCur->pL.x + curMat.m2 * p_pairCur->pL.y + curMat.m3;
            double expectedRy = curMat.m4 * p_pairCur->pL.x + curMat.m5 * p_pairCur->pL.y + curMat.m6;
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
            suitMat = curMat;
        }
    }

    return suitMat;
}

// match the interest points of two images
// returns the matched pairs of interest points
PointPair *matchInterestPoints(InterestPoint *p_pointsL, int pointNumL,
                               InterestPoint *p_pointsR, int pointNumR, unsigned int* p_pairNum,
                               unsigned short wL, unsigned short hL, unsigned short wR, unsigned short hR)
{
    if (p_pointsL == NULL || p_pointsR == NULL || p_pairNum == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointers");
        exit(-1);
    }

    PointPair* p_pairs = (PointPair*)calloc_check(max(pointNumL, pointNumR), sizeof(PointPair));

    // step 0: normalize all these feats
    normalizePointsFeats(p_pointsL, pointNumL, p_pointsR, pointNumR);

    // step 1: rough match based on mutual-minimum-distance
    *p_pairNum = roughMatch(p_pointsL, pointNumL, p_pointsR, pointNumR, p_pairs);

//    // step 2: get the projection match based on ransac
    ProjectMat mat = getProjMatByRansac(p_pairs, *p_pairNum, wL, hL, wR, hR);

    // step 3: rectify the match based on ransac
    PointPair* p_pairsEnd = p_pairs + *p_pairNum;
    PointPair* p_pairsCur = p_pairs;
    for (; p_pairsCur != p_pairsEnd; ++p_pairsCur)
    {
        p_pairsCur->pR.x = p_pairsCur->pL.x * mat.m1 + p_pairsCur->pL.y * mat.m2 + mat.m3;
        p_pairsCur->pR.y = p_pairsCur->pL.x * mat.m4 + p_pairsCur->pL.y * mat.m5 + mat.m6;
    }

    return p_pairs;
}

// draw the link of matched points of two images
void showMatchResult(const cv::Mat& matL, const cv::Mat& matR, const PointPair* p_pairs, unsigned int pairNum, string testName)
{
    if (p_pairs == NULL)
    {
        DEBUG_PRINT_DETAILED("NULL input of pointers");
        exit(-1);
    }

    // merge the input images into one image
    cv::Mat_<cv::Vec3b> matArr[2] = {matL, matR};
    cv::Mat_<cv::Vec3b> mergedMat = mergeMats(matArr, 2, horizontal);

    unsigned short wL = matL.cols;

    // draw the lines
    const PointPair* p_pairsEnd = p_pairs + (pairNum / 3) * 3;
    const PointPair* p_pairsCur = p_pairs;
    for (; p_pairsCur < p_pairsEnd; )
    {
        cv::line(mergedMat, cv::Point(p_pairsCur->pL.x, p_pairsCur->pL.y),
                 cv::Point(p_pairsCur->pR.x + wL , p_pairsCur->pR.y), cv::Scalar(255, 0, 0));
        p_pairsCur++;

        cv::line(mergedMat, cv::Point(p_pairsCur->pL.x, p_pairsCur->pL.y),
                 cv::Point(p_pairsCur->pR.x + wL , p_pairsCur->pR.y), cv::Scalar(0, 255, 0));
        p_pairsCur++;

        cv::line(mergedMat, cv::Point(p_pairsCur->pL.x, p_pairsCur->pL.y),
                 cv::Point(p_pairsCur->pR.x + wL , p_pairsCur->pR.y), cv::Scalar(0, 0, 255));
        p_pairsCur++;
    }

    cv::imshow("merged initial images", mergedMat);
    //cv::imwrite(testName + string(".png"), mergedMat);
}




































