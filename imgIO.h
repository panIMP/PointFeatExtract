#ifndef IMAGEIO_H
#define IMAGEIO_H

#include <string>
#include "opencv2/core/core.hpp"

enum Orientation
{
    vertical,
    horizontal,
};

typedef struct SIZE_INFO
{
    unsigned short w;
    unsigned short h;
} SizeInfo;

// print image pixel values into a txt file
int printImageVal(const char* fileName, const unsigned char* p_img, unsigned short w, unsigned short h, unsigned short dim);

// get size info a sequence of images
cv::Size getMergeSize(const cv::Mat* p_matArray, unsigned short matNum, Orientation orient);

// combine the images into one single image
cv::Mat mergeMats(const cv::Mat* p_matArr, unsigned short matNum, Orientation orient);

// create integral image of one image
unsigned int* createIntegImg(const unsigned char* p_img, unsigned short w, unsigned short h);

#endif // IMAGEIO_H
