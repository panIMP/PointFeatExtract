#include "imgIO.h"
#include "error.h"
#include <stdio.h>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;


// print image pixel values into a txt file
int printImageVal(const char *fileName, const unsigned char *p_img, unsigned short w, unsigned short h, unsigned short dim)
{
    if (fileName == NULL || p_img == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of file name");
        return -1;
    }

    FILE* fp;
    if ((fp = fopen(fileName, "w")) == NULL)
    {
        DEBUG_PRINT_DETAILED("file open failed in printImageVal\n");
        return -1;
    }

    if (dim == 1)
    {
        for (unsigned short j = 0; j < h; ++j)
        {
            for (unsigned short i = 0; i < w; ++i, ++p_img)
            {
                fprintf(fp, "%-4d", *p_img);
            }
            fprintf(fp, "\n");
        }
    }
    else if (dim > 1)
    {
        for (unsigned short j = 0; j < h; ++j)
        {
            for (unsigned short i = 0; i < w; ++i)
            {
                fprintf(fp, "(%4d,%4d,%4d)", *p_img, *(p_img + 1), *(p_img + 1));
                p_img += 3;
            }
            fprintf(fp, "\n");
        }
    }

    fclose(fp);
    return 0;
}

// get size info a sequence of images
cv::Size getMergeSize(const cv::Mat *p_matArray, unsigned short matNum, Orientation orient)
{
    if (p_matArray == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of mat arrays");
        exit(-1);
    }

    unsigned short t_W, t_H;
    unsigned short sumW = 0;
    unsigned short sumH = 0;
    unsigned short maxW = 0;
    unsigned short maxH = 0;

    for (unsigned short i = 0; i < matNum; ++i)
    {
        t_W = p_matArray[i].cols;
        t_H = p_matArray[i].rows;

        sumW += t_W;
        sumH += t_H;

        maxW = t_W > maxW ? t_W : maxW;
        maxH = t_H > maxH ? t_H : maxH;
    }

    cv::Size size;
    if (orient == vertical)
    {
        size.width = maxW;
        size.height = sumH;
    }
    else if (orient == horizontal)
    {
        size.width = sumW;
        size.height = maxH;
    }

    return size;
}

// combine the images into one single image
cv::Mat mergeMats(const cv::Mat *p_matArr, unsigned short matNum, Orientation orient)
{
    if (p_matArr == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of mat arrays");
        exit(-1);
    }

    cv::Size size = getMergeSize(p_matArr, matNum, orient);
    cv::Mat mergedImg(size, CV_8UC3);

    unsigned short leftTopX = 0;
    unsigned short leftTopY = 0;
    for (unsigned short i = 0; i < matNum; ++i)
    {
        cv::Mat tmpMat = mergedImg(cv::Rect(leftTopX, leftTopY, p_matArr[i].cols, p_matArr[i].rows));
        p_matArr[i].copyTo(tmpMat);

        leftTopX += orient == horizontal ? p_matArr[i].cols : 0;
        leftTopY += orient == vertical ? p_matArr[i].rows : 0;
    }

    return mergedImg;
}

// create integral image of one image
unsigned int *createIntegImg(const unsigned char *p_img, unsigned short w, unsigned short h)
{
    if (p_img == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of image data pointer");
        exit(-1);
    }

    unsigned int* p_integImg = (unsigned int*)calloc_check(w * h, sizeof(unsigned int));

    p_integImg[0] = p_img[0];

    for (unsigned short i = 1; i < w; ++i)
    {
        p_integImg[i] = p_integImg[i-1] + p_img[i];
    }

    for (unsigned short j = 1; j < h; ++j)
    {
        unsigned int sum = 0;
        unsigned int curRow = j * w;
        for (unsigned short i = 0; i < w; ++i)
        {
            sum += p_img[curRow + i];
            p_integImg[curRow + i] = p_integImg[curRow - w + i] + sum;
        }
    }

    return p_integImg;
}

// rotate the image by different angles and scales
void rotateImg(cv::Mat& src, cv::Mat& dst, double angle, double scale)
{
    int h = src.rows * scale;
    int w = src.cols * scale;
    int digonal = sqrt(w * w + h * h);
    int dh = (digonal - h) / 2;
    int dw = (digonal - w) / 2;

    cv::copyMakeBorder(src, dst, dh, dh, dw, dw, cv::BORDER_CONSTANT);

    cv::Point2f center((((double)dst.cols) / 2.0), (((double)dst.rows) / 2.0));

    cv::Mat rotateMat = cv::getRotationMatrix2D(center, angle, scale);

    cv::warpAffine(dst, dst, rotateMat, dst.size());

    double radian = (double)(((double)angle) / 180.0 * CV_PI);
    double sinVal = fabs(sin(radian));
    double cosVal = fabs(cos(radian));

    cv::Size targetSize((int)(w * cosVal + h * sinVal),(int)(w * sinVal + h * cosVal));

    int x = (dst.cols - targetSize.width) / 2;
    int y = (dst.rows - targetSize.height) / 2;

    cv::Rect rect(x, y, targetSize.width, targetSize.height);

    dst = cv::Mat(dst, rect);

//    cv::imwrite("dst.png", dst);
//    dst = cv::imread("dst.png", cv::IMREAD_GRAYSCALE);
}


