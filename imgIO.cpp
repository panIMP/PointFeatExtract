#include "imgIO.h"
#include <stdio.h>
#include <math.h>


// print image pixel values into a txt file
int printImageVal(const char *fileName, const unsigned char *p_img, unsigned short w, unsigned short h, unsigned short dim)
{
    FILE* fp;
    if ((fp = fopen(fileName, "w")) == NULL)
    {
        printf("file open failed in printImageVal\n");
        return -1;
    }

    if (dim == 1)
    {
        for (unsigned short j = 0; j < h; ++j)
        {
            for (unsigned short i = 0; i < w; ++i, ++p_img)
            {
                fprintf(fp, "%4d", *p_img);
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
cv::Mat mergeImgs(const cv::Mat *p_matArr, unsigned short matNum, Orientation orient)
{
    cv::Size size = getMergeSize(p_matArr, matNum, orient);
    cv::Mat mergedImg(size, CV_8UC1);

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
    unsigned int* p_integImg = (unsigned int*)calloc(w * h, sizeof(unsigned int));

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


