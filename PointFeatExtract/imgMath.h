#ifndef IMGMATH_H
#define IMGMATH_H

#define UNMARKED 0
#define MARKED_TRUE 1
#define MARKED_FALSE 2

typedef struct COORD
{
    short x;
    short y;
	double wei;
} coord;


// calculate the combinations of a circular disk

void sobel(unsigned char* imageData, unsigned short width, unsigned short height);

void gaussin(unsigned char* imageData, unsigned short width, unsigned short height);

void mean(unsigned char* imageData, unsigned short width, unsigned short height);

// calculate the neighbouring pixel coordinates
unsigned int calcDiskTmplArray(coord* p_coord, unsigned short r);

void canny(unsigned char* p_img, unsigned short w, unsigned short h);

unsigned char otsu(unsigned char* p_img, unsigned short w, unsigned short h);

void binary(unsigned char* p_img, unsigned char thresh, unsigned char maxVal, unsigned short w, unsigned short h);

void otsuBinary(unsigned char* p_img, unsigned short w, unsigned short h);

void otsuBinaryOfRegion(unsigned char* p_img, unsigned short w, unsigned short h, unsigned short wBig);

void localOtsuBinary(unsigned char* p_img, unsigned short w, unsigned short h, int numOfRegion);

void elate(unsigned char* p_img, unsigned char* p_elateImg, unsigned short w, unsigned short h);

void erode(unsigned char* p_img, unsigned char* p_erodeImg, unsigned short w, unsigned short h);

void subtract(unsigned char* p_img, unsigned char* p_imgSub, unsigned short w, unsigned short h);

void calcPreview(unsigned char* p_img, unsigned char* p_markImg, unsigned char thresh, unsigned short w, unsigned short h);

void equHist(unsigned char* p_img, unsigned char* p_markImg, unsigned short w, unsigned short h);

#endif // IMGMATH_H
