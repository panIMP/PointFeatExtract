#ifndef IMGMATH_H
#define IMGMATH_H

#define NOTTOMARKE 0
#define SHOULDMARK 255
#define MARKED_TRUE 2
#define MARKED_FALSE 3

typedef struct COORD
{
    short x;
    short y;
} Coord;

#define MAX_CONTOUR_NUM 1000
typedef struct CONTOUR
{
	Coord* p_coords;
	unsigned int num[MAX_CONTOUR_NUM];
} Contour;


// calculate the combinations of a circular disk

void sobel(unsigned char* imageData, unsigned short width, unsigned short height);

void gaussin(unsigned char* imageData, unsigned short width, unsigned short height);

void mean(unsigned char* imageData, unsigned short width, unsigned short height);

// calculate the neighbouring pixel coordinates
unsigned int calcDiskTmplArray(Coord* p_coord, unsigned short r);

void canny(unsigned char* p_img, unsigned short w, unsigned short h);

unsigned char otsu(unsigned char* p_img, unsigned short w, unsigned short h);

void binary(unsigned char* p_img, unsigned char thresh, unsigned char maxVal, unsigned short w, unsigned short h);

void otsuBinary(unsigned char* p_img, unsigned char maxVal, unsigned short w, unsigned short h);

unsigned char otsuOfRegion(unsigned char* p_img, unsigned short w, unsigned short h, unsigned short wBig);

void otsuBinaryOfRegion(unsigned char* p_img, unsigned char maxVal, unsigned short w, unsigned short h, unsigned short wBig);

void localOtsuBinary(unsigned char* p_img, unsigned char maxVal, unsigned short w, unsigned short h, int numOfRegion);

void localOtsuRecurBinary(unsigned char* p_img, unsigned char maxVal, unsigned short w, unsigned short h, int numOfRegion);

void elate(unsigned char* p_img, unsigned char* p_elateImg, unsigned short w, unsigned short h);

void erode(unsigned char* p_img, unsigned char* p_erodeImg, unsigned short w, unsigned short h);

void subtract(unsigned char* p_img, unsigned char* p_imgSub, unsigned short w, unsigned short h);

void equHist(unsigned char* p_img, unsigned char* p_markImg, unsigned short w, unsigned short h);

void fillRegion(unsigned char* p_img, unsigned short w, unsigned short h);

void seedFillRegion(unsigned char* p_img, unsigned short w, unsigned short h);

int markOutContour(unsigned char* p_img, Contour* p_contours, unsigned short w, unsigned short h);

void markMaxOutContour(unsigned char* p_img, Contour* p_contours, unsigned int contourNum, unsigned short w, unsigned short h);

#endif // IMGMATH_H
