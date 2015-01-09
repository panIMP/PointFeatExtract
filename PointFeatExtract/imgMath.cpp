#include "imgMath.h"
#include "Error/error.h"
#include <malloc.h>
#include <math.h>


void sobel(unsigned char* imageData, unsigned short width, unsigned short height)
{
	int i;

	int H, O, V;
	unsigned char  i00, i01, i02;
	unsigned char  i10, i12;
	unsigned char  i20, i21, i22;

	unsigned char* pI00;	unsigned char* pI01;	unsigned char* pI02;
	unsigned char* pI10;							unsigned char* pI12;
	unsigned char* pI20;	unsigned char* pI21;	unsigned char* pI22;

	int fullSize = width * height;
	int tranSize = width * (height - 2) - 2;

	// Bond output
	unsigned char* out = imageData;

	// Copy init data
	unsigned char* in = (unsigned char*)malloc(fullSize * sizeof(unsigned char));
	if (in == NULL) {
		printf("Melmory allocation failed in sobel!");
		exit(1);
	}
	for (i = 0; i < fullSize; ++i) {
		*in++ = *imageData++;
	}
	in = in - fullSize;

	// Init 8-neighbour pointers
	pI00 = in;					pI01 = in + 1;					pI02 = in + 2;
	pI10 = in + width;											pI12 = in + 2 + width;
	pI20 = in + 2 * width;      pI21 = in + 1 + 2 * width;		pI22 = in + 2 + 2 * width;

	// Iterate over entire image as a single, continuous raster line.
	for (i = 0; i < tranSize; ++i)
	{
		// Read in the required 3x3 region from the input.
		i00 = *pI00++;    i01 = *pI01++;    i02 = *pI02++;
		i10 = *pI10++;                      i12 = *pI12++;
		i20 = *pI20++;    i21 = *pI21++;    i22 = *pI22++;


		// Apply horizontal and vertical filter masks.  The final filter
		// output is the sum of the absolute values of these filters.
		H = -i00 - 2 * i01 - i02 +
			+i20 + 2 * i21 + i22;

		V = -i00 + i02
			- 2 * i10 + 2 * i12
			- i20 + i22;

		O = abs(H) + abs(V);

		// Clamp to 8-bit range.  The output is always positive due to
		// the absolute value, so we only need to check for overflow.
		if (O > 255) O = 255;

		// Store it.
		out[i + 1 + width] = O;
	}

	// free intermediate copy image
	free(in);
}

void gaussin(unsigned char* imageData, unsigned short w, unsigned short h)
{
	int i;

	int O;
	unsigned char  i00, i01, i02;
	unsigned char  i10, i11, i12;
	unsigned char  i20, i21, i22;

	unsigned char* pI00;	unsigned char* pI01;	unsigned char* pI02;
	unsigned char* pI10;	unsigned char* pI11;	unsigned char* pI12;
	unsigned char* pI20;	unsigned char* pI21;	unsigned char* pI22;

	int fullSize = w * h;
	int tranSize = w * (h - 2) - 2;

	// Bond output
	unsigned char* out = imageData;

	// Copy init data
	unsigned char* in = (unsigned char*)malloc(fullSize * sizeof(unsigned char));
	if (in == NULL) {
		printf("Melmory allocation failed in sobel!");
		exit(1);
	}
	for (i = 0; i < fullSize; ++i) {
		*in++ = *imageData++;
	}
	in = in - fullSize;

	// Init 8-neighbour pointers
	pI00 = in;				pI01 = in + 1;				pI02 = in + 2;
	pI10 = in + w;			pI11 = in + 1 + w;			pI12 = in + 2 + w;
	pI20 = in + 2 * w;      pI21 = in + 1 + 2 * w;		pI22 = in + 2 + 2 * w;

	// Iterate over entire image as a single, continuous raster line.
	for (i = 0; i < tranSize; ++i)
	{
		// Read in the required 3x3 region from the input.
		i00 = *pI00++;    i01 = *pI01++;    i02 = *pI02++;
		i10 = *pI10++;    i11 = *pI11++;    i12 = *pI12++;
		i20 = *pI20++;    i21 = *pI21++;    i22 = *pI22++;


		// Apply horizontal and vertical filter masks.  The final filter
		// output is the sum of the absolute values of these filters.
		O = i00		+ 2 * i01 + i02 +
			2 * i10 + 4 * i11 + 2 * i12 +
			i20		+ 2 * i21 + i22;

		O = O >> 4;

		// Clamp to 8-bit range.  The output is always positive due to
		// the absolute value, so we only need to check for overflow.
		if (O > 255) O = 255;

		// Store it.
		out[i + 1 + w] = O;
	}

	// free intermediate copy image
	free(in);
}

void mean(unsigned char* imageData, unsigned short width, unsigned short height)
{
	int i;

	short sum;
	unsigned char i00, i01, i02;
	unsigned char i10, i11, i12;
	unsigned char i20, i21, i22;

	int fullSize = width * height;
	int tranSize = width * (height - 2) - 2;

	// Bond output
	unsigned char* out = imageData;

	// Copy init data
	unsigned char* in = (unsigned char*)malloc(fullSize * sizeof(unsigned char));
	if (in == NULL) {
		printf("Melmory allocation failed in sobel!");
		exit(1);
	}
	for (i = 0; i < fullSize; ++i) {
		*in++ = *imageData++;
	}
	in = in - fullSize;

	// Iterate over entire image as a single, continuous raster line.
	for (i = 0; i < tranSize; ++i)
	{
		// Read in the required 3x3 region from the input.
		i00 = in[i]; 					i01 = in[i + 1]; 				i02 = in[i + 2];
		i10 = in[i + width];			i11 = in[i + width + 1];		i12 = in[i + width + 2];
		i20 = in[i + 2 * width]; 		i21 = in[i + 2 * width + 1]; 	i22 = in[i + 2 * width + 2];

		// Calculate the 8-neighbour pixel value sum, include itself
		sum =	i00 + i01 + i02 +
				i10 + i11 + i12 +
				i20 + i21 + i22;

		// Store it.
		out[i + 1 + width] = (double)sum / 9.0 + 0.5;
	}
}

// calculate the combinations of a circular disk
unsigned int calcDiskTmplArray(coord* p_coord, unsigned short r)
{
	unsigned int count = 0;

	unsigned int r2 = r * r;
	double sigma = r / 2;
	double div = - 2 * sigma * sigma;
	double def = 1.0 / sqrt(2.0 * 3.141592653) / sigma;
	unsigned int dist = 0;

	for (int x = -r; x <= r; ++x)
	{
		for (int y = -r; y <= r; ++y)
		{
			dist = x * x + y * y;
			if (dist <= r2)
			{
				p_coord->x = x;
				p_coord->y = y;
				//p_coord->wei = exp((double)dist / div);
				p_coord->wei = 1;

				++p_coord;
				count++;
			}
		}
	}

	return count;
}


unsigned char otsu(unsigned char* p_img, unsigned short w, unsigned short h)
{
	// get histogram
	unsigned int hist[256] = {0};
	unsigned int totalSize = w*h;
	for (unsigned int i = 0; i < totalSize; ++i)
	{
		hist[p_img[i]]++;
	}

	unsigned char startVal = 0;
	while (hist[startVal] == 0)   startVal++;

	unsigned char endVal = 255;
	while (hist[endVal] == 0)   endVal--;

	unsigned int sum = totalSize;
	unsigned long long moment = 0;
	for (unsigned short i = startVal; i <= endVal; i++)
	{
		moment += i * hist[i];
	}

	unsigned int w0 = 0;
	unsigned int w1 = 0;
	unsigned long long tmp = 0;
	double u0, u1;
	double u = (double)moment / (double)sum;
	double equ;
	double max = 0;
	unsigned char thresh;
	for (unsigned short i = startVal; i <= endVal; i++)
	{
		w0 += hist[i];             
		w1 = sum - w0;						 
		tmp += i * hist[i];      
		u0 = (double)tmp / (double)w0;               
		u1 = (double)(moment - tmp) / (double)w1;     

		equ = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u);        

		if (equ > max)
		{
			thresh = i;
			max = equ;
		}
	}

	return thresh;
}


void binary(unsigned char* p_img, unsigned char thresh, unsigned char maxVal, unsigned short w, unsigned short h)
{
	unsigned int totalSize = w * h;
	for (unsigned int i = 0; i < totalSize; ++i)
	{
		if (p_img[i] > thresh)
			p_img[i] = maxVal;
		else
			p_img[i] = 0;
	}
}


void calcPreview(unsigned char* p_img, unsigned char* p_markImg, unsigned char thresh, unsigned short w, unsigned short h)
{
	unsigned int totalSize = w * h;

	unsigned int hist[256] = { 0 };

	unsigned int count = 0;
	for (unsigned int i = 0; i < totalSize; ++i)
	{
		if (p_img[i] > thresh)
		{
			p_markImg[i] = MARKED_TRUE;
			hist[p_img[i]]++;
			count++;
		}
		else
			p_markImg[i] = MARKED_FALSE;
	}

	for (unsigned short i = 1; i < 256; ++i)
	{
		hist[i] = hist[i] + hist[i - 1];
	}

	for (unsigned int i = 0; i < totalSize; ++i)
	{
		if (p_markImg[i] == MARKED_TRUE)
		{
			p_img[i] = (double)hist[p_img[i]] / (double)count * 255.0;
		}
	}
}

void erode(unsigned char* p_img, unsigned char* p_erodeImg, unsigned short w, unsigned short h)
{
	unsigned int totalSize = w * h;
	unsigned int filtSize = w * (h - 1) - 1;

	for (unsigned int i = w + 1; i < filtSize; ++i)
	{
		if (i == 155786)
		{
			i++;
			i--;
		}

		if (p_img[i - w - 1] == 255 && p_img[i - w] == 255 && p_img[i - w + 1] == 255 &&
			p_img[i - 1] == 255 && p_img[i] == 255 && p_img[i + 1] == 255 &&
			p_img[i + w - 1] == 255 && p_img[i + w] == 255 && p_img[i + w + 1] == 255)
		{
			continue;
		}
		p_erodeImg[i] = 0;
	}

}

void subtract(unsigned char* p_img, unsigned char* p_imgSub, unsigned short w, unsigned short h)
{
	unsigned int totalSize = w * h;
	for (unsigned int i = 0; i < totalSize; ++i)
	{
		p_img[i] = (p_img[i] - p_imgSub[i]);
	}
}