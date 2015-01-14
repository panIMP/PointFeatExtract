#include "imgMath.h"
#include "Error/error.h"
#include <malloc.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

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
	int* out =(int*)malloc_check(fullSize * sizeof(int));
	int maxVal = 0;

	// Copy init data
	unsigned char* in = imageData;

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
		if (O > maxVal)
			maxVal = O;

		// Store it.
		out[i + 1 + width] = O;
	}

	for (i = 0; i < fullSize; ++i)
	{
		out[i] = (out[i] * 255) / (double)maxVal;
	}

	for (i = 0; i < fullSize; ++i)
	{
		in[i] = out[i];
	}


	// free intermediate copy image
	free(out);
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
unsigned int calcDiskTmplArray(Coord* p_coord, unsigned short r)
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

				++p_coord;
				count++;
			}
		}
	}

	return count;
}

void setBoundaryZero(unsigned char* p_img, unsigned short w, unsigned short h)
{
	for (int x = 0; x < w; ++x)
	{
		p_img[x] = 0;
		p_img[(h - 1) * w + x] = 0;
	}

	for (int y = 0; y < h; ++y)
	{
		p_img[y * w] = 0;
		p_img[y * w + w - 1] = 0;
	}
}

unsigned char otsu(unsigned char* p_img, unsigned short w, unsigned short h)
{
	// get histogram
	unsigned int hist[256] = { 0 };

	unsigned int sum = w * h;
	unsigned int sum0 = 0;
	unsigned int sum1 = 0;

	unsigned long long moment = 0;
	unsigned long long moment0 = 0;
	unsigned long long moment1 = 0;

	double u0, u1, u;
	double interClassInvar;
	double maxInterClassInvar = 0;

	unsigned char thresh, startVal, endVal;

	for (unsigned int i = 0; i < sum; ++i)
	{
		hist[p_img[i]]++;
	}

	startVal = 0;
	while (hist[startVal] == 0)   startVal++;

	endVal = 255;
	while (hist[endVal] == 0)   endVal--;


	for (unsigned short i = startVal; i <= endVal; i++)
	{
		moment += i * hist[i];
	}

	u = (double)moment / (double)sum;

	for (unsigned short i = startVal; i <= endVal; i++)
	{
		sum0 += hist[i];
		sum1 = sum - sum0;
		moment0 += i * hist[i];
		moment1 = moment - moment0;

		u0 = (double)moment0 / (double)sum0;
		u1 = (double)moment1 / (double)sum1;

		interClassInvar = sum0 * (u0 - u) * (u0 - u) + sum1 * (u1 - u) * (u1 - u);

		if (interClassInvar > maxInterClassInvar)
		{
			thresh = i;
			maxInterClassInvar = interClassInvar;
		}
	}

	return thresh;
}


unsigned char threshByFirstVally(unsigned char* p_img, unsigned short w, unsigned short h)
{
	unsigned int i = 0;
	unsigned int sum = w * h;
	unsigned char dixVal = 0;
	unsigned int maxFreq = 0;

	// get histogram
	unsigned int hist[256] = { 0 };

	for (i = 0; i < sum; ++i)
	{
		hist[p_img[i]] ++;
	}

	for (i = 0; i < 255; ++i)
	{
		if (hist[i] > maxFreq)
		{
			maxFreq = hist[i];
			dixVal = i;
		}
	}

	i = dixVal;
	while (hist[i + 1] < hist[i] && i < 256)
	{
		i++;
	}

	return i;
}


unsigned char threshByTriangularFoot(unsigned char* p_img, unsigned short w, unsigned short h)
{
	unsigned int i = 0;
	unsigned int sum = w * h;
	unsigned char dixVal = 0;
	unsigned int maxFreq = 0;
	unsigned char a = 0;
	unsigned char b = 0;
	unsigned char thresh = 0;

	// get histogram
	unsigned int hist[256] = { 0 };
	
	for (i = 0; i < sum; ++i)
	{
		hist[p_img[i]] ++;
	}
	
	for (i = 0; i < 255; ++i)
	{
		if (hist[i] > maxFreq)
		{
			maxFreq = hist[i];
			dixVal = i;
		}
	}

	a = maxFreq;
	b = 255 - dixVal;
	thresh = (unsigned int)(a * a * b) / (unsigned int)(a * a + b * b);
	thresh += dixVal;
	
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


void otsuBinary(unsigned char* p_img, unsigned char maxVal, unsigned short w, unsigned short h)
{
	// get histogram
	unsigned int hist[256] = { 0 };

	unsigned int sum = w * h;
	unsigned int sum0 = 0;
	unsigned int sum1 = 0;

	unsigned long long moment = 0;
	unsigned long long moment0 = 0;
	unsigned long long moment1 = 0;

	double u0, u1, u;
	double interClassInvar;
	double maxInterClassInvar = 0;

	unsigned char thresh, startVal, endVal;

	for (unsigned int i = 0; i < sum; ++i)
	{
		hist[p_img[i]]++;
	}

	startVal = 0;
	while (hist[startVal] == 0)   startVal++;

	endVal = 255;
	while (hist[endVal] == 0)   endVal--;


	for (unsigned short i = startVal; i <= endVal; i++)
	{
		moment += i * hist[i];
	}

	u = (double)moment / (double)sum;

	for (unsigned short i = startVal; i <= endVal; i++)
	{
		sum0 += hist[i];
		sum1 = sum - sum0;
		moment0 += i * hist[i];
		moment1 = moment - moment0;

		u0 = (double)moment0 / (double)sum0;
		u1 = (double)moment1 / (double)sum1;

		interClassInvar = sum0 * (u0 - u) * (u0 - u) + sum1 * (u1 - u) * (u1 - u);

		if (interClassInvar > maxInterClassInvar)
		{
			thresh = i;
			maxInterClassInvar = interClassInvar;
		}
	}

	for (unsigned int i = 0; i < sum; ++i)
	{
		if (p_img[i] > thresh)
			p_img[i] = maxVal;
		else
			p_img[i] = 0;
	}
}


void otsuBinaryOfRegion(unsigned char* p_img, unsigned char maxVal, unsigned short w, unsigned short h, unsigned short wBig)
{
	// get histogram
	unsigned int hist[256] = { 0 };

	unsigned int sum = w * h;
	unsigned int sum0 = 0;
	unsigned int sum1 = 0;

	unsigned long long moment = 0;
	unsigned long long moment0 = 0;
	unsigned long long moment1 = 0;

	double u0, u1, u;
	double interClassInvar;
	double maxInterClassInvar = 0;

	unsigned char thresh = 0;
	unsigned char startVal = 0;
	unsigned char endVal = 0;

	unsigned int i = 0;
	unsigned int j = 0;

	for (unsigned int j = 0; j < h; ++j)
	{
		for (unsigned int i = 0; i < w; ++i)
		{
			hist[p_img[j * wBig + i]]++;
		}
	}

	startVal = 0;
	while (hist[startVal] == 0)   startVal++;

	endVal = 255;
	while (hist[endVal] == 0)   endVal--;

	for (i = startVal; i <= endVal; i++)
	{
		moment += i * hist[i];
	}

	u = (double)moment / (double)sum;

	for (i = startVal; i <= endVal; i++)
	{
		sum0 += hist[i];
		sum1 = sum - sum0;
		moment0 += i * hist[i];
		moment1 = moment - moment0;

		u0 = (double)moment0 / (double)sum0;
		u1 = (double)moment1 / (double)sum1;

		interClassInvar = sum0 * (u0 - u) * (u0 - u) + sum1 * (u1 - u) * (u1 - u);

		if (interClassInvar > maxInterClassInvar)
		{
			thresh = i;
			maxInterClassInvar = interClassInvar;
		}
	}

	for (j = 0; j < h; ++j)
	{
		for (i = 0; i < w; ++i)
		{
			if (p_img[j * wBig + i] > thresh)
				p_img[j * wBig + i] = maxVal;
			else
				p_img[j * wBig + i] = 0;
		}
	}
}


unsigned char otsuOfRegion(unsigned char* p_img, unsigned short w, unsigned short h, unsigned short wBig)
{
	// get histogram
	unsigned int hist[256] = { 0 };

	unsigned int sum = w * h;
	unsigned int sum0 = 0;
	unsigned int sum1 = 0;

	unsigned long long moment = 0;
	unsigned long long moment0 = 0;
	unsigned long long moment1 = 0;

	double u0, u1, u;
	double interClassInvar;
	double maxInterClassInvar = 0;

	unsigned int i = 0;

	unsigned char thresh, startVal, endVal;

	for (unsigned int j = 0; j < h; ++j)
	{
		for (unsigned int i = 0; i < w; ++i)
		{
			hist[p_img[j * wBig + i]]++;
		}
	}

	startVal = 0;
	while (hist[startVal] == 0)   startVal++;

	endVal = 255;
	while (hist[endVal] == 0)   endVal--;

	for (i = startVal; i <= endVal; i++)
	{
		moment += i * hist[i];
	}

	u = (double)moment / (double)sum;

	for (i = startVal; i <= endVal; i++)
	{
		sum0 += hist[i];
		sum1 = sum - sum0;
		moment0 += i * hist[i];
		moment1 = moment - moment0;

		u0 = (double)moment0 / (double)sum0;
		u1 = (double)moment1 / (double)sum1;

		interClassInvar = sum0 * (u0 - u) * (u0 - u) + sum1 * (u1 - u) * (u1 - u);

		if (interClassInvar > maxInterClassInvar)
		{
			thresh = i;
			maxInterClassInvar = interClassInvar;
		}
	}

	return thresh;
}


void localOtsuRecurBinary(unsigned char* p_img, unsigned char maxVal, unsigned short w, unsigned short h, int numOfRegion)
{
	int i = 0;
	int div = sqrt((double)numOfRegion);

	unsigned short wDiv = w / div;
	unsigned short hDiv = h / div;
	unsigned short gapNumOfX = 0;
	unsigned short gapNumOfY = 0;
	unsigned short x = 0;
	unsigned short y = 0;

	unsigned int sum = w * h;

	unsigned int histOfThresh[256] = { 0 };

	unsigned int numOfRegion0 = 0;
	unsigned int numOfRegion1 = 0;

	unsigned long long moment = 0;
	unsigned long long moment0 = 0;
	unsigned long long moment1 = 0;

	double u0, u1, u;
	double interClassInvar;
	double maxInterClassInvar = 0;

	unsigned char thresh = 0;
	unsigned char startVal = 0; 
	unsigned char endVal = 0;

	for (gapNumOfX = 0; gapNumOfX < div; ++gapNumOfX)
	{
		for (gapNumOfY = 0; gapNumOfY < div; ++gapNumOfY)
		{
			histOfThresh[otsuOfRegion(p_img + gapNumOfX * wDiv + gapNumOfY * hDiv * w, wDiv, hDiv, w)] ++;
		}
	}

	startVal = 0;
	while (histOfThresh[startVal] == 0)   startVal++;

	endVal = 255;
	while (histOfThresh[endVal] == 0)   endVal--;


	if (numOfRegion == 1)
	{
		thresh = startVal;
	}
	else
	{
		for (i = startVal; i <= endVal; i++)
		{
			moment += i * histOfThresh[i];
		}

		u = (double)moment / (double)numOfRegion;

		for (i = startVal; i <= endVal; i++)
		{
			numOfRegion0 += histOfThresh[i];
			numOfRegion1 = numOfRegion - numOfRegion0;
			moment0 += i * histOfThresh[i];
			moment1 = moment - moment0;

			u0 = (double)moment0 / (double)numOfRegion0;
			u1 = (double)moment1 / (double)numOfRegion1;

			interClassInvar = numOfRegion0 * (u0 - u) * (u0 - u) + numOfRegion1 * (u1 - u) * (u1 - u);

			if (interClassInvar > maxInterClassInvar)
			{
				thresh = i;
				maxInterClassInvar = interClassInvar;
			}
		}
	}

	//thresh += 1;
	for (i = 0; i < sum; ++i)
	{
		if (p_img[i] > thresh)
			p_img[i] = maxVal;
		else
			p_img[i] = 0;
	}
}


void localOtsuBinary(unsigned char* p_img, unsigned char maxVal, unsigned short w, unsigned short h, int numOfRegion)
{
	int i = 0;
	int div = sqrt((double)numOfRegion);

	unsigned short wDiv = w / div;
	unsigned short hDiv = h / div;
	unsigned short gapNumOfX = 0;
	unsigned short gapNumOfY = 0;
	unsigned short x = 0;
	unsigned short y = 0;

	unsigned int sum = w * h;

	unsigned int histOfThresh[256] = { 0 };

	unsigned int numOfRegion0 = 0;
	unsigned int numOfRegion1 = 0;

	unsigned long long moment = 0;
	unsigned long long moment0 = 0;
	unsigned long long moment1 = 0;

	double u0, u1, u;
	double interClassInvar;
	double maxInterClassInvar = 0;

	unsigned char thresh = 0;
	unsigned char startVal = 0;
	unsigned char endVal = 0;

	for (gapNumOfX = 0; gapNumOfX < div; ++gapNumOfX)
	{
		for (gapNumOfY = 0; gapNumOfY < div; ++gapNumOfY)
		{
			otsuBinaryOfRegion(p_img + gapNumOfX * wDiv + gapNumOfY * hDiv * w, maxVal, wDiv, hDiv, w);
		}
	}
}


void elate(unsigned char* p_img, unsigned char* p_elateImg, unsigned short w, unsigned short h)
{
	unsigned int totalSize = w * h;
	unsigned int filtSize = w * (h - 1) - 1;

	for (unsigned int i = w + 1; i < filtSize; ++i)
	{
		if (p_img[i - w - 1] == 255 || p_img[i - w] == 255 || p_img[i - w + 1] == 255 ||
			p_img[i - 1] == 255 || p_img[i] == 255 || p_img[i + 1] == 255 ||
			p_img[i + w - 1] == 255 || p_img[i + w] == 255 || p_img[i + w + 1] == 255)
		{
			p_elateImg[i] = 255;
		}
		else
			p_elateImg[i] = 0;
	}

}


void erode(unsigned char* p_img, unsigned char* p_erodeImg, unsigned short w, unsigned short h)
{
	unsigned int totalSize = w * h;
	unsigned int filtSize = w * (h - 1) - 1;

	for (unsigned int i = w + 1; i < filtSize; ++i)
	{
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


void equHist(unsigned char* p_img, unsigned char* p_markImg, unsigned short w, unsigned short h)
{
	int i = 0;
	int sum = w * h;
	int regionSum = 0;

	unsigned int hist[256] = { 0 };

	for (i = 0; i < sum; ++i)
	{
		if (p_markImg[i] == 0)
			continue;

		hist[p_img[i]]++;
		regionSum++;
	}

	for (i = 1; i < 256; ++i)
	{
		hist[i] += hist[i - 1];
	}

	for (i = 0; i < sum; ++i)
	{
		if (p_markImg[i] == 0)
			continue;

		p_img[i] = ((double)hist[p_img[i]] * 255.0) / (double)regionSum;
	}
}


int markOutContour(unsigned char* p_img, Contour* p_contours, unsigned short w, unsigned short h)
{
	unsigned short x = 0;
	unsigned short y = 0;

	unsigned short xCur = 0;
	unsigned short yCur = 0;

	unsigned short xStart = 0;
	unsigned short yStart = 0;

	unsigned short xTmp = 0;
	unsigned short yTmp = 0;

	short offsetSeq = 0;
	short searchTime = 0;
	Coord coordOffsetOf8Conn[8] = { { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 }, { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 } };

	char atStartPos = 0;

	unsigned int contourSeq = 0;

	unsigned int i = 0;
	unsigned int j = 0;

	for (y = 1; y < h-1; ++y)
	{
		for (x = 1; x < w-1; ++x)
		{
			if (p_img[y * w + x] == 0)
				continue;

			xStart = x;
			yStart = y;
			xCur = x;
			yCur = y;
			atStartPos = 1;
			offsetSeq = 0;
			p_contours->num[contourSeq] = 0;

			while ((xCur != xStart || yCur != yStart || atStartPos) && (xCur > 1 && xCur < w -1 && yCur > 1 && yCur < h - 1))
			{
				atStartPos = 0;

				xTmp = xCur + coordOffsetOf8Conn[offsetSeq].x;
				yTmp = yCur + coordOffsetOf8Conn[offsetSeq].y;

				searchTime = 1;

				while (p_img[yTmp * w + xTmp] == 0 && searchTime <= 8)
				{
					offsetSeq++;
					searchTime++;
					if (offsetSeq >= 8)
						offsetSeq -= 8;

					xTmp = xCur + coordOffsetOf8Conn[offsetSeq].x;
					yTmp = yCur + coordOffsetOf8Conn[offsetSeq].y;
				}

				// singular point
				if (searchTime > 8)
				{
					p_img[yCur * w + xCur] = 0;
					break;
				}

				// add the newly founded point into the "contour"
				p_img[yTmp * w + xTmp] = 0;
				p_contours->p_coords[i].x = xTmp;
				p_contours->p_coords[i].y = yTmp;
				p_contours->num[contourSeq]++;
				i++;
				xCur = xTmp;
				yCur = yTmp;

				offsetSeq += 6;
				if (offsetSeq >= 8)
					offsetSeq -= 8;
			}

			if (xCur != xStart || yCur != yStart || p_contours->num[contourSeq] <= 3)
			{
				// if there is only one pixel size gap between starting point and ending point, tolerate it to be a closed contour
				/*if (abs(xCur - xStart) < 3 && abs(yCur - yStart) < 3 && p_contours->num[contourSeq] > 3)
				{
					contourSeq++;
					continue;
				}*/

				// indicate that this stored edge points are not edge of a closed contour, clear this useless stored data
				i -= p_contours->num[contourSeq];
				p_contours->num[contourSeq] = 0;

				// clear the starting point
				p_img[y * w + x] = 0;

				continue;
			}

			if (searchTime > 8)
				continue;

			// if this stored edge are of a closed contour, go to store another contour.
			contourSeq++;
		}
	}

	return contourSeq;
}


void markMaxOutContour(unsigned char* p_img, Contour* p_contours, unsigned int contourNum, unsigned short w, unsigned short h)
{
	unsigned int c = 0;
	unsigned int maxLen = 0;
	unsigned int cMax = 0;
	unsigned int i = 0;

	for (c = 0; c < contourNum; ++c)
	{
		if (p_contours->num[c] > maxLen)
		{
			maxLen = p_contours->num[c];
			cMax = c;
		}
	}

	for (i = 0; i < maxLen; ++i)
	{
		p_img[p_contours->p_coords[i].y * w + p_contours->p_coords[i].x] = 255;
	}
}


int fillRegion(unsigned char* p_img, unsigned short w, unsigned short h, unsigned char maxVal)
{
	unsigned short x = 0;
	unsigned short y = 0;
	unsigned int pos = 0;
	unsigned int t_pos = 0;
	unsigned int posLeft = 0;
	unsigned int posRight = 0;
	unsigned char posLeftFound = 0;
	unsigned char posRightFound = 0;
	unsigned int count = 0;

	for (y = 1; y < h - 1; ++y)
	{
		posLeftFound = 0;
		posRightFound = 0;

		for (x = 1; x < w - 1; ++x)
		{
			pos = y * w + x;
			
			if (p_img[pos] == 0)
				continue;

			if (!posLeftFound && !posRightFound)
			{
				while (p_img[pos + 1] != 0)
				{
					pos++;
					x++;
				}

				if (p_img[pos - w - 1] != 0 || p_img[pos - w] != 0 || p_img[pos - w + 1] != 0
					|| p_img[pos - 1] != 0
					|| p_img[pos + w - 1] != 0 || p_img[pos + w] != 0 || p_img[pos + w + 1] != 0)
				{
					posLeftFound = 1;
					posLeft = pos;
				}
			}
			else if (posLeftFound)
			{
				if (p_img[pos - w - 1] != 0 || p_img[pos - w] != 0 || p_img[pos - w + 1] != 0
					|| p_img[pos - 1] != 0 || p_img[pos + 1] != 0
					|| p_img[pos + w - 1] != 0 || p_img[pos + w] != 0 || p_img[pos + w + 1] != 0)
				{
					posLeftFound = 0;
					posRightFound = 1;
					posRight = pos;
					for (t_pos = posLeft + 1; t_pos < posRight; ++t_pos)
					{
						p_img[t_pos] = maxVal;
					}

					while (p_img[pos + 1] != 0)
					{
						x++;
						pos++;
					}
					x--;
					posRightFound = 0;
				}
			}
		}
	}

	for (pos = 0; pos < w * h; ++pos)
	{
		if (p_img[pos] != 0)
			count++;
	}

	return count;
}


void seedFillRegion(unsigned char* p_img, unsigned short w, unsigned short h)
{
	
}


int markMaxConnRegion(unsigned char* p_img, unsigned short w, unsigned short h)
{
	int i = 0; 
	int count = 0;

	for (i = 0; i < w * h; ++i)
	{
		if (p_img[i] != 0)
			count++;
	}

	return count;
}


int preProcess(unsigned char* p_img, unsigned char* p_markImg, unsigned char maxVal, unsigned short w, unsigned short h)
{
	int preArea = 0;
	gaussin(p_img, w, h);

	memcpy(p_markImg, p_img, w * h);

	binary(p_markImg, threshByFirstVally(p_markImg, w, h), maxVal, w, h);

	setBoundaryZero(p_markImg, w, h);
	unsigned char* p_tmpImg = (unsigned char*)malloc_check(w * h);
	memcpy(p_tmpImg, p_markImg, w * h);
	elate(p_tmpImg, p_markImg, w, h);
	subtract(p_markImg, p_tmpImg, w, h);

	Contour contours;
	contours.p_coords = (Coord*)malloc_check(w * h * sizeof(Coord));
	int contourNum = markOutContour(p_markImg, &contours, w, h);
	markMaxOutContour(p_markImg, &contours, contourNum, w, h);
	preArea = fillRegion(p_markImg, w, h, maxVal);
	equHist(p_img, p_markImg, w, h);

	free(p_tmpImg);
	p_tmpImg = NULL;
	free(contours.p_coords);
	contours.p_coords = NULL;

	return preArea;
}
