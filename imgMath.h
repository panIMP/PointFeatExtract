#ifndef IMGMATH_H
#define IMGMATH_H

typedef struct COORD
{
    short x;
    short y;
} coord;


// calculate the combinations of a circular disk
coord* calcDiskTmplArray(unsigned short r, unsigned int* p_arrNum);

#endif // IMGMATH_H
