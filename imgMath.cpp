#include "imgMath.h"
#include "error.h"
#include <malloc.h>

// calculate the combinations of a circular disk
coord *calcDiskTmplArray(unsigned short r, unsigned int *p_arrNum)
{
    if (p_arrNum == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of pointer to array num");
        exit(-1);
    }

    coord* p_coord = (coord*)calloc_check(4 * r * r, sizeof(coord));
    int r2 = r * r;
    unsigned int count = 0;

    for (short i = -r; i <= r; ++i)
    {
        for (short j = -r; j <= r; ++j)
        {
            if (i * i + j * j <= r2)
            {
                p_coord[count].x = i;
                p_coord[count].y = j;
                count ++;
            }
        }
    }

    *p_arrNum = count;

    return p_coord;
}
