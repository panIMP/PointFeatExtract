#include "error.h"
#include <malloc.h>
#include <stdlib.h>
#include <stdarg.h>


void *malloc_check(size_t _Size)
{
    void* p = malloc(_Size);
    if (p == NULL) {
        DEBUG_PRINT_DETAILED("Memory allocation failed!");
        exit(-1);
    }

    return p;
}


void *calloc_check(size_t _NumOfElements, size_t _SizeOfElements)
{
    void* p = calloc(_NumOfElements, _SizeOfElements);
    if (p == NULL) {
        DEBUG_PRINT_DETAILED("Memory allocation failed!");
        exit(-1);
    }

    return p;
}


