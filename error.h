#ifndef ERROR_H
#define ERROR_H

#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>

void *malloc_check(size_t _Size);
void *calloc_check(size_t _NumOfElements, size_t _SizeOfElements);


#define DEBUG


#ifdef DEBUG
#define DEBUG_PRINT_DETAILED(format, args...) printf("\n\nFile: " __FILE__ "\n" "Line: %d\nMsgs: " format, __LINE__, ##args)
#define DEBUG_PRINT_SIMPLIFIED(format, args...) printf(format, ##args)

#else
#define DEBUG_PRINT_DETAILED(format, args...)
#define DEBUG_PRINT_SIMPLIFIED(format, args...)

#endif

#endif
