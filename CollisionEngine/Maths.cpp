#include "Maths.h"

#include <stdlib.h>

float Sign(float a)
{
	return Select(a >= 0.0f, 1.0f, -1.0f);
}

float Random(float from, float to)
{
	return from + (to - from) * (((float)rand()) / ((float)RAND_MAX));
}

size_t Random(size_t from, size_t to)
{
	return from + (to - from) * ((rand()) / (RAND_MAX));
}