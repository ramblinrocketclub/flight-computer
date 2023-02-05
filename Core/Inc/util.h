#ifndef UTIL_H
#define UTIL_H

#include "printf.h"
#include "arm_math.h"

#define ARM_CHECK_STATUS(x) if (x != ARM_MATH_SUCCESS) {\
		    printf("Arm math operation failed!\n"); \
		}

#endif /* UTIL_H */