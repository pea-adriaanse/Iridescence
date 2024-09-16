#pragma once
#include <pbrt/base/bxdf.h>

namespace pbrt {
	// 1, 4, 16, 64
	PBRT_CPU_GPU static constexpr unsigned int pow4(unsigned int exponent)
	{
		return 1 << (2 * exponent);
	}

	// 0, 4, 20, 84
	PBRT_CPU_GPU static constexpr unsigned int pow4sum(unsigned int exponent)
	{
		return (4 * (pow4(exponent) - 1)) / 3; // Modified geometric series
	}
}