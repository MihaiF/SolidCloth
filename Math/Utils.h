#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "Vector3.h"

#undef min
#undef max
#include <algorithm>
using std::max;
using std::min;

#undef INFINITY

class XMLElement;

namespace Math
{
	const float INFINITY = 1e10f;
	const float PI = 3.1415926535897932384626433832795f;

#define RADIAN(x) (Math::PI * (x) / 180.0f ) // convert from degrees to radians
#define DEGREE(x) (180.0f * (x) / Math::PI ) // convert from radians to degrees

	template<typename T>
	inline T clamp(T val, T minVal, T maxVal)
	{
		return std::max(minVal, std::min(maxVal, val));
	}

	inline float lerp(float a, float b, float t)
	{
		return (1.0f - t) * a + t * b;
	}

}

//inline bool isnan(float x)
//{
//	return x != x;
//}

inline int sgn(float x)
{
	return x >= 0 ? 1 : -1; // TODO: bit tricks
}

inline int NextMultiple(int x, int div)
{
	// TODO: (x + (div - 1)) / div
	int r = x % div;
	if (r == 0)
		return x;
	return x - r + div;
}

#endif

