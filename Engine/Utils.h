#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>

//#define DISABLE_PRINTF
#ifndef DISABLE_PRINTF
void Printf(const char* text, ...);
#else
inline void Printf(const char* text, ...) { }
#endif

#ifndef DISABLE_PRINTF
#include <stdarg.h>
#include <stdio.h>
#include "Platform.h"
#if defined(LINUX)
#	define Printf printf
#else
//extern FILE* out;
inline void Printf(const char* text, ...)
{
	va_list arg_list;
	const int strSize = 8192;
	static char str[strSize];

	va_start(arg_list, text);
#ifdef WIN32
	vsprintf_s(str, strSize, text, arg_list);
#else
	vsprintf(str, text, arg_list);
#endif
	va_end(arg_list);

#if defined(_CONSOLE)
	printf("%s", str);
#elif defined(WIN32)
	OutputDebugStringA(str);
#endif

	//if (out)
	//	fprintf(out, "%s", str);

#ifdef ANDROID_NDK
	__android_log_print(ANDROID_LOG_INFO, "GameEngine", "%s", str);
#endif
}
#endif
#endif // !LINUX

#if defined(_DEBUG) && !defined(ANDROID_NDK)
//#	include <assert.h>
#	define ASSERT(_condition) if (!(_condition)) __debugbreak(); //DebugBreak(); //assert(_condition)
#else
#	define ASSERT(_condition)
#endif

#define ALIGN16 __declspec(align(16))

#define SAFE_DELETE(x) if (x != nullptr ) { delete x; x = nullptr; }

#ifdef ANDROID_NDK
#	include <android/log.h>
#	define HWND int
#	define sprintf_s(a, b, c, ...) sprintf(a, c, ## __VA_ARGS__)

struct POINT
{
	int x, y;
};

#	define VK_UP 201
#	define VK_DOWN 202
#	define VK_LEFT 203
#	define VK_RIGHT 204
#	define VK_LBUTTON 205
#endif // ANDROID_NDK

#ifdef ANDROID_NDK
#	define INLINE inline
#else
#	define INLINE __forceinline
#endif

inline unsigned int GetRandomColor(int offset = 60)
{
	int modulo = 256 - offset;
	int red = offset + rand() % modulo;
	int green = offset + rand() % modulo;
	int blue = offset + rand() % modulo;
	return 0xff000000 | (red & 0xff) << 16 | (green & 0xff) << 8 | (blue & 0xff);
}

// TODO: move to math?
inline float GetRandomReal01()
{
	const int r = rand();
	return (float)r / (float)RAND_MAX;
}

inline float GetRandomReal11()
{
	return -1.0f + 2.0f * GetRandomReal01();
}

#endif // UTILS_H
