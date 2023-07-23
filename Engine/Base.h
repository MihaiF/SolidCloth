#ifndef BASE_H
#define BASE_H

#include "Platform.h"

// C RunTime Header Files
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <malloc.h>

#ifdef WIN32
// TODO: make it worth in Math; remove it from Vector2SSE.h; guard it with ifdef
	#include <intrin.h>
#endif // ANDROID_NDK

// MemoryFramework
// TODO: solve LNK4098
//#include "MemoryFramework/newdelete.h"
namespace MemoryFramework
{
	inline void Free(void* ptr)
	{
		free(ptr);
	}

	inline void* Alloc(int size, const char*, const char*)
	{
		return malloc(size);
	}
}

// TODO: analyze
#include <Engine/Types.h>
#include <Engine/Utils.h>
#include <Graphics2D/Graphics.h>

#endif // BASE_H