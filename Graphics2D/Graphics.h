#ifndef GRAPHICS_H
#define GRAPHICS_H

// TODO: the name is a bit misleading - rename to GraphicsBase.h (defines & includes)?

#include <Engine/Platform.h>
#include <Engine/Utils.h>
#include <Math/Utils.h>

#define OPENGL		1
#define OPENGLES	2

#if defined(ANDROID_NDK) || defined(GLES)
#	define RENDERER OPENGLES
#	define FIXED_PIPELINE
#else
#	define RENDERER OPENGL
//#	define FIXED_PIPELINE
#endif

#if defined(_WIN32) && (RENDERER == GDIPLUS)
#	include <objidl.h>
#	include <gdiplus.h>
#	pragma comment(lib, "gdiplus.lib")
#endif

#if (RENDERER == OPENGL) || (RENDERER == GDIPLUS)

// FreeImage
#if (RENDERER != GDIPLUS)
// todo: static linkage (#define freeimage_lib)
#	include <FreeImage.h>
#endif

// GLEW
#include <GL/glew.h>
#include <GL/wglew.h>

// OpenGL Header Files
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include <GL/glut.h>

#define ANGLE(x) (x)
#elif (RENDERER == OPENGLES)

#ifndef ANDROID_NDK
#	include <EGL\egl.h>
#	include <EGL\eglplatform.h>

#	pragma comment(lib, "FreeImage.lib")
#	include <freeimage/freeimage.h>

#	pragma comment(lib, "libEGL.lib")
#	pragma comment(lib, "libgles_cm.lib")
#	pragma comment(lib, "FreeImage.lib")
#endif

#	include <GLES\gl.h>
#	include <GLES\glext.h>

#	define ANGLE(x) (x)
#else
#	pragma comment(lib, "d3d9.lib")
#	pragma comment(lib, "d3dx9.lib")

// Direct3D 9 Header Files
#	include <DirectX/D3D9.h>
#	include <DirectX/D3dx9math.h>

//#	define USE_D3DXSPRITE
#	define MATRIX_STACK_SIZE 10
#	define ANGLE(x) RADIAN(x)
#endif

inline bool CheckGlError()
{
#if defined(_DEBUG) 
	GLenum err = glGetError();
	if (err != GL_NO_ERROR)
	{
#if (RENDERER != OPENGLES)
		Printf("GL error: %s\n", gluErrorString(err));
#else
		if (err == GL_INVALID_ENUM)
			Printf("GL error: invalid enum\n");
		else if (err == GL_INVALID_VALUE)
			Printf("GL error: invalid value\n");
		else
			Printf("GL error: 0x%x\n", err);
#endif
		ASSERT(false);
		return false;
	}
#endif
	return true;
}

#ifdef _DEBUG
#define GL_DEBUG
#endif

#ifdef GL_DEBUG
#define GL_CALL(func) func; if (!CheckGlError()) Printf(#func" at line %d in %s\n", __LINE__, __FILE__);
#else
#define GL_CALL(func) func;
#endif

#endif // GRAPHICS_H
