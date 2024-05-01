#include <Engine/Base.h>
#include <Graphics2D/Graphics2D.h>
#include <Engine/Profiler.h>

#if (RENDERER == OPENGL)

// FreeType includes
#include <ft2build.h>
#include FT_FREETYPE_H

//#define USE_NANOVG

#ifdef USE_NANOVG
#include "nanovg.h"
#define NANOVG_GL3_IMPLEMENTATION
#include "nanovg_gl.h"
#endif

#ifndef FIXED_PIPELINE

using namespace Math;

FT_Face face;
unsigned int fontShader;

struct GlyphInfo
{
	bool valid;
	FT_GlyphSlotRec_ slot;
	unsigned int tex;

	GlyphInfo() : valid(false) { }
};
GlyphInfo cache[256];
 
bool LoadShaders(const char* vertex_file_path, const char * fragment_file_path, GLuint& ProgramID);

// force a laptop to use the Nvidia card (Optimus technology)
extern "C" {
	_declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
}

#define SIMPLE_OPENGL_CLASS_NAME "SimpleGLClass"

LRESULT CALLBACK MsgHandlerSimpleOpenGLClass(HWND hWnd, UINT uiMsg, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;
	switch(uiMsg)
	{
	case WM_PAINT:									
		BeginPaint(hWnd, &ps);							
		EndPaint(hWnd, &ps);					
		break;

	default:
		return DefWindowProc(hWnd, uiMsg, wParam, lParam); // Default window procedure
	}
	return 0;
}

bool InitGLEW(HINSTANCE hInstance)
{ 
	//RegisterSimpleOpenGLClass
	WNDCLASSEX wc;

	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style =  CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_DBLCLKS;
	wc.lpfnWndProc = (WNDPROC)MsgHandlerSimpleOpenGLClass;
	wc.cbClsExtra = 0; wc.cbWndExtra = 0;
	wc.hInstance = hInstance;
	wc.hIcon = LoadIcon(hInstance, IDI_APPLICATION);
	wc.hIconSm = LoadIcon(hInstance, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_MENUBAR+1);
	wc.lpszMenuName = NULL;
	wc.lpszClassName = SIMPLE_OPENGL_CLASS_NAME;

	RegisterClassEx(&wc);

	HWND hWndFake = CreateWindow(SIMPLE_OPENGL_CLASS_NAME, "FAKE", WS_OVERLAPPEDWINDOW | WS_MAXIMIZE | WS_CLIPCHILDREN, 
		0, 0, CW_USEDEFAULT, CW_USEDEFAULT, NULL, 
		NULL, hInstance, NULL); 

	HDC hDC = GetDC(hWndFake); 

	// First, choose false pixel format

	PIXELFORMATDESCRIPTOR pfd; 
	memset(&pfd, 0, sizeof(PIXELFORMATDESCRIPTOR)); 
	pfd.nSize= sizeof(PIXELFORMATDESCRIPTOR); 
	pfd.nVersion   = 1; 
	pfd.dwFlags    = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW; 
	pfd.iPixelType = PFD_TYPE_RGBA; 
	pfd.cColorBits = 32; 
	pfd.cDepthBits = 32; 
	pfd.iLayerType = PFD_MAIN_PLANE; 

	int iPixelFormat = ChoosePixelFormat(hDC, &pfd); 
	if (iPixelFormat == 0)return false; 

	if(!SetPixelFormat(hDC, iPixelFormat, &pfd))return false; 

	// Create the false, old style context (OpenGL 2.1 and before)

	HGLRC hRCFake = wglCreateContext(hDC); 
	wglMakeCurrent(hDC, hRCFake); 

	bool bResult = true; 

	// TODO: get max supported version and use it below (or fall back to fixed pipeline)
	int major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);
	const GLubyte *oglVersion = glGetString(GL_VERSION);
	Printf("This system supports OpenGL %d.%d: %s\n", major, minor, oglVersion);
	const GLubyte *oglVendor = glGetString(GL_VENDOR);
	Printf("Vendor: %s\n", oglVendor);
	const GLubyte *oglRenderer = glGetString(GL_RENDERER);
	Printf("Renderer: %s\n", oglRenderer);
	if (major <= 1)
		bResult = false;

	GLenum err = glewInit();
	if(err != GLEW_OK) 
	{ 
		MessageBox(NULL, "Couldn't initialize GLEW!", "Fatal Error", MB_ICONERROR); 
		Printf("Error: %s\n", glewGetErrorString(err));
		bResult = false; 
	} 

	wglMakeCurrent(NULL, NULL); 
	wglDeleteContext(hRCFake); 
	DestroyWindow(hWndFake); 

	return bResult; 
}

bool Graphics2D::Init(HWND window, HINSTANCE instance)
{
	// setup the pixel format descriptor
	hDC = GetDC(window);
	
#ifdef OLD_WAY
	// old way of initializing
	PIXELFORMATDESCRIPTOR pfd = { 
		sizeof(PIXELFORMATDESCRIPTOR),    // size of this pfd 
		1,                                // version number 
		PFD_DRAW_TO_WINDOW |              // support window 
		PFD_SUPPORT_OPENGL |              // support Graphics2D 
		PFD_DOUBLEBUFFER,                 // double buffered
		PFD_TYPE_RGBA,                    // RGBA type 
		32,                               // 32-bit color depth 
		0, 0, 0, 0, 0, 0,                 // color bits ignored 
		0,                                // no alpha buffer 
		0,                                // shift bit ignored 
		0,                                // no accumulation buffer 
		0, 0, 0, 0,                       // accum bits ignored 
		32, // TODO: no z buffer?         // 32-bit z-buffer     
		8,                                // no stencil buffer 
		0,                                // no auxiliary buffer 
		PFD_MAIN_PLANE,                   // main layer 
		0,                                // reserved 
		0, 0, 0                           // layer masks ignored 
	}; 
	int  iPixelFormat; 
	// get the device context's best, available pixel format match 
	iPixelFormat = ChoosePixelFormat(hDC, &pfd); 
	// make that match the device context's current pixel format 
	SetPixelFormat(hDC, iPixelFormat, &pfd); 

	// if we can create a rendering context ...  
	if (hGLRC = wglCreateContext(hDC))
	{
		// try to make it the thread's current rendering context 
		BOOL bHaveCurrentRC = wglMakeCurrent(hDC, hGLRC);  
	}
	else
		return false;

	// Setup GLEW which loads OGL function pointers
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		// Problem: glewInit failed, something is seriously wrong.
		Printf("Error: %s\n", glewGetErrorString(err));
		return false;
	}
#else	
	if (!InitGLEW(instance))
		return false;
	
	int nPixCount = 0;
	// Specify the important attributes we care about
	int pixAttribs[] = {
		WGL_SUPPORT_OPENGL_ARB, GL_TRUE, // Must support OGL rendering
		WGL_DRAW_TO_WINDOW_ARB, GL_TRUE, // pf that can run a window
		WGL_ACCELERATION_ARB, WGL_FULL_ACCELERATION_ARB, // must be HW accelerated
		WGL_COLOR_BITS_ARB, 24, // 8 bits of each R, G and B
		WGL_DEPTH_BITS_ARB, 16, // 16 bits of depth precision for window
		WGL_STENCIL_BITS_ARB, 8, // 8 bits of stencil precision for window
		WGL_DOUBLE_BUFFER_ARB, GL_TRUE, // Double buffered context
		WGL_PIXEL_TYPE_ARB, WGL_TYPE_RGBA_ARB, // pf should be RGBA type
		WGL_SAMPLE_BUFFERS_ARB, 1, //Number of buffers (must be 1 at time of writing)
		WGL_SAMPLES_ARB, 1,        //Number of samples
		0 }; // NULL termination
	// Ask OpenGL to find the most relevant format matching our attribs. Only get one format back.
	int nPixelFormat[10]; // TODO: try more formats
	if (!wglChoosePixelFormatARB(hDC, &pixAttribs[0], NULL, 10, nPixelFormat, (UINT*)&nPixCount))
	{
		Printf("Failed to choose a pixel format\n");
		return false;
	}
	//CheckGlError();

	// Got a format, now set it as the current one
	PIXELFORMATDESCRIPTOR pfd;
	SetPixelFormat(hDC, nPixelFormat[0], &pfd);

	GLint attribs[] = {
		WGL_CONTEXT_MAJOR_VERSION_ARB, 3,
		WGL_CONTEXT_MINOR_VERSION_ARB, 1,
		//WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_DEBUG_BIT_ARB,
		WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_COMPATIBILITY_PROFILE_BIT_ARB,
		0 };
	hGLRC = wglCreateContextAttribsARB(hDC, 0, attribs);	
	if (hGLRC == NULL)
	{
		Printf("Error creating new render context\n");
		return false;
	}
	wglMakeCurrent(hDC, hGLRC);
	CheckGlError();
#endif

	if (!GL_ARB_explicit_attrib_location)
		Printf("No support for explicit attrib location\n");
	if (!GL_ARB_vertex_buffer_object)
		Printf("No support for vertex buffer object\n");
	if (!GL_ARB_texture_rectangle)
		Printf("No support for texture rectangle\n");

	// create VBO
	//GLuint VertexArrayID;
	//glGenVertexArrays(1, &VertexArrayID);
	//glBindVertexArray(VertexArrayID); 

	glGenVertexArrays(1, &vao);
	
	glGenBuffers(1, &vertexBuffer);
	glGenBuffers(1, &uvBuffer);
	glGenBuffers(1, &indexBuffer);

	// Create and compile our GLSL program from the shaders
	if (!LoadShaders("../Shaders/vertex2d.shader", "../Shaders/pixel2d.shader", programID))
		return false;

	if (!LoadShaders("../Shaders/vertex2d.shader", "../Shaders/pixel2drect.shader", programRectID))
		return false;

	MatrixID = glGetUniformLocation(programID, "MVP");
	ColorID  = glGetUniformLocation(programID, "color");
	hasTexID = glGetUniformLocation(programID, "has_texture");

	// Frame Buffer Object
	glGenFramebuffers(1, &frameBufferObj);

	if (!LoadShaders( "../Shaders/vertex2d.shader", "../Shaders/pixel2d_fb.shader", shaderFb))
		return false;

	if (!LoadShaders( "../Shaders/vertex2d.shader", "../Shaders/pixel2d_fb_rect.shader", shaderFbRect))
		return false;

	mvpFb = glGetUniformLocation(shaderFb, "MVP");
	sizeFb = glGetUniformLocation(shaderFb, "size");
	kernelFb = glGetUniformLocation(shaderFb, "kernel");
	widthFb = glGetUniformLocation(shaderFb, "width");
	heightFb = glGetUniformLocation(shaderFb, "height");

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Free Type init
	FT_Library ft;
 
	if(FT_Init_FreeType(&ft))
	{
		Printf("Could not init freetype library\n");
		return false;
	}

	if(FT_New_Face(ft, "../Res/ARLRDBD.TTF", 0, &face))
	{
		Printf("Could not open font\n");
		return false;
	}

	FT_Set_Pixel_Sizes(face, 0, 12);

	unsigned int glyphs[256];
	glGenTextures(256, glyphs);
	for (int i = 0; i < 256; i++)
	{
		glBindTexture(Texture::texType, glyphs[i]);

		glTexParameteri(Texture::texType, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(Texture::texType, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
 
		glTexParameteri(Texture::texType, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(Texture::texType, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		cache[i].tex = glyphs[i];
	}

	glUseProgram(programID);
	currProgram = programID;

	CheckGlError();

	texFlag = 1;
	oldProj = Matrix4::Identity();
	//w = h = -1;

	glGetIntegerv(GL_UNPACK_ALIGNMENT, &align);

	glUniform1i(glGetUniformLocation(programID, "texSampler"), 0);
	glUniform1i(glGetUniformLocation(programID, "texSampler2"), 1);
	glUniform1i(glGetUniformLocation(programID, "texSampler3"), 2);

#ifdef USE_NANOVG
	vg = nvgCreateGL3(NVG_ANTIALIAS | NVG_STENCIL_STROKES/* | NVG_DEBUG*/);

	if (vg == nullptr)
		return false;

	nvgCreateFont(vg, "sans", "../Res/ARLRDBD.TTF");
	nvgCreateFont(vg, "serif", "C:/Windows/Fonts/times.ttf");
#endif

	return true;
}

void Graphics2D::SetContext()
{
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE); // TODO: reorient quad

	// solid/wireframe
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	
	// line antialiasing
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	glUseProgram(programID);

	glEnable(GL_STENCIL_TEST);

	CheckGlError();
}

void Graphics2D::Resize(int width, int height)
{
	w = width;
	h = height;
	Projection = Matrix4::Translation(-1, 1, 0) * Matrix4::Scale(2.f / w, -2.f / h, 1);

	ResetTransform();
}

void Graphics2D::DeInit()
{
	//glDeleteBuffers(1, &vertexBuffer);
	// TODO: delete the other buffers too

	wglMakeCurrent(0, 0);
	ReleaseDC(hWnd, hDC) ;
	wglDeleteContext(hGLRC);

#ifdef USE_GDIPLUS
	Gdiplus::GdiplusShutdown(gdiplusToken);
#elif !defined(_WIN64)
	FreeImage_DeInitialise();
#endif
}

void Graphics2D::Viewport()
{
	glViewport(0, 0, w, h);

	CheckGlError();
}

void Graphics2D::BeginDraw()
{
	glClearColor(bgColor[0], bgColor[1], bgColor[2], 0.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glUseProgram(Texture::texType == GL_TEXTURE_2D ? programID : programRectID);
	currProgram = programID;

	CheckGlError();

#ifdef USE_NANOVG
	nvgBeginFrame(vg, w, h, 1.0f);
#endif
}

void Graphics2D::EndDraw()
{
#ifdef USE_NANOVG
	nvgBeginPath(vg);
	nvgEndFrame(vg);
#endif

	CheckGlError();
	SwapBuffers(hDC);
}

void Graphics2D::DrawRegion(Texture* image, float x_dest, float y_dest, int x_src, int y_src, 
	int width, int height, float scale, int anchor)
{
	//MEASURE_TIME("DrawRegion GLS");
	
	if (image->GetAlpha() == 0)
		return;
	if (x_src >= image->Width() || y_src >= image->Height())
		return;

	y_src = image->Height() - y_src;

	//remember original xsrc & ysrc
	int xsrc = x_src;
	int ysrc = y_src;
	
	//if we're before the image move to its begining
	if(xsrc < 0)
		xsrc = 0;
	if(ysrc < 0)
		ysrc = 0;
	
	//if we moved xsrc or ysrc we need to update x_dst or y_dst & width or height
	x_dest += ( xsrc - x_src );
	y_dest += ( ysrc - y_src );
	width  -= ( xsrc - x_src );
	height -= ( ysrc - y_src );
	
	//adjust x_dest & y_dest based on align request
	//if( anchor & Align::Right )
	//	x_dest -= width;
	//if( anchor & Align::Bottom )
	//	y_dest -= height;
	if (anchor & Align::HCenter)
		x_dest -= 0.5f * scale * width;
	if (anchor & Align::VCenter)
		y_dest -= 0.5f * scale * height;

	//if( (transform & Transform::Rot90)!=0 )
	//{
	//	Translate(height, 0);
	//	Rotate(90);
	//}
	//if( (transform & Transform::FlipX)!=0 )
	//{
	//	Scale(-1, 1);
	//	Translate(-width, 0);
	//}
	//if( (transform & Transform::FlipY)!=0 )
	//{
	//	Scale(1, -1);
	//	Translate(0, -height);
	//}

	// compute texture coordinates
	const float imageW = image->Width();
	const float imageH = image->Height();
	float minX, minY, maxX, maxY;
	if (Texture::texType == GL_TEXTURE_2D)
	{
		minX  = (x_src) / imageW;
		minY  = (y_src) / imageH;
		maxX  = (x_src + width ) / imageW;
		maxY  = (y_src - height ) / imageH;
	}
	else
	{
		// TODO: fix for regions
		minX  = 0;
		minY  = image->Height();
		maxX  = image->Width();
		maxY  = 0;
	}

	
	GL_CALL(glActiveTexture(GL_TEXTURE0));
	GL_CALL(glBindTexture(Texture::texType, image->ID));
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE); // this is for shadow maps

	float texCoords[] = { minX, minY, minX, maxY, maxX, maxY, maxX, maxY, maxX, minY, minX, minY};
	float coords[] = { 0, 0, 0, 0, (float)height, 0, (float)width, (float)height, 0, (float)width, (float)height, 0, (float)width, 0, 0, 0, 0, 0 };

	// TODO: proper VAO
	GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer));
	GL_CALL(glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(Vector3), &coords[0], GL_STATIC_DRAW));
	
	GL_CALL(glEnableVertexAttribArray(0));
	GL_CALL(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0));
 
	GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, uvBuffer));
	GL_CALL(glBufferData(GL_ARRAY_BUFFER, 6 * 2 * sizeof(float), &texCoords[0], GL_STATIC_DRAW));
	GL_CALL(glEnableVertexAttribArray(1));
	GL_CALL(glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0));

	Matrix4 MVP = Projection * Matrix4::Translation(x_dest, y_dest, 0) * View * Matrix4::Scale(scale, scale, scale);
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	if (currProgram == programID)
	{
		glUniform1i(hasTexID, texFlag); 
		glUniform4f(ColorID, 1, 1, 1, image->GetAlpha() / 255.f);		
	}
	
	GL_CALL(glDrawArrays(GL_TRIANGLES, 0, 6));

	glDisableVertexAttribArray(0);

	CheckGlError();
}

void Graphics2D::DrawBlendedImages(Texture* image1, Texture* image2, Texture* image3, float x, float y, float scale)
{
	float width = image1->Width();
	float height = image1->Height();

	// compute texture coordinates
	float minX  = 0;
	float minY  = 1;
	float maxX  = 1;
	float maxY  = 0;

	float texCoords[] = { minX, minY, minX, maxY, maxX, maxY, maxX, maxY, maxX, minY, minX, minY};
	float coords[] = { 0, 0, 0, 0, height, 0, width, height, 0, width, height, 0, width, 0, 0, 0, 0, 0 };

	GL_CALL(glActiveTexture(GL_TEXTURE2));
	GL_CALL(glBindTexture(Texture::texType, image3->ID));

	GL_CALL(glActiveTexture(GL_TEXTURE1));
	GL_CALL(glBindTexture(Texture::texType, image2->ID));

	GL_CALL(glActiveTexture(GL_TEXTURE0));
	GL_CALL(glBindTexture(Texture::texType, image1->ID));

	GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer));
	GL_CALL(glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(Vector3), &coords[0], GL_STATIC_DRAW));
	GL_CALL(glEnableVertexAttribArray(0));
	GL_CALL(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0));

	GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, uvBuffer));
	GL_CALL(glBufferData(GL_ARRAY_BUFFER, 6 * 2 * sizeof(float), &texCoords[0], GL_STATIC_DRAW));
	GL_CALL(glEnableVertexAttribArray(1));
	GL_CALL(glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0));

	Matrix4 MVP = Projection * Matrix4::Translation(x, y, 0) * View * Matrix4::Scale(scale, scale, scale);
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData()); 
	if (currProgram == programID)
	{
		glUniform1i(hasTexID, 4);
		float alpha2 = image2->GetAlpha() / 255.f;
		float alpha3 = image3->GetAlpha() / 255.f;
		if (image2 == image1)
			alpha2 = 0;
		if (image3 == image1 || image3 == image2)
			alpha3 = 0;
		glUniform4f(ColorID, image1->GetAlpha() / 255.f, alpha2, alpha3, 0);
	}

	GL_CALL(glDrawArrays(GL_TRIANGLES, 0, 6));

	//glDisableVertexAttribArray(0);
	//glDisableVertexAttribArray(1);

	glUniform1i(hasTexID, 1); 

	CheckGlError();
}

// TODO: VAO
void Graphics2D::DrawRect(float x, float y, float width, float height)
{
#ifndef USE_NANOVG
	float rect[] = { x, y, 0, 
		x + width, y, 0, 
		x + width, y + height, 0, 
		x, y + height, 0,
		x, y, 0 };

	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, 5 * sizeof(Vector3), &rect[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
 	
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawArrays(GL_LINE_STRIP, 0, 5);

	glDisableVertexAttribArray(0);
	CheckGlError();
#else
	nvgBeginPath(vg);
	nvgStrokeWidth(vg, lineWidth);
	nvgRect(vg, x, y, width, height);
	nvgStroke(vg);
#endif
}

void Graphics2D::FillRect(float x, float y, float width, float height)
{
	// TODO: triangle fan
	float rect[] = { x, y, 0, x + width, y, 0, x + width, y + height, 0, x, y + height, 0, x, y, 0, x + width, y + height, 0 };

	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(Vector3), &rect[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
 	
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawArrays(GL_TRIANGLES, 0, 6);

	glDisableVertexAttribArray(0);
	CheckGlError();
}

void Graphics2D::DrawLine(float x1, float y1, float x2, float y2)
{
#ifndef USE_NANOVG
	float line[] = { x1, y1, 0, x2, y2, 0 };

	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(Vector3), &line[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawArrays(GL_LINES, 0, 2);

	glDisableVertexAttribArray(0);
	CheckGlError();
#else
	nvgBeginPath(vg);
	nvgStrokeWidth(vg, lineWidth);
	NVGcolor col;
	col.rgba[0] = color[0];
	col.rgba[1] = color[1];
	col.rgba[2] = color[2];
	col.rgba[3] = color[3];
	nvgStrokeColor(vg, col);
	nvgMoveTo(vg, x1, y1);
	nvgLineTo(vg, x2, y2);
	nvgStroke(vg);
#endif
}

void Graphics2D::DrawPolyLine(float v[], int n, bool fill)
{
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, n * sizeof(Vector2), &v[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
 	
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawArrays(fill ? GL_TRIANGLE_FAN : GL_LINE_LOOP, 0, n);

	glDisableVertexAttribArray(0);
	CheckGlError();
}

void Graphics2D::DrawCircle(float x, float y, float radius, float arcLen)
{
#ifndef USE_NANOVG
	int n = (int)(2.0f * PI * radius / arcLen + 0.5f);
	const int minSegments = 7;
	if (n < minSegments)
		n = minSegments;
	Vector3* circle = new Vector3[n]; // stack alloc
	const float delta = 360.0f / n;
	float angle = 0;
	for (int i = 0; i < n; i++)
	{
		float x1 = x + radius * cos(RADIAN(angle));
		float y1 = y + radius * sin(RADIAN(angle));
		circle[i].Set(x1, y1, 0);
		angle += delta;
	}
	
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, n * sizeof(Vector3), &circle[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
 	
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawArrays(GL_LINE_LOOP, 0, n);

	glDisableVertexAttribArray(0);
	CheckGlError();

	delete[] circle; // TODO: better way (static allocation)
#else
	nvgBeginPath(vg);
	nvgStrokeWidth(vg, lineWidth);
	nvgCircle(vg, x, y, radius);
	nvgStroke(vg);

#endif
}

void Graphics2D::DrawEllipse(float x, float y, float rx, float ry, float arcLen)
{
	int n = (int)(2.0f * PI * max(rx, ry) / arcLen + 0.5f);
	const int minSegments = 7;
	if (n < minSegments)
		n = minSegments;
	Vector3* circle = new Vector3[n]; // stack alloc
	const float delta = 360.0f / n;
	float angle = 0;
	for (int i = 0; i < n; i++)
	{
		float x1 = x + rx * cos(RADIAN(angle));
		float y1 = y + ry * sin(RADIAN(angle));
		circle[i].Set(x1, y1, 0);
		angle += delta;
	}
	
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, n * sizeof(Vector3), &circle[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
 	
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawArrays(GL_LINE_LOOP, 0, n);

	glDisableVertexAttribArray(0);
	CheckGlError();

	delete[] circle; // TODO: better way (static allocation)
}

void Graphics2D::FillCircle(float x, float y, float radius, float arcLen)
{
	int n = (int)(2.0f * PI * radius / arcLen + 0.5f);
	const int minSegments = 7;
	if (n < minSegments)
		n = minSegments;
	Vector3* circle = new Vector3[n + 2]; // stack alloc
	const float delta = 360.0f / n;
	float angle = 0;
	circle[0].Set(x, y, 0);
	for (int i = 1; i <= n; i++)
	{
		float x1 = x + radius * cos(RADIAN(angle));
		float y1 = y + radius * sin(RADIAN(angle));
		circle[i].Set(x1, y1, 0);
		angle += delta;
	}
	circle[n + 1].Set(x + radius, y, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, (n + 2) * sizeof(Vector3), &circle[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
 	
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawArrays(GL_TRIANGLE_FAN, 0, n + 2);

	glDisableVertexAttribArray(0);
	CheckGlError();

	delete[] circle; // TODO: better way (static allocation)
}

void Graphics2D::FillEllipse(float x, float y, float rx, float ry, float arcLen)
{
	int n = (int)(2.0f * PI * max(rx, ry) / arcLen + 0.5f);
	const int minSegments = 7;
	if (n < minSegments)
		n = minSegments;
	Vector3* circle = new Vector3[n + 2]; // stack alloc
	const float delta = 360.0f / n;
	float angle = 0;
	circle[0].Set(x, y, 0);
	for (int i = 1; i <= n; i++)
	{
		float x1 = x + rx * cos(RADIAN(angle));
		float y1 = y + ry * sin(RADIAN(angle));
		circle[i].Set(x1, y1, 0);
		angle += delta;
	}
	circle[n + 1].Set(x + rx, y, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, (n + 2) * sizeof(Vector3), &circle[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
 	
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawArrays(GL_TRIANGLE_FAN, 0, n + 2);

	glDisableVertexAttribArray(0);
	CheckGlError();

	delete[] circle; // TODO: better way (static allocation)
}

void Graphics2D::DrawString(float x, float y, const char* text)
{
#ifndef USE_NANOVG
	CheckGlError();
	const char *p;
	
	// TODO: scaling
	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 2);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);
	CheckGlError();

	for(p = text; *p; p++) 
	{
		char c = *p;
		int idx = (int)c;
		FT_GlyphSlot g;
		
		ASSERT(idx >= 0 && idx < 256);
		GL_CALL(glActiveTexture(GL_TEXTURE0));
		glBindTexture(Texture::texType, cache[idx].tex);
		CheckGlError();

		if (cache[idx].valid)
		{
			g = &cache[idx].slot;
		}
		else
		{
			if(FT_Load_Char(face, *p, FT_LOAD_RENDER))
				continue;
			g = face->glyph;
			cache[idx].valid = true;
			cache[idx].slot = *g;				
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glTexImage2D(Texture::texType, 0, GL_RED, g->bitmap.width, g->bitmap.rows, 0, GL_RED, GL_UNSIGNED_BYTE, g->bitmap.buffer);
			GLint swizzleMask[] = {GL_ZERO, GL_ZERO, GL_ZERO, GL_RED};
			glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
			CheckGlError();
		}

		float x2 = x + g->bitmap_left;// * sx;
		float y2 = y - g->bitmap_top;// * sy;
		float w = g->bitmap.width;// * sx;
		float h = g->bitmap.rows;// * sy;

		GLfloat box[4][3] = {
			{x2,     y2    , 0},
			{x2 + w, y2    , 0},
			{x2,     y2 + h, 0},
			{x2 + w, y2 + h, 0},
		};

		//glBindVertexArray(vao); // for GL 3.3 core
		// TODO: combine pos and uv in a vec4
		float uvs[] = {0, 0, 1, 0, 0, 1, 1, 1};
		GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, uvBuffer));
		GL_CALL(glEnableVertexAttribArray(1));
		GL_CALL(glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0));
		GL_CALL(glBufferData(GL_ARRAY_BUFFER, sizeof(uvs), uvs, GL_DYNAMIC_DRAW));

		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glBufferData(GL_ARRAY_BUFFER, sizeof(box), box, GL_DYNAMIC_DRAW);
		CheckGlError();
		
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		x += (g->advance.x >> 6);// * sx;
		y += (g->advance.y >> 6);// * sy;
	}

	//glPixelStorei(GL_UNPACK_ALIGNMENT, align);

	CheckGlError();
#else
	nvgFontFace(vg, "serif");
	nvgFontSize(vg, 15.0f);
	nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
	nvgFillColor(vg, nvgRGBA(0, 0, 0, 255));
	nvgText(vg, x, y, text, NULL);
#endif
}

void Graphics2D::DrawFormattedString(float x, float y, const char* text, ...)
{
	va_list arg_list;
	char str[256];

	va_start(arg_list, text);
	vsprintf_s(str, 256, text, arg_list);
	va_end(arg_list);

	DrawString(x, y, str);
}

void Graphics2D::SetRenderTarget(Texture* dst)
{
	if (dst == NULL)
	{
		// reset to normal rendering
		GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
		if (w > 0 && h > 0)
		{
			glClearColor(0.5f, 0.3f, 0.4f, 0.f);
			glViewport(0, 0, w, h);
			Projection = oldProj;
		}
		return;
	}

	ASSERT(dst->ID != 0);

	GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObj));

	// Set "renderedTexture" as our colour attachement #0
	GL_CALL(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, Texture::texType, dst->ID, 0));
 
	// Set the list of draw buffers.
	GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
	GL_CALL(glDrawBuffers(1, DrawBuffers)); // "1" is the size of DrawBuffers

	GLenum ret = GL_CALL(glCheckFramebufferStatus(GL_FRAMEBUFFER));
	if(ret != GL_FRAMEBUFFER_COMPLETE)
	{
		Printf("Framebuffer not ok: %d\n", ret);
		GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
		return;
	}

	// Render to our framebuffer
	glViewport(0, 0, dst->Width(), dst->Height());

	oldProj = Projection;
	Projection = Matrix4::Translation(-1, 1, 0) * Matrix4::Scale(2.f / dst->Width(), -2.f / dst->Height(), 1);

	glClearColor(0.f, 0.3f, 0.f, 0.f);
	glClear(GL_COLOR_BUFFER_BIT);
}

void Graphics2D::DrawLineMesh(const LineMesh& mesh)
{	
	//PROFILE_SCOPE("DrawLineMesh");
	// TODO: move everything over to Vector2
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, mesh.numVerts * mesh.stride, mesh.vertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, mesh.stride, (void*)0);
 	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(uint16), &mesh.indices[0], GL_STATIC_DRAW);

	Matrix4 MVP = Projection * View;
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, MVP.GetData());
	glUniform1i(hasTexID, 0);
	glUniform4f(ColorID, color[0], color[1], color[2], color[3]);

	glDrawElements(GL_LINES, (GLsizei)mesh.indices.size(), GL_UNSIGNED_SHORT, NULL); // TODO: use the pointer in the last param?
}

#endif // FIXED_PIPELINE

#endif // OPEN_GL