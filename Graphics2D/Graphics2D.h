#ifndef GRAPHICS2D_H
#define GRAPHICS2D_H

#include <Graphics2D/Graphics.h>
#include <Graphics2D/Texture.h>
#include <Engine/Utils.h>
#include <Engine/Types.h>
#include <Math/Vector2.h> // TODO: move to math
#include <Math/Matrix4.h>

#include <vector>

struct LineMesh
{
	char* vertices;
	int stride, numVerts;
	std::vector<uint16> indices;

	void AddLine(uint16 a, uint16 b)
	{
		indices.push_back(a);
		indices.push_back(b);
	}
};

struct Align 
{
	static const uint8 Left		= (1<<0); //1
	static const uint8 HCenter	= (1<<1); //2
	static const uint8 Right      = (1<<2); //4
	static const uint8 Top        = (1<<3); //8
	static const uint8 VCenter    = (1<<4); //16
	static const uint8 Bottom     = (1<<5); //32
	static const uint8 BaseLine   = (1<<6); //64

	static const uint8 TopLeft = Top | Left;
	static const uint8 TopRight = Top | Right;
	static const uint8 Center = HCenter | VCenter;
	static const uint8 BottomLeft = Bottom | Left;
	static const uint8 BottomRight = Bottom | Right;

};

// TODO: replace by rotation
struct Transform
{
	static const uint8 None = (0<<0);
	static const uint8 FlipX = (1<<0);
	static const uint8 FlipY = (1<<1);
	static const uint8 Rot90 = (1<<2);
};

class Texture;

class Graphics2D
{
	public:

#ifdef _WIN32
		HWND hWnd;
#	if (RENDERER == GDIPLUS)
		ULONG_PTR gdiplusToken;
#	endif
		HDC hDC;
#endif
		int w, h;
		float color[4];

		Math::Matrix4 View;
		
		float lineWidth;

#if defined(WIN32) && (RENDERER == OPENGL)
		// TODO: rename
		HGLRC hGLRC;
		struct NVGcontext* vg = NULL;

		// TODO: move
		Graphics2D() : lineWidth(1.f)
		{
			color[0] = color[1] = color[2] = color[3] = 1.f;
			//bgColor[0] = bgColor[1] = bgColor[2] = .2f;
			bgColor[0] = 0.5f;
			bgColor[1] = 0.3f;
			bgColor[2] = 0.4f;
		}
#	if !defined(FIXED_PIPELINE)
		Math::Matrix4 Projection;
		GLuint programID, programRectID, currProgram;
		GLuint MatrixID, ColorID;
		GLuint widthID, heightID;
		GLuint vertexBuffer, uvBuffer, indexBuffer, vao;
		GLuint hasTexID;
		GLint align;
		// for image processing
		GLuint frameBufferObj;
		GLuint shaderFb, shaderFbRect, mvpFb, sizeFb, kernelFb, widthFb, heightFb;
		Math::Matrix4 oldProj;
		int texFlag;
#	endif

		float bgColor[3];

#elif (RENDERER == OPENGLES) 
#	if !defined(ANDROID_NDK)
		EGLDisplay display;
		EGLSurface window;
#	endif
#ifdef LINE_BUFFER
		enum { BUFFER_SIZE = 70 * 1024 };
		float lines[BUFFER_SIZE];
		int lineIdx;
#endif
#elif (RENDERER == DIRECTX)
		LPDIRECT3D9 direct3D9;

		static LPDIRECT3DDEVICE9 device;

		LPDIRECT3DVERTEXBUFFER9 m_pVB;

		D3DXMATRIX transformMatrix, tempMatrix;

		static D3DXMATRIX Identity;

		uint32 color;

		ID3DXFont* font;

		ID3DXSprite* d3dxSprite;

	protected:

		D3DXMATRIX matrixStack[MATRIX_STACK_SIZE];

		int stackCursor;

	public:

		Graphics2D(): stackCursor(0) { }
#elif (RENDERER == GDIPLUS)
		Gdiplus::Graphics* gdi = nullptr;
		Gdiplus::Bitmap* frameBmp = nullptr;
		Gdiplus::Graphics *frameGdi = nullptr;
		Gdiplus::Pen* pen;
		Gdiplus::Font *font;
		Gdiplus::SolidBrush *brush;

		float sx, sy;
		bool rescale;

		uint8 bgColor[3];

		Graphics2D() : gdi(NULL), frameGdi(NULL), frameBmp(NULL), pen(NULL), font(NULL), brush(NULL), sx(1), sy(1), rescale(false)
		{
			bgColor[0] = 127;
			bgColor[1] = 76;
			bgColor[2] = 102;
		}

		~Graphics2D()
		{
			Gdiplus::GdiplusShutdown(gdiplusToken);
		}
#endif
	
	public:

		bool Init(HWND window, HINSTANCE instance);

		void DeInit();

		void Resize(int width, int height);

		void BeginDraw();
		
		void EndDraw();

		void DrawImage(Texture* image, float x_dest, float y_dest, float scale = 1.f, int anchor = 0); // TODO: const Texture*

		void DrawRegion(Texture* image, float x_dest, float y_dest, int x_src, int y_src, int width, int height, float scale = 1.f, int anchor = 0);

		void DrawBlendedImages(Texture* image1, Texture* image2, Texture* image3, float x, float y, float scale);

		void DrawRect(float x, float y, float width, float height);

		void FillRect(float x, float y, float width, float height);

		void DrawLine(float x1, float y1, float x2, float y2);

		void DrawPolyLine(float v[], int n, bool fill = false);

		void DrawCircle(float x, float y, float radius, float arcLen = 7.f);
		void DrawCircle(const Math::Vector2& c, float radius, float arcLen = 7.f) { DrawCircle(c.GetX(), c.GetY(), radius, arcLen); }
		void DrawEllipse(float x, float y, float rx, float ry, float arcLen = 7.f);
		void DrawEllipse(const Math::Vector2& c, float rx, float ry, float arcLen = 7.f) { DrawEllipse(c.GetX(), c.GetY(), rx, ry, arcLen); }
		
		void FillCircle(float x, float y, float radius, float arcLen = 7.f);
		void FillEllipse(float x, float y, float rx, float ry, float arcLen = 7.f);

		// aliases
		void DrawLine(const Math::Vector2& a, const Math::Vector2& b) {	DrawLine(a.GetX(), a.GetY(), b.GetX(), b.GetY()); }
		void DrawRect(const Math::Vector2& a, const Math::Vector2& b) {	DrawRect(a.GetX(), a.GetY(), b.GetX(), b.GetY()); }

		void DrawString(float x, float y, const char* text);

		void DrawFormattedString(float x, float y, const char* text, ...);

		void Translate(float x, float y);

		void Rotate(float angle);

		void Scale(float s, float t);

		void PushMatrix();

		void PopMatrix();

		void SetColor(uint32 color);
		void SetBgColor(uint32 color);

		void SetFiltering(Texture::FilteringType val);

		void Viewport();

		void SetContext();

#ifndef FIXED_PIPELINE
		void PrefilterTexture(Texture* src, Texture* dst, float scale);
		void SetRenderTarget(Texture* dst);
#endif

		int GetWidth() const { return w; }
		int GetHeight() const { return h; }

		void ResetTransform();
		Math::Matrix4 GetTransform() const;
		void SetTransform(const Math::Matrix4& m);

		void DrawLineMesh(const LineMesh& mesh);

		void SetLineWidth(float val);
		void SetLineStyle(int val);
		float GetLineWidth() const { return lineWidth; }

	protected:

		void InitPF(void);
};

#if (RENDERER == OPENGL) || (RENDERER == OPENGLES)
#	include "Graphics2DGL.inl"
#elif (RENDERER == DIRECTX)
#	include "Graphics2DDX9.inl"
#elif (RENDERER == GDIPLUS)
#	include "Graphics2DGDI.inl"
#endif

#endif // GRAPHICS2D_H
