#ifndef TEXTURE_H
#define TEXTURE_H

#include <xmmintrin.h> // added to fix the build

#include <Graphics2D\Graphics.h>
#include <Engine/Types.h>
#include <Graphics2D/Color.h> // TODO: move color in different file
//#include <Graphics2D/ImageProcessing.h>

#define RESCALE // TODO: remove

class Texture
{
public:
	enum FilteringType
	{
		DEFAULT,
		NEAREST,
		BILINEAR,
		TRILINEAR,
		BICUBIC,
		HQ_BILINEAR,
	};

public:
#if (RENDERER == OPENGL) || (RENDERER == OPENGLES)
	GLuint ID;
#elif (RENDERER == DIRECTX)
	LPDIRECT3DTEXTURE9 D3DTexture;
	D3DXIMAGE_INFO D3DTextureInfo;
#elif (RENDERER == GDIPLUS)
	mutable Gdiplus::CachedBitmap* cache = nullptr; // dirty hack
	Gdiplus::Bitmap* bmp = nullptr;
	Gdiplus::BitmapData data;
	static Gdiplus::Graphics* graphics;
#endif
	static unsigned texType; // TODO: move to Graphics2D

private:
	int width, height;
	uint8 alpha;
	static FilteringType filtering;
#if (RENDERER == OPENGL) || ((RENDERER == OPENGLES) && !defined(ANDROID_NDK))
	//pointer to the image, once loaded
	FIBITMAP *dib;
#endif
#ifdef WIN32
	__m128* sseData;
#endif
	uint8* pot;

public:
	Texture() : alpha(255), width(0), height(0)
	{
#ifdef WIN32
		sseData = NULL;
#endif
#if (RENDERER == OPENGL)
		dib = NULL;
		ID = 0;
#endif
	}

	// TODO: copy constructor

	~Texture() 
	{ 
#if (RENDERER == GDIPLUS)
		if (bmp != nullptr)
			delete bmp;
		if (cache != nullptr)
			delete cache;
#endif
#ifdef WIN32
		if (sseData)
			_aligned_free(sseData);
#endif
		Delete();
	}

	void Delete()
	{
#if (RENDERER == OPENGL)
		if (ID > 0)
			glDeleteTextures(1, &ID);
		if (dib)
			FreeImage_Unload(dib);
#endif
	}

	bool LoadTexture(const char* filename); // TODO: rename to LoadFromFile
	bool LoadInfo(const char* filename);
	bool LoadFromDDS(const char* filename);

	void SaveToFile(const char* filename);

	void Create(int w, int h, bool transparent = false);
	void LoadFromBuffer(int w, int h, void* bits);

	int Width() const { return width; } // TODO: get

	int Height() const { return height; }

	void SetAlpha(uint8 val) { alpha = val; }
	uint8 GetAlpha() const { return alpha; }

	static void SetFiltering(FilteringType val) { filtering = val; }
	static FilteringType GetFiltering(FilteringType val) { return filtering; }

	//void Filter(float scale, ImageFilter filter, int iterations = 1, bool separable = true);

	static void CreateGradient(Texture& tex, int width, int height, GRB col1, GRB col2); // TODO: Color struct
	static void CreateCircularGradient(Texture& tex, int width, int height, int radius, GRB col1, GRB col2);

#if (RENDERER == OPENGLES)
	uint8* GetBits() { return NULL; }
	void ReleaseBits() { }
	int GetOriginalWidth() const { return -1; }
	int GetOriginalHeight() const { return -1; }
#elif (RENDERER == OPENGL)
	uint8* GetBits() { return FreeImage_GetBits(dib); }
	void ReleaseBits() { }
	int GetOriginalWidth() const { return FreeImage_GetWidth(dib); }
	int GetOriginalHeight() const { return FreeImage_GetHeight(dib); }
#elif (RENDERER == GDIPLUS)
	uint8* GetBits()
	{
		Gdiplus::Rect rect(0, 0, bmp->GetWidth(), bmp->GetHeight());
		Gdiplus::PixelFormat pf = bmp->GetPixelFormat();
		bmp->LockBits(&rect, Gdiplus::ImageLockModeRead, pf, &data);
		return (uint8*)data.Scan0;
	}

	void ReleaseBits()
	{
		bmp->UnlockBits(&data);
	}

	int GetOriginalWidth() const { return bmp->GetWidth(); }
	int GetOriginalHeight() const { return bmp->GetHeight(); }
#endif

	bool LoadAndroid(char const *path);

private:
	bool LoadTextureFromFile(const char* filename, int level, int maxLevels);
	bool GenerateTexture(void* bits, uint32 format, int level, int w, int h, int maxLevels);
	void ConvertToPot(void* bits, int w, int h, int& nw, int& nh);

	friend class OpenGLSData; // FIXME
};

#ifdef ANDROID_NDK
#include <jni.h>

//struct AndroidImage
//{
//	jintArray array;
//	jobject png;
//	int w, h;
//
//	AndroidImage() : array(NULL) { }
//};
//
//AndroidImage LoadImage(char const *path);
//void ReleaseImage(const AndroidImage& img);

#endif

#endif //TEXTURE_H
