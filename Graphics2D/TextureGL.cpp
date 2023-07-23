#include <Engine/Base.h>

#if (RENDERER == OPENGL) || (RENDERER == OPENGLES)

#include <Graphics2D/Texture.h>
#include <Engine/Engine.h>
#include <Engine/Profiler.h>

#if (RENDERER == OPENGLES)
#define FMT_RGB GL_RGB
#else
#define FMT_RGB GL_BGR
#endif

Texture::FilteringType Texture::filtering = Texture::DEFAULT;
unsigned Texture::texType = GL_TEXTURE_2D;

#ifdef ANDROID_NDK
extern jobject g_pngmgr;
extern JavaVM* g_jvm;
extern JNIEnv *g_env;

bool Texture::LoadAndroid(char const *path)
{
	JNIEnv* env = g_env;

	jclass cls = env->GetObjectClass(g_pngmgr);
	jmethodID mid;

	/* Ask the PNG manager for a bitmap */
	mid = env->GetMethodID(cls, "open", "(Ljava/lang/String;)Landroid/graphics/Bitmap;");
	jstring name = env->NewStringUTF(path);
	jobject png = env->CallObjectMethod(g_pngmgr, mid, name);
	if (png == NULL)
	{
		Printf("[PNG] Couldn't load file %s", path);
		return false;
	}
	env->DeleteLocalRef(name);
	//env->NewGlobalRef(png); // TODO: not needed, just pixel data

	/* Get image dimensions */
	mid = env->GetMethodID(cls, "getWidth", "(Landroid/graphics/Bitmap;)I");
	int width = env->CallIntMethod(g_pngmgr, mid, png);
	mid = env->GetMethodID(cls, "getHeight", "(Landroid/graphics/Bitmap;)I");
	int height = env->CallIntMethod(g_pngmgr, mid, png);

	/* Get pixels */
	jintArray array = env->NewIntArray(width * height);
	//env->NewGlobalRef(array);
	mid = env->GetMethodID(cls, "getPixels", "(Landroid/graphics/Bitmap;[I)V");
	env->CallVoidMethod(g_pngmgr, mid, png, array);
	//env->DeleteGlobalRef(array);

	if (!array)
		return false;
	Printf("%s %d, %d\n", path, width, height);

	jint *pixels = g_env->GetIntArrayElements(array, 0);
	// convert to rgba
	uint8* data = new uint8[width * height * 4];
	for (int i = 0; i < width * height; i++)
	{
		int idx = i << 2;
		data[idx] = (pixels[i] >> 16) & 0xff;
		data[idx + 1] = (pixels[i] >> 8) & 0xff;
		data[idx + 2] = pixels[i] & 0xff;
		data[idx + 3] = pixels[i] >> 24;
	}

	Create(width, height, true);
	GenerateTexture(data, GL_RGBA, 0, width, height, 0);

	delete[] data;
	g_env->ReleaseIntArrayElements(array, pixels, 0);

	//CheckGlError();

	env->DeleteLocalRef(array);

	//jclass cls = env->GetObjectClass(g_pngmgr);
	mid = env->GetMethodID(cls, "close", "(Landroid/graphics/Bitmap;)V");
	env->CallVoidMethod(g_pngmgr, mid, png);
	//env->DeleteGlobalRef(img.png);
	env->DeleteLocalRef(png);
}

//void ReleaseImage(const AndroidImage& img)
//{
//	JNIEnv* env = g_env;
//	env->DeleteGlobalRef(img.array);
//
//	/* Free image */
//	jclass cls = env->GetObjectClass(g_pngmgr);
//	jmethodID mid = env->GetMethodID(cls, "close", "(Landroid/graphics/Bitmap;)V");
//	env->CallVoidMethod(g_pngmgr, mid, img.png);
//	//env->DeleteGlobalRef(img.png);
//}
#endif

void Texture::CreateGradient(Texture& tex, int width, int height, GRB col1, GRB col2)
{
	GRB* proc = new GRB[width * height];
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			proc[x + y * width] = GRB((col1.g * x  + col2.g * (width - x)) / width, 
									(col1.r * x  + col2.r * (width - x)) / width,
									(col1.b * x  + col2.b * (width - x)) / width);
		}
	}
	// TODO: use LoadFromBuffer
	tex.Create(width, height);
	tex.GenerateTexture(proc, FMT_RGB, 0, width, height, 0);
	delete[] proc;
}

void Texture::CreateCircularGradient(Texture& tex, int width, int height, int radius, GRB col1, GRB col2)
{
	//MEASURE_TIME("Gradient");

	// force width to next multiple of 4
	width = ((width >> 2) + 1) << 2;

	GRB* proc = new GRB[width * height];
	// TODO: float
	int cx = width / 2;
	int cy = height / 2;
	GRBf c1(col1);
	GRBf c2(col2);
	// precomputed table (linear gradient)
	//const size_t size = 100; // TODO: = radius
	//GRB table[size];
	//for (size_t i = 0; i < size; i++)
	//{
	//	float t = (float)(i + 1) / (float)size;
	//	GRBf col = t * c1 + (1 - t) * c2;
	//	table[i] = col;
	//}

	GRB fill(255, 255, 255);
	for (int y = 0; y < height; y++)
	{
		int dy = y - cy;
		int dy2 = dy * dy;
		for (int x = 0, dx = -cx, r2 = cx * cx + dy2; x < width; x++, dx++, r2 += 2 * dx + 1)
		{
			// TODO: more optimization (sqrt approx, SSE)
			float r = sqrtf((float)r2);
			float t = min(1.f, r / radius);
			//GRB grb = table[(int)(t * size)];
			GRBf col = t * c1 + (1 - t) * c2;
			proc[x + y * width] = col;
		}
	}
	tex.Create(width, height, true);
	tex.GenerateTexture(proc, FMT_RGB, 0, width, height, 0);
	delete[] proc;
}

void Texture::Create(int w, int h, bool transparent)
{
	Delete();

	width = w;
	height = h;

	glGenTextures(1, &ID); 
	glBindTexture(texType, ID);
	GLenum format = transparent ? GL_RGBA : GL_RGB;
	glTexImage2D(texType, 0, format, w, h, 0, format, GL_UNSIGNED_BYTE, 0);
 
	glTexParameteri(texType, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(texType, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glTexParameterf(texType, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(texType, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

bool Texture::GenerateTexture(void* bits, uint32 format, int level, int w, int h, int maxLevels)
{
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	//MEASURE_TIME("GenerateTexture");
	GLenum dstFormat = format;
#if (RENDERER == OPENGL)
	dstFormat = format == GL_BGRA ? GL_RGBA8 : GL_RGB; // TODO: replace by num components
	if (0/*GLEW_VERSION_4_2 || GLEW_ARB_texture_storage*/)
	{
		if (level == 0)
		{
			if (maxLevels == 1)
			{
				int hw = w;
				int hh = h;
				while (hw > 1 && hh > 1)
				{
					maxLevels++;
					hw /= 2;
					hh /= 2;
				}
			}
			glTexStorage2D(texType, max(1, maxLevels), dstFormat, w, h);
		}
		glTexSubImage2D(texType, level, 0, 0, w, h, format, GL_UNSIGNED_BYTE, bits);
	}
	else
	{
		glTexImage2D(texType, level, dstFormat, w, h, 0, format, GL_UNSIGNED_BYTE, bits);
	}
#else
	if (w > 2048 || h > 2048) // TODO: use max texture size
		return false;
	glTexImage2D(texType, level, dstFormat, w, h, 0, format, GL_UNSIGNED_BYTE, bits);
#endif
	CheckGlError();
	return true;
}

inline int NextPot(int x)
{
	// compute the next highest power of 2 of 32-bit v
	unsigned int v = x; 

	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;

	return v;
}

// TODO: remove as not needed
void Texture::ConvertToPot(void* bits, int w, int h, int& nw, int& nh)
{
	nw = NextPot(w);
	nh = NextPot(h);
	size_t size = nw * nh * 3; // FIXME (only RGB)
	pot = new uint8[size]; // DELETE!!!
	memset(pot, 0, size);
	size_t line = w * 3;
	size_t pitch = nw * 3;
	uint8* src = (uint8*)bits;
	uint8* dst = (uint8*)pot;
	for (int i = 0; i < h; i++)
	{
		memcpy(dst, src, line);
		dst += pitch;
		src += line;
	}
}

bool Texture::LoadTextureFromFile(const char* filename, int level, int maxLevel)
{
	MEASURE_TIME("LoadTextureFromFile");
#ifndef ANDROID_NDK
	//image format
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	//pointer to the image data
	BYTE* bits(0);
	
	//check the file signature and deduce its format
	fif = FreeImage_GetFileType(filename, 0);
	//if still unknown, try to guess the file format from the file extension
	if(fif == FIF_UNKNOWN) 
		fif = FreeImage_GetFIFFromFilename(filename);
	//if still unkown, return failure
	if(fif == FIF_UNKNOWN)
		return false;

	//check that the plugin has reading capabilities and load the file
	if(FreeImage_FIFSupportsReading(fif))
		dib = FreeImage_Load(fif, filename);
	//if the image failed to load, return failure
	if(!dib)
		return false;

	//retrieve the image data
	bits = FreeImage_GetBits(dib);
	//get the image width and height
	int pitch, line;
	if (level == 0)
	{
		width = FreeImage_GetWidth(dib);
		height = FreeImage_GetHeight(dib);
		pitch = FreeImage_GetPitch(dib);
		line = FreeImage_GetLine(dib);
	}
	//if this somehow one of these failed (they shouldn't), return failure
	if((bits == 0) || (width == 0) || (height == 0))
		return false;

	unsigned bpp = FreeImage_GetBPP(dib);
#if (RENDERER == OPENGL) // rather android
	GLenum format = GL_BGRA;
	if (bpp == 24)
	{
		format = GL_BGR;
	}
#else
	GLenum format = GL_RGBA;
	if (bpp == 24)
	{
		format = GL_RGB;
	}
#endif

	GenerateTexture(bits, format, level, FreeImage_GetWidth(dib), FreeImage_GetHeight(dib), maxLevel);
	// TODO: free image object when not used for filtering
#elif defined(USE_GDIPLUS)
	wchar_t ucFilename[100];
	const int len = (int)strlen(filename) + 1;
	MultiByteToWideChar(CP_ACP, 0, filename, len, ucFilename, len);

	bmp = new Gdiplus::Bitmap(ucFilename);
	if (bmp->GetLastStatus() != Gdiplus::Status::Ok)
		return false;
	Gdiplus::Rect rect(0, 0, bmp->GetWidth(), bmp->GetHeight());
	Gdiplus::PixelFormat pf = bmp->GetPixelFormat();
#if (RENDERER == OPENGL) // rather android
	uint32 format = GL_BGRA;
	if (pf == PixelFormat24bppRGB)
	{
		format = GL_BGR;
	}
#else
	GLenum format = GL_RGBA;
	if (if (pf == PixelFormat24bppRGB))
	{
		format = GL_RGB;
	}
#endif
	//Gdiplus::BitmapData data;
	bmp->LockBits(&rect, Gdiplus::ImageLockModeRead, pf, &data);
	if (level == 0)
	{
		width = data.Width;
		height = data.Height;
	}
	uint8* bits = (uint8*)data.Scan0;
	GenerateTexture(data.Scan0, format, level, data.Width, data.Height, maxLevel);
	//bmp->UnlockBits(&data);
#endif // ANDROID_NDK
	
	if (filtering != Texture::BICUBIC)
	{
		ReleaseBits();
		return true;
	}
#endif

	ReleaseBits();

	CheckGlError();
	return true;
}

void Texture::SaveToFile(const char* filename)
{
	glBindTexture(GL_TEXTURE_2D, ID);
	GLubyte* bits = new GLubyte[width * height * 3];
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, bits);
	FIBITMAP* image = FreeImage_ConvertFromRawBits(bits, width, height,
		3 * width, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);
	FreeImage_Save(FIF_BMP, image, filename, 0);
	delete[] bits;
}

// TODO: move to platform.h?
bool FileExists(const char* szPath)
{
#ifdef _WIN32
	DWORD dwAttrib = GetFileAttributesA(szPath);
	return (dwAttrib != INVALID_FILE_ATTRIBUTES && !(dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
#else
	return true;
#endif
}

bool Texture::LoadInfo(const char* filename)
{
#if (RENDERER == OPENGL) && !defined(_WIN64)
	//image format
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	
	//check the file signature and deduce its format
	fif = FreeImage_GetFileType(filename, 0);
	//if still unknown, try to guess the file format from the file extension
	if(fif == FIF_UNKNOWN) 
		fif = FreeImage_GetFIFFromFilename(filename);
	//if still unkown, return failure
	if(fif == FIF_UNKNOWN)
		return false;

	//check that the plugin has reading capabilities and load the file
	if(FreeImage_FIFSupportsReading(fif))
		dib = FreeImage_Load(fif, filename);
	//if the image failed to load, return failure
	if(!dib)
		return false;

	//get the image width and height
	width = FreeImage_GetWidth(dib);
	height = FreeImage_GetHeight(dib);
	return true;
#else
	return false;
#endif
}

bool Texture::LoadTexture(const char* filename)
{
	MEASURE_TIME("LoadTexture");

#ifdef ANDROID_NDK
	//AndroidImage img = LoadImage(filename);
	//if (!img.array)
	//	return false;

	//jint *pixels = g_env->GetIntArrayElements(img.array, 0);
	//// convert to bgr
	//uint8* data = new uint8[img.w * img.h * 4];
	//for (int i = 0; i < img.w * img.h; i++)
	//{
	//	int idx = i << 2;
	//	data[idx] = (pixels[i] >> 16) & 0xff;
	//	data[idx + 1] = (pixels[i] >> 8) & 0xff;
	//	data[idx + 2] = pixels[i] & 0xff;
	//	data[idx + 3] = pixels[i] >> 24;
	//}

	//Create(img.w, img.h, true);
	//GenerateTexture(data, GL_RGBA, 0, img.w, img.h, 0);
	//
	//g_env->ReleaseIntArrayElements(img.array, pixels, 0);
	//delete[] data;
	//ReleaseImage(img);
	//CheckGlError();
	//return true;
	return LoadAndroid(filename);
#else

	Delete();

	// common stuff
	glGenTextures(1, &ID);
#ifdef FIXED_PIPELINE
	//glEnable(texType);
#endif
	bool mipmap = filtering == TRILINEAR;
	if (mipmap && texType != GL_TEXTURE_2D)
	{
		texType = GL_TEXTURE_2D;
	}

	glBindTexture(texType, ID);

	int level = 1;
	std::vector<std::string> levelFiles;
	while (mipmap)
	{
		std::string path(filename);
		size_t pos = path.find_last_of(".");
		std::string newPath = path.substr(0, pos);
		newPath += "_level";
		newPath += '0' + level; // TODO: itoa
		newPath += path.substr(pos);
		if (!FileExists(newPath.c_str()))
			break;
		levelFiles.push_back(newPath);
		level++;
	}

	if (!LoadTextureFromFile(filename, 0, mipmap ? level : 0)) // 0 means don't allocate storage for mipmaps, 1 generate mipmaps, >1 load mipmaps
	{
		glDeleteTextures(1, &ID);
		ID = 0;
		return false;
	}

	glTexParameteri(texType, GL_TEXTURE_MAG_FILTER, filtering == NEAREST ? GL_NEAREST : GL_LINEAR);
	glTexParameteri(texType, GL_TEXTURE_MIN_FILTER, filtering == NEAREST ? GL_NEAREST : GL_LINEAR);
#if (RENDERER == OPENGL)
	glTexParameteri(texType, GL_TEXTURE_BASE_LEVEL, 0);
#endif

	// TODO: is this what I want?
	glTexParameterf(texType, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(texType, GL_TEXTURE_WRAP_T, GL_REPEAT);

	CheckGlError();

#if (RENDERER == OPENGL)
	if (mipmap)
	{
		if (level == 1) // generate mipmap
		{
			if (GLEW_VERSION_3_0 || GLEW_ARB_framebuffer_object)
			{
				// method 2 - slower than 1 it seems
				glHint(GL_GENERATE_MIPMAP_HINT, GL_NICEST);
				glGenerateMipmap(texType);
			}
			else if (GLEW_SGIS_generate_mipmap && GLEW_ARB_texture_non_power_of_two)
			{
				// method 1 - legacy / deprecated in 3.1 - breaks on at least one machine
				glTexParameteri(texType, GL_GENERATE_MIPMAP, GL_TRUE); // TODO: check error
			}
			//else
			//{
			//	// method 3 (not recommended)
			//	gluBuild2DMipmaps(texType, 3, width, height, format, GL_UNSIGNED_BYTE, bits); // TODO: double check the 2nd param
			//}
		}
		else
		{
			// method 4: manual
			for (int i = 1; i < level; i++)
			{
				LoadTextureFromFile(levelFiles[i - 1].c_str(), i, level);
			}
			glTexParameteri(texType, GL_TEXTURE_MAX_LEVEL, level - 1);
		}

		glTexParameteri(texType, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	}
	else
	{
		glTexParameteri(texType, GL_TEXTURE_MIN_FILTER, filtering == NEAREST ? GL_NEAREST : GL_LINEAR);
		glTexParameteri(texType, GL_TEXTURE_MAX_LEVEL, 0);
	}	
#endif // OPENGL

	CheckGlError();
	return true;
#endif // ANDROID_NDK
}

void Texture::LoadFromBuffer(int w, int h, void* bits)
{
	Delete();
	glGenTextures(1, &ID);
	glBindTexture(GL_TEXTURE_2D, ID);
	width = w;
	height = h;
	GenerateTexture(bits, GL_BGR, 0, w, h, 0);
	glTexParameteri(texType, GL_TEXTURE_MAG_FILTER, filtering == NEAREST ? GL_NEAREST : GL_LINEAR);
	glTexParameteri(texType, GL_TEXTURE_MIN_FILTER, filtering == NEAREST ? GL_NEAREST : GL_LINEAR);
	glTexParameteri(texType, GL_TEXTURE_BASE_LEVEL, 0);
	glTexParameterf(texType, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(texType, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(texType, GL_TEXTURE_MIN_FILTER, filtering == NEAREST ? GL_NEAREST : GL_LINEAR);
	glTexParameteri(texType, GL_TEXTURE_MAX_LEVEL, 0);

	CheckGlError();
}
