#ifndef COLOR_H
#define COLOR_H

#include <Math/Utils.h>

union GRB
{
	struct
	{
		uint8 g, r, b;
	};
	uint8 c[3];

	GRB() { }
	GRB(uint8 x, uint8 y, uint8 z) : g(x), r(y), b(z) { }
	GRB(uint32 x) : r((x >> 16) & 0xff), g((x >> 8) & 0xff), b(x & 0xff) { }
	uint32 GetCode() const { return 0xff000000 | (r << 16) | (g << 8) | b; }
};

union GRBA
{
	struct
	{
		uint8 g, r, b, a;
	};
	uint8 c[4];

	GRBA() { }
	GRBA(uint8 x, uint8 y, uint8 z, uint8 w) : g(x), r(y), b(z), a(w) { }
	void Set(uint8 x, uint8 y, uint8 z, uint8 w) { g = x; r = y; b = z; a = w; }
};

struct GRBf
{
	union
	{
		struct
		{
			float g, r, b;
		};
		float c[3];
	};

	GRBf() : g(0), r(0), b(0) { }
	GRBf(const GRB& grb) : g(grb.g), r(grb.r), b(grb.b) { }
	
	GRBf& operator =(const GRB& grb)
	{
		g = grb.g;
		r = grb.r;
		b = grb.b;
		return *this;
	}

	GRBf(float val) : g(val), r(val), b(val) { }

	GRBf& operator +=(const GRBf& a)
	{
		for (int i = 0; i <3; i++)
			c[i] += a.c[i];
		return *this;
	}

	void Clamp()
	{
		g = Math::clamp(g, 0.f, 255.f);
		r = Math::clamp(r, 0.f, 255.f);
		b = Math::clamp(b, 0.f, 255.f);
	}

	operator GRB()
	{
		//Clamp();
		GRB ret;
		ret.g = (uint8)g;
		ret.r = (uint8)r;
		ret.b = (uint8)b;
		return ret;
	}
};

inline GRBf operator +(const GRBf& a, const GRBf& b)
{
	GRBf ret;
	for (int i = 0; i < 3; i++)
		ret.c[i] = a.c[i] + b.c[i];
	return ret;
}

inline GRBf operator -(const GRBf& a, const GRBf& b)
{
	GRBf ret;
	for (int i = 0; i < 3; i++)
		ret.c[i] = a.c[i] - b.c[i];
	return ret;
}

inline GRBf operator *(float a, const GRBf& b)
{
	GRBf ret;
	for (int i = 0; i < 3; i++)
		ret.c[i] = a * b.c[i];
	return ret;
}

inline GRBf operator *(const GRBf& b, float a)
{
	GRBf ret;
	for (int i = 0; i < 3; i++)
		ret.c[i] = a * b.c[i];
	return ret;
}

#define REPEAT

struct Resampler
{
	const uint8* in;
	int width, height;
	int pitch;

	// TODO: watch out for 32bit bitmaps!
	Resampler(const uint8* ptr, int w, int h, int p) : in(ptr), width(w), height(h), pitch(p / 3) { }
	
	GRB GetColor(int x, int y)
	{
#ifdef REPEAT
		x = Math::clamp(x, 0, width - 1);
		y = Math::clamp(y, 0, height - 1);
#else
		GRB black;
		black.g = black.r = black.b = 0;
		if (x < 0 || x >= width || y <= 0 || y >= height)
			return black; // TODO: border color
#endif
		const GRB* inPtr = (const GRB*)in;
		return inPtr[x + y * pitch];
	}

	GRBf GetColorF(int x, int y)
	{
		return GetColor(x, y);
	}
};

#endif // COLOR_H