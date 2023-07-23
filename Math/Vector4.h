#ifndef VECTOR4_H
#define VECTOR4_H

#include <math.h>
#include "Vector2.h"
#include "Vector3.h"

using namespace Math; // not a very good idea

struct Vector4
{
	union
	{
		float v[4]; // TODO: inherit from template?
		struct
		{
			float x, y, z, w;
		};
	};


	Vector4()
	{
		SetZero();
	}

	Vector4(float a, float b, float c, float d)
	{
		v[0] = a;
		v[1] = b;
		v[2] = c;
		v[3] = d;
	}

	explicit Vector4(float a)
	{
		v[0] = v[1] = v[2] = v[3] = a;
	}

	explicit Vector4(const Vector3& a)
	{
		v[0] = a.X();
		v[1] = a.Y();
		v[2] = a.Z();
		v[3] = 1.0f;
	}

	explicit Vector4(const Vector2& a)
	{
		v[0] = a.GetX();
		v[1] = a.GetY();
		v[2] = 0;
		v[3] = 0;
	}

	operator Vector3() const { return Vector3(v[0], v[1], v[2]); }
	operator Vector2() { return Vector2(v[0], v[1]); }

	void SetZero()
	{
		v[0] = v[1] = v[2] = v[3] = 0.f;
	}

	void Set(float a, float b, float c, float d)
	{
		v[0] = a;
		v[1] = b;
		v[2] = c;
		v[3] = d;
	}

	float operator [](int i) const { return v[i]; }
	float& operator [](int i) { return v[i]; }

	float X() const { return v[0]; }
	float Y() const { return v[1]; }
	float Z() const { return v[2]; }
	float W() const { return v[3]; }
	float& X() { return v[0]; }
	float& Y() { return v[1]; }
	float& Z() { return v[2]; }

	void Add(const Vector4& other)
	{
		for (int i = 0; i < 4; i++)
			v[i] += other[i];
	}

	void Subtract(const Vector4& other)
	{
		for (int i = 0; i < 4; i++)
			v[i] -= other[i];
	}

	Vector4& operator +=(const Vector4& other)
	{
		Add(other);
		return *this;
	}

	Vector4& operator -=(const Vector4& other)
	{
		Subtract(other);
		return *this;
	}

	float Dot(const Vector4& v) const
	{
		return X() * v.X() + Y() * v.Y() + Z() * v.Z();
	}

	void Scale(float s)
	{
		for (int i = 0; i < 4; i++)
			v[i] *= s;
	}

	void Flip()
	{
		for (int i = 0; i < 4; i++)
			v[i] = -v[i];
	}

	float LengthSquared() const
	{
		return Dot(*this);
	}

	float Length() const
	{
		return sqrtf(LengthSquared());
	}

	void Normalize()
	{
		Scale(1.f / Length());
	}

	Vector4 Cross(const Vector4& v)
	{
		return Vector4(Y() * v.Z() - v.Y() * Z(), -X() * v.Z() + Z() * v.X(), X() * v.Y() - v.X() * Y(), 0.f);
	}

};

inline Vector4 operator +(const Vector4& a, const Vector4& b)
{
	Vector4 ret;
	for (int i = 0; i < 4; i++)
		ret[i] = a[i] + b[i];
	return ret;
}

inline Vector4 operator -(const Vector4& a, const Vector4& b)
{
	Vector4 ret;
	for (int i = 0; i < 4; i++)
		ret[i] = a[i] - b[i];
	return ret;
}

inline Vector4 operator *(float s, const Vector4& v)
{
	return Vector4(s * v.X(), s * v.Y(), s * v.Z(), s * v.W());
}

inline Vector4 operator *(const Vector4& v, float s)
{
	return Vector4(s * v.X(), s * v.Y(), s * v.Z(), s * v.W());
}

inline Vector4 operator -(const Vector4& v)
{
	return Vector4(-v.X(), -v.Y(), -v.Z(), -v.W());
}

inline float length(const Vector4& v)
{
	return v.Length();
}

inline float dot(const Vector4& a, const Vector4& b)
{
	return a.Dot(b);
}

#endif // VECTOR4_H

