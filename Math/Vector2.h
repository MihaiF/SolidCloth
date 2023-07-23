#ifndef VECTOR2_H
#define VECTOR2_H

#include <math.h>

//#ifdef _WIN64
//#	define USE_SSE 1
//#else
#	define USE_SSE 0
//#endif

namespace Math
{

	template <typename Real>
	struct Vector2Tpl
	{
		Real x, y;

		Vector2Tpl() : x(0), y(0) { }

		Vector2Tpl(Real a, Real b) : x(a), y(b) { }

		//explicit Vector2Tpl(int a, int b) : x((Real)a), y((Real)b) { }

		explicit Vector2Tpl(Real val) : x(val), y(val) { }

		template <typename Real1>
		/*explicit */Vector2Tpl(const Vector2Tpl<Real1>& other) : x(other.x), y(other.y) { }

		void Set(Real a, Real b)
		{
			x = a;
			y = b;
		}

		void SetZero()
		{
			x = y = 0;
		}

		Real GetX() const
		{
			return x;
		}

		Real GetY() const
		{
			return y;
		}

		Real& operator [](int i)
		{
			// TODO: use union
			if (i == 0)
				return x;
			else
				return y;
		}

		void Add(const Vector2Tpl& v)
		{
			x += v.x;
			y += v.y;
		}

		void SetAdd(const Vector2Tpl& a, const Vector2Tpl& b)
		{
			x = a.x + b.x;
			y = a.y + b.y;
		}

		void Subtract(const Vector2Tpl& v)
		{
			x -= v.x;
			y -= v.y;
		}

		void Multiply(const Vector2Tpl& v)
		{
			x *= v.x;
			y *= v.y;
		}

		void Divide(const Vector2Tpl& v)
		{
			x /= v.x;
			y /= v.y;
		}

		Real Dot(const Vector2Tpl& v) const
		{
			return x * v.x + y * v.y;
		}

		Real Cross(const Vector2Tpl& v) const
		{
			return x * v.y - y * v.x;
		}

		Real Length() const
		{
			return (Real)sqrt(x * x + y * y);
		}

		Real LengthSquared() const
		{
			return x * x + y * y;
		}

		void Scale(Real s)
		{
			x *= s;
			y *= s;
		}

		void Flip()
		{
			x = -x;
			y = -y;
		}

		void Normalize()
		{
			Real len = Length();
			//ASSERT(len != 0); // FIXME
			Scale(1.0f / len);
		}

		bool NormalizeCheck()
		{
			Real len = Length();
			if (len < 1e-10f)
				return false;
			Scale(1.0f / len);
			return true;
		}

		Vector2Tpl& operator +=(const Vector2Tpl& other)
		{
			Add(other);
			return *this;
		}

		Vector2Tpl& operator -=(const Vector2Tpl& other)
		{
			Subtract(other);
			return *this;
		}

		Vector2Tpl& operator *=(Real s)
		{
			Scale(s);
			return *this;
		}

		void Rotate(Real theta)
		{
			const Real c = (Real)cos(theta);
			const Real s = (Real)sin(theta);
			const Real x0 = x;
			x = x * c - y * s;
			y = x0 * s + y * c;
		}

		Vector2Tpl GetPerpendicular() const
		{
			return Vector2Tpl(-y, x);
		}
	};

#if USE_SSE && !defined(ANDROID_NDK)
#	include "Vector2SSE.h"
#else
	typedef float Scalar;
	typedef Vector2Tpl<float> Vector2;
#endif

	//#include "Vector2Pair.h"

	template <typename Real>
	inline Vector2Tpl<Real> operator +(const Vector2Tpl<Real>& a, const Vector2Tpl<Real>& b)
	{
		Vector2Tpl<Real> c = a;
		c.Add(b);
		return c;
	}

	template <typename Real>
	inline Vector2Tpl<Real> operator -(const Vector2Tpl<Real>& a, const Vector2Tpl<Real>& b)
	{
		Vector2Tpl<Real> c = a;
		c.Subtract(b);
		return c;
	}

	inline Vector2 operator /(const Vector2& a, const Vector2& b)
	{
		Vector2 c = a;
		c.Divide(b);
		return c;
	}

	template <typename Real>
	inline Vector2Tpl<Real> operator *(const Vector2Tpl<Real>& v, const Real& s)
	{
		Vector2Tpl<Real> u = v;
		u.Scale(s);
		return u;
	}

	template <typename Real>
	inline Vector2Tpl<Real> operator *(const Real& s, const Vector2Tpl<Real>& v)
	{
		Vector2Tpl<Real> u = v;
		u.Scale(s);
		return u;
	}

	template <typename Real>
	inline Real operator *(const Vector2Tpl<Real>& u, const Vector2Tpl<Real>& v)
	{
		return u.Dot(v);
	}

	template<typename Real>
	inline Vector2Tpl<Real> operator -(const Vector2Tpl<Real>& v)
	{
		return Vector2Tpl<Real>(-v.x, -v.y);
	}


#if USE_SSE && defined(FLOAT_SCALING_OPERATORS)
	inline Vector2 operator *(const Vector2& v, float s)
	{
		Vector2 u = v;
		u.Scale(s);
		return u;
	}

	inline Vector2 operator *(float s, const Vector2& v)
	{
		Vector2 u = v;
		u.Scale(s);
		return u;
	}
#endif

	//inline Vector2 operator *(const Vector2& a, const Vector2& b)
	//{
	//	Vector2 c = a;
	//	c.Multiply(b);
	//	return c;
	//}

	//inline Vector2Pair operator *(float s, const Vector2Pair& vp)
	//{
	//	Vector2Pair ret = vp;
	//	ScalarPair sp(s);
	//	ret.Scale(sp);
	//	return ret;
	//}
	//
	//inline Vector2Pair operator *(const Vector2Pair& vp, float s)
	//{
	//	Vector2Pair ret = vp;
	//	ScalarPair sp(s);
	//	ret.Scale(sp);
	//	return ret;
	//}
	//
	//inline Vector2Pair operator -(const Vector2Pair& a, const Vector2Pair& b)
	//{
	//	Vector2Pair ret = a;
	//	ret.Subtract(b);
	//	return ret;
	//}
	//
	//inline Vector2Pair operator +(const Vector2Pair& a, const Vector2Pair& b)
	//{
	//	Vector2Pair ret = a;
	//	ret.Add(b);
	//	return ret;
	//}

	inline Vector2 Lerp(const Vector2& a, const Vector2& b, float t)
	{
		Vector2 a1 = a; a1.Scale(1 - t);
		Vector2 b1 = b; b1.Scale(t);
		a1.Add(b1);
		return a1;
	}

} // namespace Math

#endif // VECTOR2_H