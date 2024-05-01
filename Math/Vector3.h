#ifndef VECTOR3_H
#define VECTOR3_H

#include <algorithm>

namespace Math 
{

template<typename Real>
struct Vector3T
{
    union 
    {
        struct { Real x, y, z; };
        Real v[3];
    };

	Vector3T()
	{
		SetZero();
	}

	Vector3T(Real a, Real b, Real c)
	{
		v[0] = a;
		v[1] = b;
		v[2] = c;
	}

	explicit Vector3T(Real a)
	{
		v[0] = v[1] = v[2] = a;
	}

	void SetZero()
	{
		v[0] = v[1] = v[2] = 0.f;
	}

	static Vector3T Zero()
	{
		static Vector3T zero(0);
		return zero;
	}

	void Set(Real a, Real b, Real c)
	{
		v[0] = a;
		v[1] = b;
		v[2] = c;
	}

	Real operator [](int i) const { return v[i]; }
	Real& operator [](int i) { return v[i]; }

	Real operator ()(int i) const { return v[i]; }
	Real& operator ()(int i) { return v[i]; }

	Real X() const { return v[0]; }
	Real Y() const { return v[1]; }
	Real Z() const { return v[2]; }
	Real& X() { return v[0]; }
	Real& Y() { return v[1]; }
	Real& Z() { return v[2]; }

	void Add(const Vector3T& other)
	{
		for (int i = 0; i < 3; i++)
			v[i] += other[i];
	}

	void Subtract(const Vector3T& other)
	{
		for (int i = 0; i < 3; i++)
			v[i] -= other[i];
	}

	Vector3T& operator +=(const Vector3T& other)
	{
		Add(other);
		return *this;
	}

	Vector3T& operator -=(const Vector3T& other)
	{
		Subtract(other);
		return *this;
	}

	Vector3T& operator *=(Real s)
	{
		Scale(s);
		return *this;
	}

	Vector3T& operator /=(Real s)
	{
		Scale(Real(1) / s);
		return *this;
	}

	Real Dot(const Vector3T& v) const
	{
		return X() * v.X() + Y() * v.Y() + Z() * v.Z();
	}

	void Scale(Real s)
	{
		for (int i = 0; i < 3; i++)
			v[i] *= s;
	}

	void Scale(const Vector3T& v)
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
	}

	void Flip()
	{
		for (int i = 0; i < 3; i++)
			v[i] = -v[i];
	}

	Real LengthSquared() const
	{
		return Dot(*this);
	}

	Real Length() const
	{
		return (Real)sqrt(LengthSquared());
	}

	void Normalize()
	{
		Real len = Length();
		//ASSERT(len != 0);
		Scale(1.f / len);
	}

	void NormalizeSafe()
	{
		Real len = Length();
		if (len != 0)
			Scale(1.f / len);
	}

	Vector3T Cross(const Vector3T& v) const
	{
		return Vector3T(Y() * v.Z() - v.Y() * Z(), -X() * v.Z() + Z() * v.X(), X() * v.Y() - v.X() * Y());
	}

	Vector3T Perpendicular()
	{
		//Vector3T up(1, 0, 0);
		// TODO: check if dot is close to 1
		Vector3T abs = this->GetAbs();
		int max = abs.GetMaxComponent();
		int max1 = (max + 1) % 3;
		Vector3T up(0);
		up[max1] = 1;
		Vector3T r = this->Cross(up);
		r.Normalize();
		return r;
	}

	Vector3T Reciprocal()
	{
		Vector3T r;
		r.x = (x != 0) ? 1 / x : 0;
		r.y = (y != 0) ? 1 / y : 0;
		r.z = (z != 0) ? 1 / z : 0;
		return r;
	}

	int GetMaxComponent()
	{
		Real max = v[0];
		int idx = 0;
		for (int i = 1; i < 3; i++)
		{
			if (v[i] > max)
			{
				max = v[i];
				idx = i;
			}
		}
		return idx;
	}

	int GetMinComponent()
	{
		Real min = v[0];
		int idx = 0;
		for (int i = 1; i < 3; i++)
		{
			if (v[i] < min)
			{
				min = v[i];
				idx = i;
			}
		}
		return idx;
	}

	Vector3T GetAbs()
	{
		return Vector3T(abs(v[0]), abs(v[1]), abs(v[2]));
	}

	Vector3T Multiply(const Vector3T& u)
	{
		return Vector3T(v[0] * u.v[0], v[1] * u.v[1], v[2] * u.v[2]);
	}
};

template<typename Real>
inline Vector3T<Real> operator +(const Vector3T<Real>& a, const Vector3T<Real>& b)
{
	Vector3T<Real> ret;
	for (int i = 0; i < 3; i++)
		ret[i] = a[i] + b[i];
	return ret;
}

template<typename Real>
inline Vector3T<Real> operator -(const Vector3T<Real>& a, const Vector3T<Real>& b)
{
	Vector3T<Real> ret;
	for (int i = 0; i < 3; i++)
		ret[i] = a[i] - b[i];
	return ret;
}

template<typename Real>
inline Vector3T<Real> operator *(float s, const Vector3T<Real>& v)
{
	return Vector3T<Real>(s * v.X(), s * v.Y(), s * v.Z());
}

template<typename Real>
inline Vector3T<Real> operator *(const Vector3T<Real>& v, float s)
{
	return Vector3T<Real>(s * v.X(), s * v.Y(), s * v.Z());
}

template<typename Real>
inline Vector3T<Real> operator *(double s, const Vector3T<Real>& v)
{
	Real sr(s);
	return Vector3T<Real>(sr * v.X(), sr * v.Y(), sr * v.Z());
}

template<typename Real>
inline Vector3T<Real> operator *(const Vector3T<Real>& v, double s)
{
	return Vector3T<Real>(Real(s * v.X()), Real(s * v.Y()), Real(s * v.Z()));
}

template<typename Real>
inline Real operator *(const Vector3T<Real>& u, const Vector3T<Real>& v)
{
	return u.Dot(v);
}

template<typename Real>
inline Vector3T<Real> operator -(const Vector3T<Real>& v)
{
	return Vector3T<Real>(-v.X(), -v.Y(), -v.Z());
}

template<typename Real>
inline Vector3T<Real> cross(const Vector3T<Real>& v1, const Vector3T<Real>& v2)
{
	Vector3T<Real> ret = v1;
	return ret.Cross(v2);
}

template<typename Real>
inline Real dot(const Vector3T<Real>& v1, const Vector3T<Real>& v2)
{
    Vector3T<Real> ret = v1;
    return ret.Dot(v2);
}

template<typename Real>
inline Real triple(const Vector3T<Real>& v1, const Vector3T<Real>& v2, const Vector3T<Real>& v3)
{
	return v3.Dot(cross(v1, v2));
}

template<typename Real>
inline Vector3T<Real> vmin(const Vector3T<Real>& v1, const Vector3T<Real>& v2)
{
	return Vector3T<Real>(std::min(v1.X(), v2.X()), std::min(v1.Y(), v2.Y()), std::min(v1.Z(), v2.Z()));
}

template<typename Real>
inline Vector3T<Real> vmax(const Vector3T<Real>& v1, const Vector3T<Real>& v2)
{
	return Vector3T<Real>(std::max(v1.X(), v2.X()), std::max(v1.Y(), v2.Y()), std::max(v1.Z(), v2.Z()));
}

typedef Vector3T<float> Vector3;

typedef Vector3 BarycentricCoords;

inline Vector3 DoubleToFloat(const Vector3T<double>& v)
{
	Vector3 vf;
	vf.x = (float)v.x;
	vf.y = (float)v.y;
	vf.z = (float)v.z;
	return vf;
}

inline Vector3 DoubleToFloat(const Vector3T<float>& v)
{
	return v;
}

inline Vector3T<double> FloatToDouble(const Vector3T<float>& v)
{
	Vector3T<double> vd;
	vd.x = (double)v.x;
	vd.y = (double)v.y;
	vd.z = (double)v.z;
	return vd;
}

} // namespace

#endif // VECTOR3_H

