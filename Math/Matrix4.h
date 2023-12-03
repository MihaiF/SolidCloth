#ifndef MATRIX4_H
#define MATRIX4_H

#include "Vector3.h"
#include "Matrix3.h"
#include "Vector4.h"
#include "Utils.h"

using namespace Math; // FIXME: not a very good idea

// TODO: namespace Math

struct Matrix4;
static Matrix4 operator *(const Matrix4& a, const Matrix4& b);

// A 4 by 4 matrix structure - column major
// Homogeneous coordinates transforms have this structure:
// r1.x r2.x r3.x t.x
// r1.y r2.y r3.y t.y
// r1.z r2.y r3.z t.z
// 0    0    0    1
struct Matrix4
{
	Vector4 col[4]; // column vector

	Matrix4()
	{
		// important to initialize to zero!
	}

	const float* GetData() const { return &col[0].x; }
	float* GetWriteData() { return &col[0].x; }

	float GetValue(int i, int j) const { return col[j][i]; }

	Matrix4(const Vector3& r1, const Vector3& r2, const Vector3& r3, const Vector3& t)
	{
		col[0] = Vector4(r1);
		col[1] = Vector4(r2);
		col[2] = Vector4(r3);
		col[3] = Vector4(t.x, t.y, t.z, 1);
	}

	Matrix4(const Matrix3& mat, const Vector3& v)
	{		
		col[0][0] = mat.m[0][0];
		col[0][1] = mat.m[1][0];
		col[0][2] = mat.m[2][0];
		col[3][0] = v.X();
		col[1][0] = mat.m[0][1];
		col[1][1] = mat.m[1][1];
		col[1][2] = mat.m[2][1];
		col[3][1] = v.Y();
		col[2][0] = mat.m[0][2];
		col[2][1] = mat.m[1][2];
		col[2][2] = mat.m[2][2];
		col[3][2] = v.Z();
		col[0][3] = 0;
		col[1][3] = 0;
		col[2][3] = 0;
		col[3][3] = 1;
	}

	Vector3 GetTranslation() const
	{
		Vector3 v = col[3];
		return v;
	}

	void SetTranslation(const Vector3& t)
	{
		col[3] = Vector4(t);
	}

	Vector3 GetScale() const
	{
		// TODO: use vector length for the general case
		Vector3 s;
		s.x = col[0][0];
		s.y = col[1][1];
		s.z = col[2][2];
		return s;
	}

	Vector3 Transform(const Vector3& v) const
	{
		Vector3 r;
		r.X() = v.X() * col[0][0] + v.Y() * col[1][0] + v.Z() * col[2][0] + col[3][0];
		r.Y() = v.X() * col[0][1] + v.Y() * col[1][1] + v.Z() * col[2][1] + col[3][1];
		r.Z() = v.X() * col[0][2] + v.Y() * col[1][2] + v.Z() * col[2][2] + col[3][2];
		return r;
	}

	Vector3 TransformRay(const Vector3& v) const
	{
		Vector3 r;
		r.X() = v.X() * col[0][0] + v.Y() * col[1][0] + v.Z() * col[2][0];
		r.Y() = v.X() * col[0][1] + v.Y() * col[1][1] + v.Z() * col[2][1];
		r.Z() = v.X() * col[0][2] + v.Y() * col[1][2] + v.Z() * col[2][2];
		return r;
	}

	Matrix4 GetInverseTransform() const
	{
		// inverse of affine transformation: https://stackoverflow.com/questions/2624422/efficient-4x4-matrix-inverse-affine-transform
		Matrix3 M(col[0][0], col[1][0], col[2][0],
				col[0][1], col[1][1], col[2][1],
				col[0][2], col[1][2], col[2][2]);
		Matrix3 Minv = M.GetInverse();
		Vector3 t(-col[3].x, -col[3].y, -col[3].z);
		return Matrix4(Minv, Minv * t);
	}

	Matrix4 GetTranspose() const
	{
		return Transposed(*this);
	}

	static Matrix4 Transposed(const Matrix4& a)
	{
		Matrix4 b;
		b.col[0][0] = a.col[0][0];
		b.col[0][1] = a.col[1][0];
		b.col[0][2] = a.col[2][0];
		b.col[0][3] = a.col[3][0];
		b.col[1][0] = a.col[0][1];
		b.col[1][1] = a.col[1][1];
		b.col[1][2] = a.col[2][1];
		b.col[1][3] = a.col[3][1];
		b.col[2][0] = a.col[0][2];
		b.col[2][1] = a.col[1][2];
		b.col[2][2] = a.col[2][2];
		b.col[2][3] = a.col[3][2];
		b.col[3][0] = a.col[0][3];
		b.col[3][1] = a.col[1][3];
		b.col[3][2] = a.col[2][3];
		b.col[3][3] = a.col[3][3];
		return b;
	}

	static Matrix4 Identity()
	{
		Matrix4 I;
		I.col[0][0] = 1;
		I.col[1][1] = 1;
		I.col[2][2] = 1;
		I.col[3][3] = 1;
		return I;
	}

	static Matrix4 Perspective(float fov, float aspect, float n, float f)
	{
		// TODO: variant with 1/d
		float d = 1.f / tanf(RADIAN(fov) * 0.5f);
		Matrix4 P;
		P.col[0][0] = d / aspect;
		P.col[1][1] = d;
		P.col[2][2] = (n + f) / (n - f);
		P.col[3][2] = 2 * n * f/ (n - f);
		P.col[2][3] = -1;
		return P;
	}

	static Matrix4 Ortographic(float left, float top, float right, float bottom, float n, float f)
	{
		Matrix4 P;
		P.col[0][0] = 2.f / (right - left);
		P.col[3][0] = (right + left) / (right - left);
		P.col[1][1] = 2.f / (top - bottom);
		P.col[3][1] = (top + bottom) / (top - bottom);
		P.col[2][2] = -2.f / (f - n);
		P.col[3][2] = (f + n) / (f - n);
		P.col[3][3] = 1;
		return P;
	}

	static Matrix4 Translation(float x, float y, float z)
	{
		Matrix4 T = Identity();
		T.col[3] = Vector4(x, y, z, 1);
		return T;
	}

	static Matrix4 RotationX(float u)
	{
		float c = cosf(u);
		float s = sinf(u);
		Matrix4 R = Matrix4::Identity();
		R.col[1][1] = c;
		R.col[2][1] = -s;
		R.col[1][2] = s;
		R.col[2][2] = c;
		return R;
	}

	static Matrix4 RotationY(float u)
	{
		float c = cosf(u);
		float s = sinf(u);
		Matrix4 R = Matrix4::Identity();
		R.col[0][0] = c;
		R.col[2][0] = s;
		R.col[0][2] = -s;
		R.col[2][2] = c;
		return R;
	}

	static Matrix4 RotationZ(float u)
	{
		float c = cosf(u);
		float s = sinf(u);
		Matrix4 R = Matrix4::Identity();
		R.col[0][0] = c;
		R.col[1][0] = -s;
		R.col[0][1] = s;
		R.col[1][1] = c;
		return R;
	}

	static Matrix4 Scale(float a, float b, float c)
	{
		Matrix4 S;
		S.col[0][0] = a;
		S.col[1][1] = b;
		S.col[2][2] = c;
		S.col[3][3] = 1;
		return S;
	}

	static Matrix4 Euler(float yaw, float pitch, float roll)
	{
		return RotationZ(roll) * RotationX(pitch) * RotationY(yaw);
	}

	static Matrix4 LookAt(const Vector3& eye, const Vector3& center, const Vector3& up);
};

inline Matrix4 operator *(const Matrix4& a, const Matrix4& b)
{
	Matrix4 c; // initialized to zero
	for (int i = 0; i < 4; i++) // for each row i of the result or of matrix A
	{
		for (int j = 0; j < 4; j++) // for each column j of the result or of matrix B
		{
			for (int k = 0; k < 4; k++) // for each common column of A or row of B
			{
				c.col[j][i] += a.col[k][i] * b.col[j][k];
			}
		}
	}
	return c;
}

inline Vector4 operator *(const Matrix4& a, const Vector4& b)
{
	Vector4 c; // initialized to zero
	for (int i = 0; i < 4; i++) // for each row i of the result or of matrix A
	{
		for (int k = 0; k < 4; k++) // for each row of vector b
		{
			c[i] += a.col[k][i] * b[k];
		}
	}
	return c;
}

inline Matrix4 Matrix4::LookAt(const Vector3& eye, const Vector3& center, const Vector3& up)
{
	Vector3 z = eye - center;
	z.Normalize();
	Vector3 x = z.Cross(up);
	x.Normalize();
	Vector3 y = x.Cross(z);
	Matrix4 lookAt;// (x, y, z, Vector3());
	lookAt.col[0][0] = x.x;
	lookAt.col[0][1] = y.x;
	lookAt.col[0][2] = z.x;
	lookAt.col[1][0] = x.y;
	lookAt.col[1][1] = y.y;
	lookAt.col[1][2] = z.y;
	lookAt.col[2][0] = x.z;
	lookAt.col[2][1] = y.z;
	lookAt.col[2][2] = z.z;
	lookAt.col[3][0] = -x.Dot(eye);
	lookAt.col[3][1] = -y.Dot(eye);
	lookAt.col[3][2] = -z.Dot(eye);
	lookAt.col[3][3] = 1.f;
	return lookAt;
}

#endif // MATRIX4_H

