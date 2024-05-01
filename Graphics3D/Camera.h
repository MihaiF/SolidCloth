#ifndef CAMERA_H
#define CAMERA_H

#include <Math/Vector3.h>
#include <Math/Matrix4.h>

class PolarCamera
{
private:
	float r, ux, uy;
	Math::Vector3 offset;

public:
	PolarCamera()
	{
		r = 150.f;
		ux = 0.f;
		uy = 0.f;
		offset.Y() = 40.f;
	}

	Math::Vector3 GetPosition() const 
	{
		float rux = RADIAN(ux);
		float ruy = RADIAN(-uy);
		float sux = sin(rux);
		float cux = cos(rux);
		float suy = sin(ruy);
		float cuy = cos(ruy);
		Math::Vector3 v = Math::Vector3(r * cux * suy, r * sux, r * cux * cuy) + offset;
		return v;
	}

	Math::Vector3 GetViewDir() const
	{
		// FIXME		
		float sux = sin(ux);
		float cux = cos(ux);
		float suy = sin(uy);
		float cuy = cos(uy);
		return -Math::Vector3(r * cux * suy, r * cuy, r * sux * suy);
	}

	void SetPosition(Math::Vector3 v)
	{
		// not implemented
	}

	Math::Matrix4 View() const
	{
		return Math::Matrix4::LookAt(GetPosition(), offset, Math::Vector3(0, 1, 0));
	}

	void Rotate(int dx, int dy)
	{
		uy += 0.5f * dx;
		ux += 0.5f * dy;
	}

	void Translate(float deltaZ, float deltaX, float deltaY)
	{
		r -= deltaZ * 4;
		Math::Vector4 v(deltaX, deltaY, 0, 0);
		Math::Matrix4 V = View();
		Math::Vector4 v1 = V.GetInverseTransform() * v;
		offset += v1;
	}
};

class EulerCamera
{
public:
	EulerCamera() : yaw(0), pitch(0), roll(0), eye(0, 0, 600) { }

	Math::Matrix4 View() const { return Math::Matrix4::Euler(yaw, pitch, roll) * 
		Math::Matrix4::Translation(-eye.X(), -eye.Y(), -eye.Z()); }

	void Rotate(int dx, int dy)
	{
		yaw += RADIAN(0.25f * dx);
		pitch += RADIAN(0.25f * dy);
	}

	void Translate(float deltaZ, float deltaX, float deltaY)
	{
		Math::Matrix4 mat = View();
		Math::Vector3 viewDir(mat.col[0][2], mat.col[1][2], mat.col[2][2]);
		Math::Vector3 slideDir(mat.col[0][0], mat.col[1][0], mat.col[2][0]); // the first column of the transposed rotation matrix
		const float speed = 5.f; // FIXME
		eye += speed * (-deltaZ * viewDir + deltaX * slideDir);
	}

	Math::Vector3 GetPosition() const { return eye; }
	void SetPosition(Math::Vector3 pos) { eye = pos; }

	Math::Vector3 GetViewDir() const
	{
		Math::Matrix4 mat = View();
		return Math::Vector3(mat.col[0][2], mat.col[1][2], mat.col[2][2]);
	}

	float GetYaw() const { return yaw; }
	float GetPitch() const { return pitch; }
	float GetRoll() const { return roll; }

private:
	float yaw, pitch, roll;
	Math::Vector3 eye;
};

typedef EulerCamera Camera;

#endif // CAMERA_H

