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

	Math::Matrix4 View()
	{
		return Math::Matrix4::Translation(0, 0, -r) * Math::Matrix4::RotationX(RADIAN(ux)) * 
			Math::Matrix4::RotationY(RADIAN(uy)) * Math::Matrix4::Translation(-offset.X(), -offset.Y(), -offset.Z());
	}

	void Rotate(int dx, int dy)
	{
		uy += 0.5f * dx;
		ux += 0.5f * dy;
	}

	void Translate(float deltaZ, float deltaX)
	{
		r -= deltaZ * 4;
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

	void Translate(float deltaZ, float deltaX)
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

