#ifndef CAMERA_H
#define CAMERA_H

#include <Math/Vector3.h>
#include <Math/Matrix4.h>

class PolarCamera
{
private:
	float r, ux, uy;
	Vector3 offset;

public:
	PolarCamera()
	{
		r = 150.f;
		ux = 0.f;
		uy = 0.f;
		offset.Y() = 40.f;
	}

	Matrix4 View()
	{
		return Matrix4::Translation(0, 0, -r) * Matrix4::RotationX(RADIAN(ux)) * Matrix4::RotationY(RADIAN(uy)) * Matrix4::Translation(-offset.X(), -offset.Y(), -offset.Z());
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

	Matrix4 View() const { return Matrix4::Euler(yaw, pitch, roll) * Matrix4::Translation(-eye.X(), -eye.Y(), -eye.Z()); }

	void Rotate(int dx, int dy)
	{
		yaw += RADIAN(0.25f * dx);
		pitch += RADIAN(0.25f * dy);
	}

	void Translate(float deltaZ, float deltaX)
	{
		Matrix4 mat = View();
		Vector3 viewDir(mat.col[0][2], mat.col[1][2], mat.col[2][2]);
		Vector3 slideDir(mat.col[0][0], mat.col[1][0], mat.col[2][0]); // the first column of the transposed rotation matrix
		const float speed = 5.f; // FIXME
		eye += speed * (-deltaZ * viewDir + deltaX * slideDir);
	}

	Vector3 GetPosition() const { return eye; }
	void SetPosition(const Vector3& pos) { eye = pos; }

	Vector3 GetViewDir() const
	{
		Matrix4 mat = View();
		return Vector3(mat.col[0][2], mat.col[1][2], mat.col[2][2]);
	}

	float GetYaw() const { return yaw; }
	float GetPitch() const { return pitch; }
	float GetRoll() const { return roll; }

private:
	float yaw, pitch, roll;
	Vector3 eye;

};

typedef EulerCamera Camera;

#endif // CAMERA_H

