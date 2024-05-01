#pragma once

#include <Geometry/Collision3D.h>
#include <Geometry/Mesh.h>
#include <Engine/Profiler.h>

#include <functional>
#include <cmath>

namespace Geometry
{

class SDF
{
public:
	enum CombineOp
	{
		CO_UNION,
		CO_UNION_SMOOTHED,
		CO_UNION_ROUNDED,
		CO_DIFFERENCE1,
		CO_DIFFERENCE2,
		CO_INTERSECT,
		CO_SMOOTH_UNION,
		CO_BLEND
	};

public:
	void Create(int nx, int ny, int nz, const Geometry::Aabb3& box);

	void Create(int res[], const Geometry::Aabb3& box);

	void CreateFromImplicit(std::function<float(Math::Vector3 pos)> eval);

	void CreateFromMesh(const Geometry::Mesh& mesh, const struct AabbTree* tree = nullptr);

	void WriteToFile(const char* path);

	void LoadFromFile(const char* path);

	void Blur();

	void ComputeGradient();

	void ComputeGradientParallel(float narrowBand = FLT_MAX);

	Math::Vector3 GetSteps() const { return mSteps; }

	int Index(int x, int y, int z) const
	{
		return x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1];
	}

	size_t GetCount() { return mSDF.size(); }

	void GetCoords(int idx, int& x, int& y, int& z)
	{
		x = idx % numValsPerSide[0];
		int w = idx / numValsPerSide[0];
		y = w % numValsPerSide[1];
		z = w / numValsPerSide[1];
	}

	Math::Vector3 GetPosAt(int x, int y, int z)
	{
		Math::Vector3 org((float)x, (float)y, (float)z);
		org.Scale(mSteps);
		org += mBox.min;
		return org;
	}

	float GetValue(int x, int y, int z) const
	{
		if (x > numCellsPerSide[0] || y > numCellsPerSide[1] || z > numCellsPerSide[2] || x < 0 || y < 0 || z < 0)
			return 0; // it should have been FLT_MAX, but we return 0 for the trilinear interpolation
		return mSDF[Index(x, y, z)];
	}

	void SetValue(int x, int y, int z, float val)
	{
		if (x > numCellsPerSide[0] || y > numCellsPerSide[1] || z > numCellsPerSide[2] || x < 0 || y < 0 || z < 0)
			return;
		mSDF[Index(x, y, z)] = val;
	}

	float GetValue(int idx) const
	{
		return mSDF[idx];
	}

	Math::Vector3 GetGrad(int x, int y, int z) const
	{
		if (x > numCellsPerSide[0] || y > numCellsPerSide[1] || z > numCellsPerSide[2] || x < 0 || y < 0 || z < 0)
			return Math::Vector3::Zero();
		return mGrad[Index(x, y, z)];
	}

	void SetGrad(int x, int y, int z, Math::Vector3 grad)
	{
		if (x > numCellsPerSide[0] || y > numCellsPerSide[1] || z > numCellsPerSide[2] || x < 0 || y < 0 || z < 0)
			return;
		mGrad[Index(x, y, z)] = grad;
	}

	void GetGridCoords(Math::Vector3 pos, int& x, int& y, int& z,
		float& fx, float& fy, float& fz) const
	{
		float rx = (pos.x - mBox.min.x) / mSteps.x;
		float ry = (pos.y - mBox.min.y) / mSteps.y;
		float rz = (pos.z - mBox.min.z) / mSteps.z;
		x = (int)std::floorf(rx);
		y = (int)std::floorf(ry);
		z = (int)std::floorf(rz);
		fx = fabs(rx - x);
		fy = fabs(ry - y);
		fz = fabs(rz - z);
	}

	Math::Vector3 GetGrad(Math::Vector3 pos) const
	{
		int x, y, z;
		float fx, fy, fz;
		GetGridCoords(pos, x, y, z, fx, fy, fz);

		if (x > numCellsPerSide[0] || y > numCellsPerSide[1] || z > numCellsPerSide[2] || x < 0 || y < 0 || z < 0)
			return Math::Vector3::Zero();

		return (1.f - fx) * (1.f - fy) * (1.f - fz) * GetGrad(x, y, z) +
			(1.f - fx) * (1.f - fy) * fz * GetGrad(x, y, z + 1) +
			(1.f - fx) * fy * (1.f - fz) * GetGrad(x, y + 1, z) +
			(1.f - fx) * fy * fz * GetGrad(x, y + 1, z + 1) +
			fx * (1.f - fy) * (1.f - fz) * GetGrad(x + 1, y, z) +
			fx * (1.f - fy) * fz * GetGrad(x + 1, y, z + 1) +
			fx * fy * (1.f - fz) * GetGrad(x + 1, y + 1, z) +
			fx * fy * fz * GetGrad(x + 1, y + 1, z + 1);
	}

	float GetValue(Math::Vector3 pos) const
	{
		int x, y, z;
		float fx, fy, fz;
		GetGridCoords(pos, x, y, z, fx, fy, fz);

		if (x > numCellsPerSide[0] || y > numCellsPerSide[1] || z > numCellsPerSide[2] || x < 0 || y < 0 || z < 0)
			return FLT_MAX;

		return (1.f - fx) * (1.f - fy) * (1.f - fz) * GetValue(x, y, z) +
			(1.f - fx) * (1.f - fy) * fz * GetValue(x, y, z + 1) +
			(1.f - fx) * fy * (1.f - fz) * GetValue(x, y + 1, z) +
			(1.f - fx) * fy * fz * GetValue(x, y + 1, z + 1) +
			fx * (1.f - fy) * (1.f - fz) * GetValue(x + 1, y, z) +
			fx * (1.f - fy) * fz * GetValue(x + 1, y, z + 1) +
			fx * fy * (1.f - fz) * GetValue(x + 1, y + 1, z) +
			fx * fy * fz * GetValue(x + 1, y + 1, z + 1);
	}

	const std::vector<float>& GetData() const { return mSDF; }
	std::vector<float>& GetData() { return mSDF; }

	const Geometry::Aabb3& GetBox() const { return mBox; }

	void GetNumCellsPerSide(int precision[]) const
	{
		precision[0] = numCellsPerSide[0];
		precision[1] = numCellsPerSide[1];
		precision[2] = numCellsPerSide[2];
	}

	void Combine(const SDF& other, CombineOp op);

	void Combine(std::function<float(Math::Vector3 pos)> eval, CombineOp op);

	template<int op, typename FUNC>
	void CombineParallel(FUNC&& eval, float param = 4.25f, float narrowBand = FLT_MAX)
	{
		PROFILE_SCOPE("Combine SDF parallel");

		int count = numValsPerSide[0] * numValsPerSide[1] * numValsPerSide[2];

		#pragma omp parallel for num_threads(3)
		for (int idx = 0; idx < count; idx++)
		{
			float val1 = mSDF[idx];

			int x, y, z;
			GetCoords(idx, x, y, z);
			Vector3 org = GetPosAt(x, y, z);
			float val2 = eval(org);

			if (fabs(val1) > narrowBand && fabs(val2) > narrowBand)
				continue;

			if constexpr (op == CO_UNION)
				mSDF[Index(x, y, z)] = std::min(val1, val2);
			else if constexpr (op == CO_INTERSECT)
				mSDF[Index(x, y, z)] = std::max(val1, val2);
			else if constexpr (op == CO_UNION_SMOOTHED)
			{
				const float k = param;
				float h = std::max(k - fabs(val1 - val2), 0.0f);
				mSDF[Index(x, y, z)] = std::min(val1, val2) - h * h * 0.25f / k;
			}
			else if constexpr (op == CO_UNION_ROUNDED)
			{
				const float r = param;
				float ux = std::max(r - val1, 0.f);
				float uy = std::max(r - val2, 0.f);
				Vector2 u(ux, uy);
				mSDF[Index(x, y, z)] = std::max(r, std::min(val1, val2)) - u.Length();
			}
			else if constexpr (op == CO_SMOOTH_UNION)
				mSDF[Index(x, y, z)] = 2.0 / 3.0 * (val1 + val2 - sqrt(val1 * val1 + val2 * val2 - val1 * val2));
			else if constexpr (op == CO_BLEND)
			{

				float sqr_sum = val1 * val1 + val2 * val2;
				if (sqr_sum > 1e12f)
				{
					mSDF[Index(x, y, z)] = std::min(val1, val2);
				}
				else
				{
					const float weight1 = 1.1f;
					const float weight2 = 1.1f;
					const float blend = param;

					float w1 = val1 / weight1;
					float w2 = val2 / weight2;
					float b = blend / (1.0f + w1 * w1 + w2 * w2);

					mSDF[Index(x, y, z)] = 0.666666f * (val1 + val2 - sqrt(sqr_sum - val1 * val2)) - b;
				}
			}
			else if constexpr (op == CO_DIFFERENCE1)
				mSDF[Index(x, y, z)] = std::max(-val1, val2);
			else if constexpr (op == CO_DIFFERENCE2)
				mSDF[Index(x, y, z)] = std::max(-val2, val1);
		}
	}

private:
	void CacheData();

private:
	int numCellsPerSide[3];
	int numValsPerSide[3];
	std::vector<float> mSDF;
	std::vector<Math::Vector3> mGrad;
	Geometry::Aabb3 mBox;
	Math::Vector3 mSteps;
};

} // namespace Geometry