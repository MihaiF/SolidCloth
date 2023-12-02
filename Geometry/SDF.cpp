#include "SDF.h"
#include "ClosestPointOnMesh.h"

#include <Engine/Utils.h>
#include <Geometry/AabbTree.h>

using namespace Geometry;

void SDF::Create(int nx, int ny, int nz, const Aabb3& box)
{
	mBox = box;
	numCellsPerSide[0] = nx;
	numCellsPerSide[1] = ny;
	numCellsPerSide[2] = nz;

	CacheData();
}

void SDF::Create(int res[], const Aabb3& box)
{
	mBox = box;
	numCellsPerSide[0] = res[0];
	numCellsPerSide[1] = res[1];
	numCellsPerSide[2] = res[2];

	CacheData();
}

void SDF::CacheData()
{
	numValsPerSide[0] = numCellsPerSide[0] + 1;
	numValsPerSide[1] = numCellsPerSide[1] + 1;
	numValsPerSide[2] = numCellsPerSide[2] + 1;
	mSDF.resize(numValsPerSide[0] * numValsPerSide[1] * numValsPerSide[2]);

	mSteps = mBox.GetExtent();
	Vector3 invRes(1.f / numCellsPerSide[0], 1.f / numCellsPerSide[1], 1.f / numCellsPerSide[2]);
	mSteps.Scale(invRes);
}

void SDF::CreateFromImplicit(std::function<float(const Vector3& pos)> eval)
{
	for (int x = 0; x <= numCellsPerSide[0]; x++)
	{
		for (int y = 0; y <= numCellsPerSide[1]; y++)
		{
			for (int z = 0; z <= numCellsPerSide[2]; z++)
			{
				Vector3 org = GetPosAt(x, y, z);
				float val = eval(org);
				mSDF[Index(x, y, z)] = val;
			}
		}
	}
}

void SDF::CreateFromMesh(const Mesh& mesh, const AabbTree* tree)
{
	if (tree == nullptr)
		tree = ComputeMeshTree(mesh, ATF_TRIANGLES, 10, 0.1f);

	#pragma omp parallel for num_threads(3)
	for (int idx = 0; idx < GetCount(); idx++)
	{
		int x, y, z;
		GetCoords(idx, x, y, z);
		Vector3 org = GetPosAt(x, y, z);
		ClosestTriangleToPoint info = ClosestPointOnMeshToPointAcc(org, mesh, tree);
		float val = info.distance;
		Vector3 delta = org - info.closestPtMesh;
		if (delta.Dot(info.normal) < 0)
			val = -val;
		mSDF[idx] = val;
	}
}

void SDF::WriteToFile(const char* path)
{
	FILE* file;
	if (fopen_s(&file, path, "wb") == 0)
	{
		fwrite(numCellsPerSide, sizeof(int), 3, file);

		double val;
		val = mBox.min.x;
		fwrite(&val, sizeof(double), 1, file);
		val = mBox.max.x;
		fwrite(&val, sizeof(double), 1, file);

		val = mBox.min.y;
		fwrite(&val, sizeof(double), 1, file);
		val = mBox.max.y;
		fwrite(&val, sizeof(double), 1, file);

		val = mBox.min.z;
		fwrite(&val, sizeof(double), 1, file);
		val = mBox.max.z;
		fwrite(&val, sizeof(double), 1, file);

		// we need to make sure we write first all the x values, hence the inner loop
		for (int z = 0; z <= numCellsPerSide[2]; z++)
		{
			for (int y = 0; y <= numCellsPerSide[1]; y++)
			{
				for (int x = 0; x <= numCellsPerSide[0]; x++)
				{
					val = GetValue(x, y, z);
					fwrite(&val, sizeof(double), 1, file);
				}
			}
		}

		fclose(file);
	}
}

void SDF::LoadFromFile(const char* path)
{
	FILE* file;
	if (fopen_s(&file, path, "rb") == 0)
	{
		fread(numCellsPerSide, sizeof(int), 3, file);

		double val;
		fread(&val, sizeof(double), 1, file);
		mBox.min.x = val;
		fread(&val, sizeof(double), 1, file);
		mBox.max.x = val;

		fread(&val, sizeof(double), 1, file);
		mBox.min.y = val;
		fread(&val, sizeof(double), 1, file);
		mBox.max.y = val;

		fread(&val, sizeof(double), 1, file);
		mBox.min.z = val;
		fread(&val, sizeof(double), 1, file);
		mBox.max.z = val;

		CacheData();
		
		for (int i = 0; i < mSDF.size(); i++)
		{
			fread(&val, sizeof(double), 1, file);
			mSDF[i] = val;
		}

		fclose(file);
	}
}

void SDF::Blur()
{
	// first blur along z direction
	for (int x = 0; x <= numCellsPerSide[0]; x++)
	{
		for (int y = 0; y <= numCellsPerSide[1]; y++)
		{
			for (int z = 0; z < numCellsPerSide[2]; z++)
			{
				float val1 = GetValue(x, y, z);
				float val2 = GetValue(x, y, z + 1);
				SetValue(x, y, z, 0.5f * (val1 + val2));
			}
			int z = numCellsPerSide[2];
			float val1 = GetValue(x, y, z);
			float val2 = GetValue(x, y, z - 1);
			SetValue(x, y, z, 0.5f * (val1 + val2));
		}
	}

	// then along y direction
	for (int x = 0; x <= numCellsPerSide[0]; x++)
	{
		for (int z = 0; z <= numCellsPerSide[2]; z++)
		{
			for (int y = 0; y < numCellsPerSide[1]; y++)
			{
				float val1 = mSDF[x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]];
				float val2 = mSDF[x + (y + 1) * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]];
				mSDF[x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]] = 0.5f * (val1 + val2);
			}
			int y = numCellsPerSide[1];
			float val1 = mSDF[x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]];
			float val2 = mSDF[x + (y - 1) * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]];
			mSDF[x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]] = 0.5f * (val1 + val2);
		}
	}

	// finally, along x direction
	for (int z = 0; z <= numCellsPerSide[2]; z++)
	{
		for (int y = 0; y <= numCellsPerSide[1]; y++)
		{
			for (int x = 0; x < numCellsPerSide[0]; x++)
			{
				float val1 = mSDF[x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]];
				float val2 = mSDF[x + 1 + (y + 1) * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]];
				mSDF[x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]] = 0.5f * (val1 + val2);
			}
			int x = numCellsPerSide[0];
			float val1 = mSDF[x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]];
			float val2 = mSDF[x - 1 + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]];
			mSDF[x + y * numValsPerSide[0] + z * numValsPerSide[0] * numValsPerSide[1]] = 0.5f * (val1 + val2);
		}
	}
}

void SDF::ComputeGradient()
{
	PROFILE_SCOPE("Compute SDF gradients");

	mGrad.resize(mSDF.size());

	// at each grid point compute the 3 gradient components
	for (int x = 0; x < numCellsPerSide[0]; x++)
	{
		for (int y = 0; y < numCellsPerSide[1]; y++)
		{
			for (int z = 0; z < numCellsPerSide[2]; z++)
			{
				float val = GetValue(x, y, z);
				// df/dx
				float valX = GetValue(x + 1, y, z);
				float dfdx = (valX - val) / mSteps.x;
				// df/dy
				float valY = GetValue(x, y + 1, z);
				float dfdy = (valY - val) / mSteps.y;
				// df/dz
				float valZ = GetValue(x, y, z + 1);
				float dfdz = (valZ - val) / mSteps.z;
				
				Vector3 grad(dfdx, dfdy, dfdz);
				SetGrad(x, y, z, grad);
			}
			int z = numCellsPerSide[2];
			SetGrad(x, y, z, GetGrad(x, y, z - 1));
		}
		
		int y = numCellsPerSide[1];
		for (int z = 0; z <= numCellsPerSide[2]; z++)
		{
			SetGrad(x, y, z, GetGrad(x, y - 1, z));
		}
	}

	int x = numCellsPerSide[0];
	for (int y = 0; y <= numCellsPerSide[1]; y++)
	{
		for (int z = 0; z <= numCellsPerSide[2]; z++)
		{
			SetGrad(x, y, z, GetGrad(x - 1, y, z));
		}
	}
}

void SDF::ComputeGradientParallel(float narrowBand)
{
	PROFILE_SCOPE("SDF gradients parallel");

	mGrad.resize(mSDF.size());
	int count = numCellsPerSide[0] * numCellsPerSide[1] * numCellsPerSide[2];

	// at each grid point compute the 3 gradient components
	#pragma omp parallel for num_threads(3)
	for (int idx = 0; idx < count; idx++)
	{
		int x = idx % numCellsPerSide[0];
		int w = idx / numCellsPerSide[0];
		int y = w % numCellsPerSide[1];
		int z = w / numCellsPerSide[1];

		float val = GetValue(x, y, z);
		if (fabs(val) > narrowBand)
			continue;

		// df/dx
		float valX = GetValue(x + 1, y, z);
		float dfdx = (valX - val) / mSteps.x;
		// df/dy
		float valY = GetValue(x, y + 1, z);
		float dfdy = (valY - val) / mSteps.y;
		// df/dz
		float valZ = GetValue(x, y, z + 1);
		float dfdz = (valZ - val) / mSteps.z;
		
		Vector3 grad(dfdx, dfdy, dfdz);
		SetGrad(x, y, z, grad);
	}

	// TODO: don't copy the equal gradients, just incorporate it directly into GetGrad()
}

void SDF::Combine(const SDF& other, CombineOp op)
{
	int precision[3];
	other.GetNumCellsPerSide(precision);
	if (numCellsPerSide[0] != precision[0] ||
		numCellsPerSide[1] != precision[1] ||
		numCellsPerSide[2] != precision[2])
		return;

	for (int x = 0; x <= numCellsPerSide[0]; x++)
	{
		for (int y = 0; y <= numCellsPerSide[1]; y++)
		{
			for (int z = 0; z <= numCellsPerSide[2]; z++)
			{
				float val1 = GetValue(x, y, z);
				float val2 = other.GetValue(x, y, z);
				mSDF[Index(x, y, z)] = std::min(val1, val2);
			}
		}
	}
}

void SDF::Combine(std::function<float(const Vector3& pos)> eval, CombineOp op)
{
	PROFILE_SCOPE("Combine SDF");

	for (int x = 0; x <= numCellsPerSide[0]; x++)
	{
		for (int y = 0; y <= numCellsPerSide[1]; y++)
		{
			for (int z = 0; z <= numCellsPerSide[2]; z++)
			{
				Vector3 org = GetPosAt(x, y, z);
				float val1 = GetValue(x, y, z);
				float val2 = eval(org);
				if (op == CO_UNION)
					mSDF[Index(x, y, z)] = std::min(val1, val2);
				else if (op == CO_UNION_SMOOTHED)
				{
					const float k = 4.25f;
					float h = std::max(k - fabs(val1 - val2), 0.0f);
					mSDF[Index(x, y, z)] = std::min(val1, val2) - h * h * 0.25f / k;
				}
				else if (op == CO_UNION_ROUNDED)
				{
					const float r = 4.25f;
					float ux = std::max(r - val1, 0.f);
					float uy = std::max(r - val2, 0.f);
					Vector2 u(ux, uy);
					mSDF[Index(x, y, z)] = std::max(r, std::min(val1, val2)) - u.Length();
				}
				else if (op == CO_DIFFERENCE1)
					mSDF[Index(x, y, z)] = std::max(-val1, val2);
				else if (op == CO_DIFFERENCE2)
					mSDF[Index(x, y, z)] = std::max(-val2, val1);
			}
		}
	}
}

