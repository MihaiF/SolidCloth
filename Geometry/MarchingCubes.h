#pragma once

#include <Geometry/Mesh.h>
#include <Geometry/Collision3D.h>
#include <Engine/Profiler.h>

#include <array>
#include <functional>

// TODO: namespace Geometry

// TODO: turn into a class

struct Cell
{
	float val[8]; // value at each cube's corner
	Vector3 pos[8]; // cube positions
	int x[8];
	int y[8];
	int z[8];
};

void CreateIsoSurface(Geometry::Mesh& mcMesh, std::function<float(int, int, int)> eval,
	std::function<Vector3(const Vector3&)> grad, const Geometry::Aabb3& box, int numCellsPerSide[], float isoLevel = 0.f, float narrowBand = FLT_MAX);

int polygonise(Cell& grid, float iso_lvl, Vector3* triangles);

struct TriangleBucket
{
	std::array<Vector3, 16> arr;
	std::array<Vector3, 16> normals;
	int n;
	int off;
};

template<typename EVAL, typename GRAD>
inline void CreateIsoSurfaceParallel(Geometry::Mesh& mcMesh, EVAL&& eval,
	GRAD&& grad, const Geometry::Aabb3& box, int numCellsPerSide[], float isoLevel = 0.f, float narrowBand = FLT_MAX)
{
	PROFILE_SCOPE("Marching cubes parallel");

	Vector3 steps = box.GetExtent();
	Vector3 invRes(1.f / numCellsPerSide[0], 1.f / numCellsPerSide[1], 1.f / numCellsPerSide[2]);
	steps.Scale(invRes);

	static std::vector<TriangleBucket> triangles(numCellsPerSide[0] * numCellsPerSide[1] * numCellsPerSide[2]);
	{
		PROFILE_SCOPE("Polygonize");
		#pragma omp parallel for num_threads(3)
		for (int cellNum = 0; cellNum < triangles.size(); cellNum++)
		{
			int x = cellNum % numCellsPerSide[0];
			int w = cellNum / numCellsPerSide[0];
			int y = w % numCellsPerSide[1];
			int z = w / numCellsPerSide[1];
			Vector3 org(x, y, z);
			org.Scale(steps);
			org += box.min;

			Cell cell;
			cell.x[0] = x; cell.y[0] = y; cell.z[0] = z;
			cell.x[1] = x + 1; cell.y[1] = y; cell.z[1] = z;
			cell.x[2] = x + 1; cell.y[2] = y; cell.z[2] = z + 1;
			cell.x[3] = x; cell.y[3] = y; cell.z[3] = z + 1;
			cell.x[4] = x; cell.y[4] = y + 1; cell.z[4] = z;
			cell.x[5] = x + 1; cell.y[5] = y + 1; cell.z[5] = z;
			cell.x[6] = x + 1; cell.y[6] = y + 1; cell.z[6] = z + 1;
			cell.x[7] = x; cell.y[7] = y + 1; cell.z[7] = z + 1;

			float minAbsDist = FLT_MAX;
			for (int i = 0; i < 8; ++i)
			{
				float val = eval(cell.x[i], cell.y[i], cell.z[i]);
				cell.val[i] = val;
				minAbsDist = std::min(minAbsDist, fabs(val));
			}

			if (minAbsDist != FLT_MAX && minAbsDist > narrowBand)
				continue;

			cell.pos[0] = Vector3(org.x, org.y, org.z);
			cell.pos[1] = Vector3(org.x + steps.x, org.y, org.z);
			cell.pos[2] = Vector3(org.x + steps.x, org.y, org.z + steps.z);
			cell.pos[3] = Vector3(org.x, org.y, org.z + steps.z);
			cell.pos[4] = Vector3(org.x, org.y + steps.y, org.z);
			cell.pos[5] = Vector3(org.x + steps.x, org.y + steps.y, org.z);
			cell.pos[6] = Vector3(org.x + steps.x, org.y + steps.y, org.z + steps.z);
			cell.pos[7] = Vector3(org.x, org.y + steps.y, org.z + steps.z);

			int n = polygonise(cell, isoLevel, triangles[cellNum].arr.data());
			triangles[cellNum].n = n;

			// compute normals
			for (int i = 0; i < n; i++)
			{
				Vector3 normal = grad(triangles[cellNum].arr[i]);
				normal.Normalize();
				triangles[cellNum].normals[i] = normal;
			}
		}
	}

	{
		PROFILE_SCOPE("Assemble mesh");
		int count = 0;
		for (int cellNum = 0; cellNum < triangles.size(); cellNum++)
		{
			triangles[cellNum].off = count;
			count += triangles[cellNum].n;
		}
		mcMesh.indices.resize(count);
		mcMesh.vertices.resize(count);
		mcMesh.normals.resize(count);

		#pragma omp parallel for num_threads(3)
		for (int i = 0; i < count; i++)
			mcMesh.indices[i] = i;
		
		#pragma omp parallel for num_threads(3)
		for (int cellNum = 0; cellNum < triangles.size(); cellNum++)
		{
			memcpy(mcMesh.vertices.data() + triangles[cellNum].off, triangles[cellNum].arr.data(), triangles[cellNum].n * sizeof(Vector3));
			memcpy(mcMesh.normals.data() + triangles[cellNum].off, triangles[cellNum].normals.data(), triangles[cellNum].n * sizeof(Vector3));
		}
	}
}

