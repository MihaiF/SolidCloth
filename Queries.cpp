#include "Geometry/Mesh.h"
#include "Geometry/Assets.h"
#include "Geometry/AabbTree.h"
#include "Geometry/ClosestPointOnMesh.h"
#include "Geometry/RaycastMesh.h"
#include "Engine/Utils.h"

#include <memory>
#include <iostream>
#include <fstream>

using namespace Geometry;
using namespace Math;

class AabbTreeObj
{
public:
	void Construct(const Mesh& mesh)
	{
		mTree.reset(ComputeMeshTree(mesh, ATF_TRIANGLES | ATF_EDGES, 10, 0.1f));
	}

	AabbTree* GetTree() { return mTree.get(); }

private:
	std::unique_ptr<AabbTree> mTree;
};

void PointQueries(const Mesh& mesh)
{
	// generate some random points inside its AABB
	std::vector<Vector3> randPoints;
	randPoints.clear();
	Aabb3 box = mesh.GetAabb();
	box.Extrude(10.f);
	for (int i = 0; i < 1000; i++)
	{
		Vector3 rnd(GetRandomReal01(), GetRandomReal01(), GetRandomReal01());
		rnd.Scale(box.GetExtent());
		randPoints.push_back(box.min + rnd);
	}

	std::ofstream file("point_queries.csv");

	// the actual queries
	std::vector<ClosestTriangleToPoint> infos;
	for (int i = 0; i < randPoints.size(); i++)
	{
		const Vector3& p = randPoints[i];

		infos.clear();
		ClosestPointsOnMeshToPoint(p, mesh, infos, 10.f);

		for (int j = 0; j < infos.size(); j++)
		{
			const ClosestTriangleToPoint& info = infos[j];
			float sign = info.normal.Dot(p - info.closestPtMesh);

			file << i << ", " << j << ", ";
			file << p.x << ", " << p.y << ", " << p.z << ", ";
			file << info.closestPtMesh.x << ", " << info.closestPtMesh.y << ", " << info.closestPtMesh.z << ", ";
			file << info.normal.x << ", " << info.normal.y << ", " << info.normal.z << ", ";
			file << info.baryOnMesh.x << ", " << info.baryOnMesh.y << ", " << info.baryOnMesh.z << ", ";
			file << info.distance << ", " << info.tri << ", " << info.region << ", " << info.feature << ", ";
			file << info.side << ", " << sign << std::endl;
		}
	}
}

void EdgeQueries(const Mesh& mesh)
{
	// generate some random points inside its AABB
	std::vector<Vector3> randPoints;
	randPoints.clear();
	Aabb3 box = mesh.GetAabb();
	box.Extrude(10.f);
	for (int i = 0; i < 1000; i++)
	{
		Vector3 rnd(GetRandomReal01(), GetRandomReal01(), GetRandomReal01());
		rnd.Scale(box.GetExtent());
		randPoints.push_back(box.min + rnd);
	}

	std::ofstream file("edge_queries.csv");

	for (size_t i = 0; i < randPoints.size(); i += 2)
	{
		// build a segment
		const Vector3& p = randPoints[i];
		const Vector3& q = randPoints[i + 1];

		file << i << ", ";
		RaycastInfo info;
		if (RaycastMesh(p, q, mesh, info, false))
		{
			// report intersection
			file << "1, ";
			file << info.pt.x << ", " << info.pt.y << ", " << info.pt.z << ", ";
			file << info.pt.x << ", " << info.pt.y << ", " << info.pt.z << ", "; // same point on mesh and on segment
			file << info.normal.x << ", " << info.normal.y << ", " << info.normal.z << ", ";
			file << "0, "; // zero distance (ill defined)
			file << info.tri;
		}
		else
		{
			ClosestEdgeToSegment result;
			float dist = ClosestPointOnMeshToSegment(p, q, mesh, result);
			// report closest point
			file << "0, ";
			file << result.closestPtMesh.x << ", " << result.closestPtMesh.y << ", " << result.closestPtMesh.z << ", ";		
			file << result.closestPtSegment.x << ", " << result.closestPtSegment.y << ", " << result.closestPtSegment.z << ", ";
			file << result.normal.x << ", " << result.normal.y << ", " << result.normal.z << ", ";
			file << result.distance << ", ";
			file << result.edge;
		}
		file << std::endl;
	}
}

void main()
{
	Mesh mesh;
	bool ret = LoadMeshFromObj("../Models/bunny.obj", mesh);
	if (!ret)
		return;

	Printf("The mesh has %d vertices, %d normals and %d triangles.\n", mesh.vertices.size(), mesh.normals.size(), mesh.GetNumTriangles());

	// most are probably not used, besides the normals
	mesh.ComputeNormals();
	mesh.ConstructEdges();
	mesh.ConstructVertexOneRings();
	mesh.ConstructTriangleOneRings();
	mesh.ConstructEdgeOneRings();

	AabbTreeObj tree;

	// compute AABB tree
	tree.Construct(mesh); // not yet used

	PointQueries(mesh);
	EdgeQueries(mesh);
}