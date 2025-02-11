#include "Geometry/Mesh.h"
#include "Geometry/Assets.h"
#include "Geometry/AabbTree.h"
#include "Geometry/ClosestPointOnMesh.h"
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

void main()
{
	Mesh mesh;
	bool ret = LoadMeshFromObj("../Models/bunny.obj", mesh);
	if (!ret)
		return;

	Printf("The mesh has %d vertices, %d normals and %d triangles.\n", mesh.vertices.size(), mesh.normals.size(), mesh.GetNumTriangles());

	mesh.ComputeNormals();
	mesh.ConstructEdges();
	mesh.ConstructVertexOneRings();
	mesh.ConstructTriangleOneRings();
	mesh.ConstructEdgeOneRings();

	AabbTreeObj tree;

	// compute AABB tree
	tree.Construct(mesh);

	// generate some random points inside its AABB
	std::vector<Math::Vector3> randPoints;
	randPoints.clear();
	Aabb3 box = mesh.GetAabb();
	box.Extrude(10.f);
	for (int i = 0; i < 1000; i++)
	{
		Vector3 rnd(GetRandomReal01(), GetRandomReal01(), GetRandomReal01());
		rnd.Scale(box.GetExtent());
		randPoints.push_back(box.min + rnd);
	}

	std::ofstream file("queries.csv");

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

			file << i << ", " << infos.size() << ", ";
			file << p.x << ", " << p.y << ", " << p.z << ", ";
			file << info.closestPtMesh.x << ", " << info.closestPtMesh.y << ", " << info.closestPtMesh.z << ", ";
			file << info.normal.x << ", " << info.normal.y << ", " << info.normal.z << ", ";
			file << info.baryOnMesh.x << ", " << info.baryOnMesh.y << ", " << info.baryOnMesh.z << ", ";
			file << info.distance << ", " << info.tri << ", " << info.region << ", " << info.feature << ", ";
			file << info.side << ", " << sign << std::endl;
		}
	}

}