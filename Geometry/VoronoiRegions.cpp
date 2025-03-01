#include "VoronoiRegions.h"
#include "Mesh.h"
#include "Engine/Utils.h"

#include <cmath>

using namespace Math;

namespace Geometry
{
	bool TestPointInTriangleVoronoiRegion(Vector3 p, const Mesh& mesh, int tri)
	{
		int i1 = mesh.indices[tri * 3];
		int i2 = mesh.indices[tri * 3 + 1];
		int i3 = mesh.indices[tri * 3 + 2];

		Vector3 v1 = mesh.vertices[i1];
		Vector3 v2 = mesh.vertices[i2];
		Vector3 v3 = mesh.vertices[i3];

		Vector3 e1 = v2 - v1;
		Vector3 e2 = v3 - v2;
		Vector3 e3 = v1 - v3;

		Vector3 n = mesh.triangles[tri].n;
		Vector3 h1 = e1.Cross(n);
		Vector3 h2 = e2.Cross(n);
		Vector3 h3 = e3.Cross(n);

		float dot1 = h1.Dot(p - v1);
		float dot2 = h2.Dot(p - v2);
		float dot3 = h3.Dot(p - v3);

		return std::signbit(dot1) == std::signbit(dot2) == std::signbit(dot3);
	}

	bool TestPointInEdgeVoronoiRegion(Vector3 p, const Mesh& mesh, int eID, float eps)
	{
		int i1 = mesh.edges[eID].i1;
		int i2 = mesh.edges[eID].i2;
		Vector3 a = mesh.vertices[i1];
		Vector3 b = mesh.vertices[i2];

		// compute the edge direction (aligned with the first triangle)
		Vector3 e = b - a;
		e.Normalize();

		float dot3 = e.Dot(p - a);
		float dot4 = e.Dot(p - b);

		if (dot3 < 0 || dot4 > 0)
			return false;

		if (mesh.edges[eID].swapped)
			e.Flip();

		// compute the 2 plane normals
		if (mesh.edges[eID].t2 >= 0)
		{
			Vector3 h1 = e.Cross(mesh.triangles[mesh.edges[eID].t1].n);
			Vector3 h2 = -e.Cross(mesh.triangles[mesh.edges[eID].t2].n);
			h1.Normalize();
			h2.Normalize();

			float dot1 = h1.Dot(p - a);
			float dot2 = h2.Dot(p - a);

			if (dot1 < -eps || dot2 < -eps)
				return false;
		}
		else
		{
			Vector3 h1 = e.Cross(mesh.triangles[mesh.edges[eID].t1].n);
			h1.Normalize();

			float dot1 = h1.Dot(p - a);

			if (dot1 < -eps)
				return false;
		}
		return true;
	}

	bool TestPointInVertexVoronoiRegion(Vector3 p, const Mesh& mesh, int vID)
	{
		ASSERT(mesh.edgeOneRings.size() > 0);
		auto edgeOneRing = mesh.edgeOneRings[vID];
		Vector3 v = mesh.vertices[vID];
		
		float side = 1;
		Vector3 n = mesh.normals[vID];
		if (n.Dot(p - v) < 0)
			side = -1;
		const float eps = 0.1f;

		for (int i = 0; i < edgeOneRing.size(); i++)
		{
			const Mesh::Edge& edge = mesh.edges[edgeOneRing[i]];
			Vector3 v1 = mesh.vertices[edge.i1];
			Vector3 v2 = mesh.vertices[edge.i2];
			Vector3 e = v1 - v2;
			e.Normalize();
			if (edge.i2 != vID)
				e.Flip();
			float d = e.Dot(p - v);
			if (side > 0 && d < -eps)
				return false;
			if (side < 0 && d > eps)
				return false;
		}
		
		return true;
	}

}