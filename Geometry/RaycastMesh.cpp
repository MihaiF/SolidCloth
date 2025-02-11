#include "RaycastMesh.h"

using namespace Math;

namespace Geometry
{
	bool RaycastMesh(Vector3 p, Vector3 q, const Mesh& mesh, RaycastInfo& info, bool takeFirst, Matrix4* transform, bool isRay)
	{
		// prepare output
		info.bary.x = -1;
		info.bary.y = -1;
		info.bary.z = -1;
		info.tri = -1;
		float tMin = FLT_MAX;

		// go through all potential triangles
		#pragma omp parallel for
		for (int j = 0; j < mesh.GetNumTriangles(); j++)
		{
			uint32 i0 = mesh.indices[j * 3];
			uint32 i1 = mesh.indices[j * 3 + 1];
			uint32 i2 = mesh.indices[j * 3 + 2];
			Vector3 a = mesh.vertices[i0];
			Vector3 b = mesh.vertices[i1];
			Vector3 c = mesh.vertices[i2];
			if (transform != nullptr)
			{
				a = transform->Transform(a);
				b = transform->Transform(b);
				c = transform->Transform(c);
			}

			Vector3 bary, pt;
			float t;
			if (IntersectSegmentTriangle(p, q, a, b, c, bary, t, pt, isRay))
			{
				if (takeFirst || t < tMin)
				{
					info.bary = bary;
					info.pt = pt;
					info.t = t;
					info.normal = mesh.normals[i0] * bary.x + mesh.normals[i1] * bary.y + mesh.normals[i2] * bary.z;
					info.normal.Normalize();
					info.tri = j;

					if (takeFirst)
						break;

					if (t < tMin)
						tMin = t;
				}
			}
		}

		return info.tri >= 0;
	}
}
