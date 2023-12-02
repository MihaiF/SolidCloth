#include "ClosestPointsBetweenMeshes.h"
#include "AabbTree.h"
#include "Engine/Profiler.h"

namespace Geometry
{
	void MeshClosestPoints::TestTrees(const Mesh* mesh1, const Mesh* mesh2, AabbTree* node1, AabbTree* node2, float radTol)
	{
		if (node1 == NULL || node2 == NULL)
			return;

		Aabb3 box1 = node1->box;
		Aabb3 box2 = node2->box;
		if (!AabbOverlap3D(box1, box2))
			return;

		if (node1->triangles.empty() && node1->vertices.empty())
		{
			if (node1->left)
				TestTrees(mesh1, mesh2, node1->left, node2, radTol);
			if (node1->right)
				TestTrees(mesh1, mesh2, node1->right, node2, radTol);
		}
		else if (node2->triangles.empty() && node2->vertices.empty())
		{
			if (node2->left)
				TestTrees(mesh1, mesh2, node1, node2->left, radTol);
			if (node2->right)
				TestTrees(mesh1, mesh2, node1, node2->right, radTol);
		}
		else
		{
			const Vector3 extrude(radTol);

			if ((mCollFlags & CF_VERTICES) && !node1->vertices.empty() && !node2->triangles.empty())
			{
				Vector3 bndMin = node2->box.min - extrude;
				Vector3 bndMax = node2->box.max + extrude;
				for (size_t i = 0; i < node1->vertices.size(); i++)
				{
					int idx = node1->vertices[i].idx;
					const Vector3& v = mesh1->vertices[idx];
					if (!PointInAabb3D(bndMin, bndMax, v)) // isn't this redundant?
						continue;
					for (size_t j = 0; j < node2->triangles.size(); j++)
					{
						int k = node2->triangles[j].idx * 3; // TODO: remove * 3
						PrimitivePair pair;
						pair.idx1 = idx;
						pair.idx2 = k;
						mPotentialContacts.push_back(pair);
					}
				}
			}

			if (mCollFlags & CF_TRIANGLES && !node2->vertices.empty() && !node1->triangles.empty())
			{
				for (size_t j = 0; j < node1->triangles.size(); j++)
				{
					int tri = node1->triangles[j].idx;
					const uint16 i1 = mesh1->indices[tri * 3 + 0];
					const uint16 i2 = mesh1->indices[tri * 3 + 1];
					const uint16 i3 = mesh1->indices[tri * 3 + 2];
					Vector3 v1 = mesh1->vertices[i1];
					Vector3 v2 = mesh1->vertices[i2];
					Vector3 v3 = mesh1->vertices[i3];

					// build and test AABB
					Vector3 minV = vmin(vmin(v1, v2), v3) - extrude;
					Vector3 maxV = vmax(vmax(v1, v2), v3) + extrude;
					if (!AabbOverlap3D(node2->box, Aabb3(minV, maxV)))
						continue;

					for (size_t i = 0; i < node2->vertices.size(); i++)
					{
						int idx = node2->vertices[i].idx;
						Vector3 v = mesh2->vertices[idx];

						if (!PointInAabb3D(minV, maxV, v))
							continue;
						
						PrimitivePair pair;
						pair.idx1 = tri;
						pair.idx2 = idx;
						mPotentialTriContacts.push_back(pair);
					}
				}
			}
		}
	}

	void MeshClosestPoints::VertexVsTriangle(const Mesh* mesh1, const Mesh* mesh2, int vertexIndex, int triangleIndex)
	{
		const Vector3& v = mesh1->vertices[vertexIndex];

		const uint16 i1 = mesh2->indices[triangleIndex];
		const uint16 i2 = mesh2->indices[triangleIndex + 1];
		const uint16 i3 = mesh2->indices[triangleIndex + 2];
		const Vector3& v1 = mesh2->vertices[i1];
		const Vector3& v2 = mesh2->vertices[i2];
		const Vector3& v3 = mesh2->vertices[i3];

		Vector3 coords;
		Vector3 p;
		int region = ClosestPtPointTriangle(v, v1, v2, v3, p, coords);
		Vector3 delta = p - v;
		float dist = delta.Length(); // unsigned distance

		// use the pseudo-normal
		Vector3 n = ComputeNormal(*mesh2, triangleIndex / 3, coords);
		// TODO: sign the distance

		if (dist < mVertexInfos[vertexIndex].distance)
		#pragma omp critical
		{
			mVertexInfos[vertexIndex].distance = dist;
			mVertexInfos[vertexIndex].tri = triangleIndex / 3;
			mVertexInfos[vertexIndex].closestPtMesh = p;
			mVertexInfos[vertexIndex].baryOnMesh = coords;
			mVertexInfos[vertexIndex].normal = n;
			mVertexInfos[vertexIndex].region = region;
		}
	}

	void MeshClosestPoints::TriangleVsVertex(const Mesh* mesh1, const Mesh* mesh2, int idx, int tri)
	{
		const uint16 i1 = mesh1->indices[tri * 3 + 0];
		const uint16 i2 = mesh1->indices[tri * 3 + 1];
		const uint16 i3 = mesh1->indices[tri * 3 + 2];
		const Vector3& v1 = mesh1->vertices[i1];
		const Vector3& v2 = mesh1->vertices[i2];
		const Vector3& v3 = mesh1->vertices[i3];

		Vector3 v = mesh2->vertices[idx];

		// get the closest point and distance from vertex to triangle
		Vector3 coords, cp;
		int region = ClosestPtPointTriangle(v, v1, v2, v3, cp, coords); // 'cp' is the point on the triangle
		Vector3 delta = cp - v;
		float dist = delta.Length();

		// use the vertex normal
		Vector3 normal = mesh2->normals[idx];
		// TODO: sign the distance

		if (dist < mTriangleInfos[tri].distance)
		#pragma omp critical
		{
			mTriangleInfos[tri].distance = dist;
			mTriangleInfos[tri].vertex = idx;
			mTriangleInfos[tri].closestPtMesh = v;
			mTriangleInfos[tri].closestPtTri = cp;
			mTriangleInfos[tri].region = region;
			mTriangleInfos[tri].baryCoords = coords;
			mTriangleInfos[tri].normal = normal;
		}
	}

	void QueryPoint(Vector3 point, const Mesh& mesh, const std::vector<int>& triangles, ClosestTriangleToPoint& info)
	{
		if (triangles.size() == 0)
		{
			info.distance = FLT_MAX;
			return;
		}

		info = ClosestPointOnMeshToPoint(point, mesh, triangles);
		Vector3 delta = point - info.closestPtMesh;
		if (delta.Dot(info.normal) < 0)
			info.distance *= -1;
	}

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, const Mesh& mesh,
		const std::vector<int>& vertexSet, Geometry::ClosestVertexToTriangle& info)
	{
		float dist = ClosestPointOnMeshToTriangle(a, b, c, mesh, vertexSet, info);
		Vector3 delta = info.closestPtTri - info.closestPtMesh;
		if (delta.Dot(info.normal) < 0)
			info.distance *= -1;
		
		return dist;
	}


	void MeshClosestPoints::ClosestPoints(const Mesh& mesh1, const Mesh& mesh2, AabbTree* root1, AabbTree* root2, float radTol, bool useSDF)
	{
		PROFILE_SCOPE("Closest points");

		// TODO: edge-edge case

		mPotentialContacts.clear();
		mPotentialTriContacts.clear();
		{
			PROFILE_SCOPE("Test trees");
			TestTrees(&mesh1, &mesh2, root1, root2, radTol);
		}

		mVertexInfos.resize(mesh1.vertices.size());
		mTriangleInfos.resize(mesh1.indices.size() / 3);

		if (!useSDF)
		{
			for (int i = 0; i < mVertexInfos.size(); i++)
				mVertexInfos[i].distance = FLT_MAX;

			#pragma omp parallel for
			for (int i = 0; i < (int)mPotentialContacts.size(); i++)
			{
				int k = mPotentialContacts[i].idx2;
				int idx = mPotentialContacts[i].idx1;
				VertexVsTriangle(&mesh1, &mesh2, idx, k);
			}

			for (int i = 0; i < mTriangleInfos.size(); i++)
				mTriangleInfos[i].distance = FLT_MAX;

			#pragma omp parallel for
			for (int i = 0; i < (int)mPotentialTriContacts.size(); i++)
			{
				int tri = mPotentialTriContacts[i].idx1;
				int idx = mPotentialTriContacts[i].idx2;
				TriangleVsVertex(&mesh1, &mesh2, idx, tri);
			}
		}
		else
		{
			if (mSetVT.size() != mesh1.vertices.size())
				mSetVT.resize(mesh1.vertices.size());

			for (size_t i = 0; i < mesh1.vertices.size(); i++)
			{
				mSetVT[i].clear();
			}

			for (int i = 0; i < (int)mPotentialContacts.size(); i++)
			{
				int k = mPotentialContacts[i].idx2;
				int idx = mPotentialContacts[i].idx1;
				mSetVT[idx].push_back(k / 3);
			}

			#pragma omp parallel for
			for (int i = 0; i < mesh1.vertices.size(); i++)
			{
				Vector3 p, n;
				QueryPoint(mesh1.vertices[i], mesh2, mSetVT[i], mVertexInfos[i]);
			}

			if (mSetTV.size() != mesh1.triangles.size())
				mSetTV.resize(mesh1.triangles.size());

			for (int i = 0; i < mesh1.triangles.size(); i++)
			{
				mSetTV[i].clear();
			}

			for (int i = 0; i < (int)mPotentialTriContacts.size(); i++)
			{
				int tri = mPotentialTriContacts[i].idx1;
				int idx = mPotentialTriContacts[i].idx2;
				mSetTV[tri].push_back(idx);
			}

			#pragma omp parallel for
			for (int i = 0; i < mesh1.triangles.size(); i++)
			{
				int i1 = mesh1.indices[i * 3];
				int i2 = mesh1.indices[i * 3 + 1];
				int i3 = mesh1.indices[i * 3 + 2];

				const Vector3& v1 = mesh1.vertices[i1];
				const Vector3& v2 = mesh1.vertices[i2];
				const Vector3& v3 = mesh1.vertices[i3];

				QueryTriangle(v1, v2, v3, mesh2, mSetTV[i], mTriangleInfos[i]);
			}
		}
	}

	void MeshClosestPoints::ClosestPoints(const Mesh& mesh1, const Mesh& mesh2, AabbTree* root2, bool sign)
	{
		PROFILE_SCOPE("Closest points");

		// TODO: use caches
		// TODO: per-vertex tolerance

		if (mCollFlags & CF_VERTICES)
		{
			if (mVertexInfos.size() != mesh1.vertices.size())
				mVertexInfos.resize(mesh1.vertices.size());

			#pragma omp parallel for
			for (int i = 0; i < (int)mesh1.vertices.size(); i++)
			{
				mVertexInfos[i] = ClosestPointOnMeshToPointAcc(mesh1.vertices[i], mesh2, root2);
				if (sign)
				{
					Vector3 delta = mesh1.vertices[i] - mVertexInfos[i].closestPtMesh;
					if (delta.Dot(mVertexInfos[i].normal) < 0)
						mVertexInfos[i].distance *= -1;
				}
			}
		}

		if (mCollFlags & CF_TRIANGLES)
		{
			size_t nt = mesh1.indices.size() / 3;
			if (mTriangleInfos.size() != nt)
				mTriangleInfos.resize(nt);

			#pragma omp parallel for
			for (int i = 0; i < mTriangleInfos.size(); i++)
			{
				int i0 = mesh1.indices[i * 3];
				int i1 = mesh1.indices[i * 3 + 1];
				int i2 = mesh1.indices[i * 3 + 2];
				const Vector3& a = mesh1.vertices[i0];
				const Vector3& b = mesh1.vertices[i1];
				const Vector3& c = mesh1.vertices[i2];
				mTriangleInfos[i] = Geometry::ClosestPointOnMeshToTriangleAcc(a, b, c, mesh2, root2, mTriangleInfos[i].vertex);
				if (sign)
				{
					Vector3 delta = mTriangleInfos[i].closestPtTri - mTriangleInfos[i].closestPtMesh;
					if (delta.Dot(mTriangleInfos[i].normal) < 0)
						mTriangleInfos[i].distance *= -1;
				}
			}
		}

		if (mCollFlags & CF_EDGES)
		{
			if (mEdgeInfos.size() != mesh1.edges.size())
				mEdgeInfos.resize(mesh1.edges.size());

			#pragma omp parallel for
			for (int i = 0; i < mesh1.edges.size(); i++)
			{
				int i1 = mesh1.edges[i].i1;
				int i2 = mesh1.edges[i].i2;
				const Vector3& p = mesh1.vertices[i1];
				const Vector3& q = mesh1.vertices[i2];
				// TODO: return info?
				Geometry::ClosestPointOnMeshToSegmentAcc(p, q, mesh2, root2, -1, mEdgeInfos[i]);
			}
		}
	}

} // namespace Geometry