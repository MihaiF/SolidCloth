#include "ClosestPointsBetweenMeshes.h"
#include "AabbTree.h"
#include "Engine/Profiler.h"

using namespace Math;

//#define CONCAVE_CLOSEST_PTS

namespace Geometry
{
	void MeshClosestPoints::TestTrees(const Mesh& mesh1, const Mesh& mesh2, AabbTree* node1, AabbTree* node2, float radTol)
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
#if !defined(LEGACY_TREE) && defined(REP_TRIS)
			for (int i = 0; i < node1->triangles.size(); i++)
			{
				for (int j = 0; j < node2->triangles.size(); j++)
				{
					PrimitivePair pair;
					pair.idx1 = node1->triangles[i].idx;
					pair.idx2 = node2->triangles[j].idx;

					mTriTriCandidates.push_back(pair);
				}
			}
#else
			if ((mCollFlags & CF_VERTICES) && !node1->vertices.empty() && !node2->triangles.empty())
			{
				Vector3 bndMin = node2->box.min - extrude;
				Vector3 bndMax = node2->box.max + extrude;
				for (size_t i = 0; i < node1->vertices.size(); i++)
				{
					int idx = node1->vertices[i].idx;
					const Vector3& v = mesh1.vertices[idx];
					if (!PointInAabb3D(bndMin, bndMax, v)) // TODO: is this redundant?
						continue;
					for (size_t j = 0; j < node2->triangles.size(); j++)
					{
						int k = node2->triangles[j].idx;
						PrimitivePair pair;
						pair.idx1 = idx;
						pair.idx2 = k;
						mPotentialContacts.push_back(pair);
					}
				}
			}

			// TODO: isn't this the same code as above, but with the roles flipped?
			if (mCollFlags & CF_TRIANGLES && !node2->vertices.empty() && !node1->triangles.empty())
			{
				for (size_t j = 0; j < node1->triangles.size(); j++)
				{
					int tri = node1->triangles[j].idx;
					const uint16 i1 = mesh1.indices[tri * 3 + 0];
					const uint16 i2 = mesh1.indices[tri * 3 + 1];
					const uint16 i3 = mesh1.indices[tri * 3 + 2];
					Vector3 v1 = mesh1.vertices[i1];
					Vector3 v2 = mesh1.vertices[i2];
					Vector3 v3 = mesh1.vertices[i3];

					// build and test AABB
					Vector3 minV = vmin(vmin(v1, v2), v3) - extrude;
					Vector3 maxV = vmax(vmax(v1, v2), v3) + extrude;
					if (!AabbOverlap3D(node2->box, Aabb3(minV, maxV)))
						continue;

					for (size_t i = 0; i < node2->vertices.size(); i++)
					{
						int idx = node2->vertices[i].idx;
						Vector3 v = mesh2.vertices[idx];

						if (!PointInAabb3D(minV, maxV, v))
							continue;
						
						PrimitivePair pair;
						pair.idx1 = tri;
						pair.idx2 = idx;
						mPotentialTriContacts.push_back(pair);
					}
				}
			}

			if (mCollFlags & CF_EDGES && node1->edges.size() != 0 && node2->edges.size() != 0)
			{
				for (size_t j = 0; j < node1->edges.size(); j++)
				{
					int idx1 = node1->edges[j].idx;
					const auto& edge1 = mesh1.edges[idx1];
					uint32 i1 = edge1.i1;
					uint32 i2 = edge1.i2;
					Vector3 v1 = mesh1.vertices[i1];
					Vector3 v2 = mesh1.vertices[i2];

					// build and test AABB
					Vector3 minV = vmin(v1, v2) - extrude;
					Vector3 maxV = vmax(v1, v2) + extrude;
					Aabb3 box1(minV, maxV);
					if (!AabbOverlap3D(node2->box, box1))
						continue;

					for (size_t i = 0; i < node2->edges.size(); i++)
					{
						int idx2 = node2->edges[i].idx;

						// this is actually slower
						//const auto& edge2 = mesh2.edges[idx2];
						//Vector3 v3 = mesh2.vertices[edge2.i1];
						//Vector3 v4 = mesh2.vertices[edge2.i2];

						//minV = vmin(v3, v4);
						//maxV = vmax(v3, v4);
						//Aabb3 box2(minV, maxV);
						//if (!AabbOverlap3D(box2, box1))
						//	continue;

						PrimitivePair pair;
						pair.idx1 = idx1;
						pair.idx2 = idx2;
						mPotentialEdgeContacts.push_back(pair);
					}
				}
			}
#endif
		}
	}

	void MeshClosestPoints::VertexVsTriangle(const Mesh* mesh1, const Mesh* mesh2, int vertexIndex, int triangleIndex)
	{
		const Vector3& v = mesh1->vertices[vertexIndex];

		const uint16 i1 = mesh2->indices[triangleIndex * 3];
		const uint16 i2 = mesh2->indices[triangleIndex * 3 + 1];
		const uint16 i3 = mesh2->indices[triangleIndex * 3 + 2];
		const Vector3& v1 = mesh2->vertices[i1];
		const Vector3& v2 = mesh2->vertices[i2];
		const Vector3& v3 = mesh2->vertices[i3];

		Vector3 coords;
		Vector3 p;
		int region = ClosestPtPointTriangle(v, v1, v2, v3, p, coords);
		Vector3 delta = p - v;
		float dist = delta.Length(); // unsigned distance

		// use the pseudo-normal
		Vector3 n = ComputeTriangleNormal(*mesh2, triangleIndex, coords);
		// TODO: sign the distance

		if (dist < mVertexInfos[vertexIndex].distance)
		#pragma omp critical
		{
			mVertexInfos[vertexIndex].distance = dist;
			mVertexInfos[vertexIndex].tri = triangleIndex;
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

	void MeshClosestPoints::EdgeVsEdge(const Mesh* mesh1, const Mesh* mesh2, int e1, int e2)
	{
		const auto& edge1 = mesh1->edges[e1];
		const Vector3& p1 = mesh1->vertices[edge1.i1];
		const Vector3& q1 = mesh1->vertices[edge1.i2];

		const auto& edge2 = mesh2->edges[e2];
		const Vector3& p2 = mesh2->vertices[edge2.i1];
		const Vector3& q2 = mesh2->vertices[edge2.i2];

		float paramSegm, paramMesh;
		Vector3 cpSegm, cpMesh;
		int region = ClosestPtSegmSegm(p1, q1, p2, q2, paramSegm, paramMesh, cpSegm, cpMesh);
		Vector3 delta = cpSegm - cpMesh;
		float dist = delta.Length();

		auto& result = mEdgeInfos[e1];

		if (dist < result.distance)
		{
			result.edge = e1;
			result.distance = dist;
			result.closestPtMesh = cpMesh;
			result.closestPtSegment = cpSegm;
			result.coordsSegm.x = 1.0f - paramSegm;
			result.coordsSegm.y = paramSegm;
			result.coordsMesh.x = 1.0f - paramMesh;
			result.coordsMesh.y = paramMesh;
			result.normal = edge2.n; // use the mesh 2 edge pseudo-normal

			// this is a special case when the closest point is a vertex => use vertex normal
			// for now, we use 0 and 1 as results of clamping from ClosestPtSegmSegm; anything else is considered on the edge
			if (paramMesh == 0.f)
				result.normal = mesh2->normals[edge2.i1];
			if (paramMesh == 1.f)
				result.normal = mesh2->normals[edge2.i2];
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

	float QuerySegment(Vector3 p, Vector3 q, const Mesh& mesh, const std::vector<int>& edgeSet, Geometry::ClosestEdgeToSegment& info)
	{
		float dist = ClosestPointOnMeshToSegment(p, q, mesh, edgeSet, info);
		Vector3 closestPt = info.closestPtMesh;
		Vector3 normal = info.normal;
		Vector2 coords = info.coordsSegm;
		Vector3 pt = coords.x * p + coords.y * q;
		Vector3 delta = pt - closestPt;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}

	// TODO: use collision flags

	void MeshClosestPoints::HandleCandidatePairs(const Mesh& mesh1, const Mesh& mesh2)
	{
		for (int i = 0; i < mVertexInfos.size(); i++)
			mVertexInfos[i].distance = FLT_MAX;

		#pragma omp parallel for
		for (int i = 0; i < (int)mPotentialContacts.size(); i++)
		{
			int vtx = mPotentialContacts[i].idx1;
			int tri = mPotentialContacts[i].idx2;
			VertexVsTriangle(&mesh1, &mesh2, vtx, tri);
		}

		for (int i = 0; i < mTriangleInfos.size(); i++)
			mTriangleInfos[i].distance = FLT_MAX;

		#pragma omp parallel for
		for (int i = 0; i < (int)mPotentialTriContacts.size(); i++)
		{
			int tri = mPotentialTriContacts[i].idx1;
			int vtx = mPotentialTriContacts[i].idx2;
			TriangleVsVertex(&mesh1, &mesh2, vtx, tri);
		}

		for (int i = 0; i < mEdgeInfos.size(); i++)
			mEdgeInfos[i].distance = FLT_MAX;

		#pragma omp parallel for
		for (int i = 0; i < mPotentialEdgeContacts.size(); i++)
		{
			int e1 = mPotentialEdgeContacts[i].idx1;
			int e2 = mPotentialEdgeContacts[i].idx2;
			EdgeVsEdge(&mesh1, &mesh2, e1, e2);
		}
	}

	void MeshClosestPoints::HandleCandidatePairsSDF(const Mesh& mesh1, const Mesh& mesh2)
	{
		// vertices
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

		// triangles
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

		// edges
		if (mSetEE.size() != mesh1.edges.size())
			mSetEE.resize(mesh1.edges.size());

		for (int i = 0; i < mesh1.edges.size(); i++)
		{
			mSetEE[i].clear();
		}

		for (int i = 0; i < mPotentialEdgeContacts.size(); i++)
		{
			int e1 = mPotentialEdgeContacts[i].idx1;
			int e2 = mPotentialEdgeContacts[i].idx2;
			mSetEE[e1].push_back(e2);
		}

		#pragma omp parallel for
		for (int i = 0; i < mesh1.edges.size(); i++)
		{
			int i1 = mesh1.edges[i].i1;
			int i2 = mesh1.edges[i].i2;
			const Vector3& p = mesh1.vertices[i1];
			const Vector3& q = mesh1.vertices[i2];

			QuerySegment(p, q, mesh2, mSetEE[i], mEdgeInfos[i]);
		}
	}

	void MeshClosestPoints::ClosestPoints(const Mesh& mesh1, const Mesh& mesh2, AabbTree* root1, AabbTree* root2, float radTol, bool useSDF)
	{
		PROFILE_SCOPE("Closest points");

		// mesh1 is the "query" mesh and all the results (and collision flags) are relative to its features

		mPotentialContacts.clear();
		mPotentialTriContacts.clear();
		mPotentialEdgeContacts.clear();
		mTriTriCandidates.clear();
		{
			PROFILE_SCOPE("Test trees");
			TestTrees(mesh1, mesh2, root1, root2, radTol);
		}

		mVertexInfos.resize(mesh1.vertices.size());
		mTriangleInfos.resize(mesh1.indices.size() / 3);
		mEdgeInfos.resize(mesh1.edges.size());

		if (!useSDF)
			HandleCandidatePairs(mesh1, mesh2);
		else
			HandleCandidatePairsSDF(mesh1, mesh2);
	}

	bool CheckAdjacency(const Mesh& mesh, int vtx, int tri, int region)
	{
		int i1 = mesh.indices[tri * 3];
		int i2 = mesh.indices[tri * 3 + 1];
		int i3 = mesh.indices[tri * 3 + 2];
		if (i1 == vtx || i2 == vtx || i3 == vtx)
			return true;
		if (region == TR_VERTEX_A && mesh.AreAdjacent(vtx, i1))
			return true;
		if (region == TR_VERTEX_B && mesh.AreAdjacent(vtx, i2))
			return true;
		if (region == TR_VERTEX_C && mesh.AreAdjacent(vtx, i3))
			return true;
		if (region == TR_EDGE_AB && (mesh.AreAdjacent(vtx, i1) && mesh.AreAdjacent(vtx, i2)))
			return true;
		if (region == TR_EDGE_BC && (mesh.AreAdjacent(vtx, i2) && mesh.AreAdjacent(vtx, i3)))
			return true;
		if (region == TR_EDGE_AC && (mesh.AreAdjacent(vtx, i1) && mesh.AreAdjacent(vtx, i3)))
			return true;
		return false;
	}

	bool CheckSameTriangle(const Mesh& mesh, int vtx, int tri)
	{
		if (tri < 0)
			return false;
		int j1 = mesh.indices[tri * 3];
		int j2 = mesh.indices[tri * 3 + 1];
		int j3 = mesh.indices[tri * 3 + 2];
		return vtx == j1 || vtx == j2 || vtx == j3;
	}

	// returns false if the edges are not adjacent, so we can use the pair
	bool CheckEdgeAdjacency(ClosestEdgeToSegment& info, const Mesh& mesh1, int i1, int i2, const Mesh::Edge& edge1)
	{
		//info.branch = 0;
		const float eps = 1e-5f;
		int e = info.edge;
		if (e < 0)
			return true;

		const Mesh::Edge& edge2 = mesh1.edges[e];
		// check if they are the same edge
		//if (e == i) // never happens
		//	mEdgeInfos[i].distance = FLT_MAX;
		// check if they are adjacent
		if (i1 == edge2.i1 || i1 == edge2.i2 || i2 == edge2.i1 || i2 == edge2.i2)
		{
			info.distance = FLT_MAX;
			//info.branch = 1;
			return true;
		}
		// check if they are part of the same triangle (but not incident)					
		// TODO: the region is redundant
		if ((info.region & ER_VERTEX_P1) != 0 || info.coordsSegm.x < eps)
		{
			// one point is i1
			if (CheckSameTriangle(mesh1, i1, edge2.t1) || CheckSameTriangle(mesh1, i1, edge2.t2))
			{
				info.distance = FLT_MAX;
				//info.branch = 2;
				return true;
			}
		}
		if ((info.region & ER_VERTEX_Q1) != 0 || info.coordsSegm.y < eps)
		{
			// one point is i2
			if (CheckSameTriangle(mesh1, i2, edge2.t1) || CheckSameTriangle(mesh1, i2, edge2.t2))
			{
				info.distance = FLT_MAX;
				//info.branch = 3;
				return true;
			}
		}
		if ((info.region & ER_VERTEX_P2) != 0 || info.coordsMesh.x < eps)
		{
			// one point is edge.i1
			if (CheckSameTriangle(mesh1, edge2.i1, edge1.t1) || CheckSameTriangle(mesh1, edge2.i1, edge1.t2))
			{
				info.distance = FLT_MAX;
				//info.branch = 4;
				return true;
			}
		}
		if ((info.region & ER_VERTEX_Q2) != 0 || info.coordsMesh.y < eps)
		{
			// one point is edge.i2
			if (CheckSameTriangle(mesh1, edge2.i2, edge1.t1) || CheckSameTriangle(mesh1, edge2.i2, edge1.t2))
			{
				info.distance = FLT_MAX;
				//info.branch = 5;
				return true;
			}
		}
		return false;
	}

	void MeshClosestPoints::ClosestPoints(const Mesh& mesh, const AabbTree* tree, float radTol)
	{
		PROFILE_SCOPE("Closest points self");

		if (mCollFlags & CF_VERTICES)
		{
			PROFILE_SCOPE("Closest points self VT");
			mVertexInfos.clear();

			#pragma omp parallel for
			for (int i = 0; i < (int)mesh.vertices.size(); i++)
			{
				std::vector<ClosestTriangleToPoint> results;
				// TODO: tree accelerated version
				ClosestPointsOnMeshToPoint(mesh.vertices[i], mesh, results, radTol, i);
				for (int j = 0; j < results.size(); j++)
				{
					results[j].vtx = i; // add the vertex index
					if (results[j].tri >= 0)
					{
						if (!CheckAdjacency(mesh, i, results[j].tri, results[j].region))
						{
							#pragma omp critical
							mVertexInfos.push_back(results[j]);
						}
					}
				}
			}
		}

		if (mCollFlags & CF_EDGES)
		{
			PROFILE_SCOPE("Closest points self EE");
			mEdgeInfos.clear();

			#pragma omp parallel for
			for (int i = 0; i < mesh.edges.size(); i++)
			{
				const Mesh::Edge& edge1 = mesh.edges[i];
				int i1 = mesh.edges[i].i1;
				int i2 = mesh.edges[i].i2;
				const Vector3& p = mesh.vertices[i1];
				const Vector3& q = mesh.vertices[i2];
				ClosestEdgeToSegment info;
				info.edge1 = i;
				//ClosestPointOnMeshToSegmentAcc(p, q, mesh, tree, true, -1, info); // FIXME
				ClosestPointOnMeshToSegment(p, q, mesh, info, i);
				CheckEdgeAdjacency(info, mesh, i1, i2, edge1);
				
				#pragma omp critical
				mEdgeInfos.push_back(info);
			}
		}
	}


	void MeshClosestPoints::ClosestPoints(const Mesh& mesh1, const Mesh& mesh2, const AabbTree* tree, float radTol, bool sign)
	{
		PROFILE_SCOPE("Closest points");

		if (mCollFlags & CF_VERTICES)
		{
			mVertexInfos.clear();

			#pragma omp parallel for
			for (int i = 0; i < (int)mesh1.vertices.size(); i++)
			{
#ifndef CONCAVE_CLOSEST_PTS
				ClosestTriangleToPoint info = ClosestPointOnMeshToPointAcc(mesh1.vertices[i], mesh2, tree);
				info.vtx = i;
				if (info.tri >= 0)
				{
					if (sign)
					{
						Vector3 delta = mesh1.vertices[i] - info.closestPtMesh;
						if (delta.Dot(info.normal) < 0)
							info.distance *= -1;
					}
				}
				#pragma omp critical
				mVertexInfos.push_back(info);
#else				
				std::vector<ClosestTriangleToPoint> results;
				ClosestPointsOnMeshToPoint(mesh1.vertices[i], mesh2, results, radTol);
				for (int j = 0; j < results.size(); j++)
				{
					results[j].vtx = i;
					if (results[j].tri >= 0)
					{
						if (sign)
						{
							Vector3 delta = mesh1.vertices[i] - results[j].closestPtMesh;
							if (delta.Dot(results[j].normal) < 0)
								results[j].distance *= -1;
						}
					}
					#pragma omp critical
					mVertexInfos.push_back(results[j]);
				}
#endif
			}
		}

		if (mCollFlags & CF_TRIANGLES && &mesh1 != &mesh2)
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
				mTriangleInfos[i] = Geometry::ClosestPointOnMeshToTriangleAcc(a, b, c, mesh2, tree, mTriangleInfos[i].vertex);
				if (&mesh1 == &mesh2)
				{
					if (CheckAdjacency(mesh1, mTriangleInfos[i].vertex, i, mTriangleInfos[i].region))
						mTriangleInfos[i].distance = FLT_MAX;
				}
				else if (sign)
				{
					Vector3 delta = mTriangleInfos[i].closestPtTri - mTriangleInfos[i].closestPtMesh;
					if (delta.Dot(mTriangleInfos[i].normal) < 0)
						mTriangleInfos[i].distance *= -1;
				}
			}
		}

		if (mCollFlags & CF_EDGES)
		{
			mEdgeInfos.clear();

			#pragma omp parallel for
			for (int i = 0; i < mesh1.edges.size(); i++)
			{
				const Mesh::Edge& edge1 = mesh1.edges[i];
				int i1 = mesh1.edges[i].i1;
				int i2 = mesh1.edges[i].i2;
				const Vector3& p = mesh1.vertices[i1];
				const Vector3& q = mesh1.vertices[i2];
				ClosestEdgeToSegment info;
				info.edge1 = i;
				ClosestPointOnMeshToSegmentAcc(p, q, mesh2, tree, false, -1, info);
				if (&mesh1 == &mesh2)
				{
					CheckEdgeAdjacency(info, mesh1, i1, i2, edge1);
				}
				#pragma omp critical
				mEdgeInfos.push_back(info);
			}
		}
	}

} // namespace Geometry