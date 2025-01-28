#include "ClothPatch.h"
#include "Engine/Types.h"
#include "Geometry/AabbTree.h"
#include <Engine/Profiler.h>
#include "ClothModel.h"
#include "CollisionWorld.h"

#include <stack>

#define EDGE_CCD

using namespace Geometry;
using namespace Math;

namespace Physics
{
	void ClothModel::DetectCollisions()
	{
		Mesh* prevMesh = mOwnerPatch->GetPrevMesh();
		if (prevMesh != nullptr)
			UpdateMesh(*prevMesh, mOwnerPatch->IsQuadMesh(), Vector3(), true); // TODO: update it earlier above
			
		// Compute the cloth own AABB tree
		if (mCollFlags & (CF_VERTICES | CF_TRIANGLES | CF_SELF))
		{
			int flags = ATF_VERTICES | ATF_TRIANGLES | ATF_EDGES;
			ComputeTree(flags, 10, mThickness + mTolerance * 0.5f);
		}

		// prepare buffers for caching closest feature
		if (mCacheVT.size() != mParticles.size())
			mCacheVT.resize(mParticles.size(), -1);
		if (mCacheTV.size() != mTriangles.size())
			mCacheTV.resize(mTriangles.size(), -1);
		if (mCacheEE.size() != mEdges.size())
			mCacheEE.resize(mEdges.size(), -1);

		if (!mSolveSDFContacts)
			ExternalCollisions();

		if (mCollFlags & CF_SELF)
		{
			mSelfTris.clear();
			mSelfEdges.clear();
			if (prevMesh != nullptr)
				MeshCollisions(*prevMesh, mTree, CF_VERTICES | CF_EDGES);
		}
	}

	void ClothModel::ExternalCollisions()
	{
		mContacts.clear();
		mTriContacts.clear();
		mEdgeContacts.clear();
		for (size_t i = 0; i < mOwnerPatch->GetCollWorld().GetNumCollidables(); i++)
		{
			const Collidable* collider = mOwnerPatch->GetCollWorld().GetCollidable(i);
			if ((collider->mType == CT_WALLS) && (mCollFlags & CF_WALLS))
			{
				const Walls* walls = (const Walls*)collider;
				WallCollisions(walls->mBox);
			}
			else if (collider->mType == CT_SPHERE)
			{
				const Sphere* sph = (const Sphere*)collider;
				SDFCollisions(SphereSDF(*sph));
			}
			else if (collider->mType == CT_CAPSULE)
			{
				const Capsule* cap = (const Capsule*)collider;
				SDFCollisions(CapsuleSDF(*cap));
			}
			else if (collider->mType == CT_SDF)
			{
				const CollisionSDF* sdf = (const CollisionSDF*)collider;
				SDFCollisions(GridSDF(*sdf->sdf));
			}
			else if (collider->mType == CT_MESH)
			{
				CollisionMesh* mesh = (CollisionMesh*)collider;
				mesh->Update(mThickness, mCollFlags);
				//SDFCollisions(MeshSDF(*mesh->mesh, mesh->triangleTree, mesh->edgeTree, mesh->vertexTree), mesh->mesh);
				MeshCollisions(*mesh->mesh, mesh->tree, mCollFlags);
				//MeshCollisionsCCD(*mesh);
			}
		}
	}

	void ClothModel::MeshCollisions(const Mesh& mesh, AabbTree* triangleTree, int collFlags)
	{
		std::vector<float> tvs(mParticles.size());
		float maxTv = 0;
		for (int i = 0; i < mParticles.size(); i++)
		{
			tvs[i] = (mParticles[i].pos - mParticles[i].prev).Length();
			if (tvs[i] > maxTv)
				maxTv = tvs[i];
		}

		float radTol = mThickness + std::max(mTolerance, maxTv * 1.1f); // TODO: is this tolerance too big?
		mClosestPoints.SetParams(collFlags);
		const Mesh* clothMesh = mOwnerPatch->GetPrevMesh();

		if (&mesh == clothMesh)
			mClosestPoints.ClosestPoints(*clothMesh, triangleTree, radTol);
		else
			mClosestPoints.ClosestPoints(*clothMesh, mesh, triangleTree, 2 * radTol, false);

		for (int j = 0; j < mClosestPoints.mVertexInfos.size(); j++)
		{
			const auto& info = mClosestPoints.mVertexInfos[j];

			float dist = info.distance;
			if (dist < radTol && dist >= 0)
			{
				// this is the j-th cloth vertex
				if (&mesh == clothMesh)
				{
					SelfContact selfTri;
					int tri = info.tri;
					selfTri.i1 = mesh.indices[tri * 3];
					selfTri.i2 = mesh.indices[tri * 3 + 1];
					selfTri.i3 = mesh.indices[tri * 3 + 2];
					selfTri.i4 = info.vtx;
					selfTri.normal = info.normal;
					selfTri.w1 = info.baryOnMesh.x;
					selfTri.w2 = info.baryOnMesh.y;
					selfTri.w3 = info.baryOnMesh.z;
					selfTri.side = info.side;
					AddSelfTriangle(selfTri);
				}
				else
					AddContact(info.vtx, info.closestPtMesh, info.normal, Vector3(), info.tri, &mesh);
			}
		}

		for (int j = 0; j < mClosestPoints.mTriangleInfos.size(); j++)
		{
			int i1 = mTriangles[j].i1;
			int i2 = mTriangles[j].i2;
			int i3 = mTriangles[j].i3;

			// compute the threshold
			float tv = std::max(tvs[i1], std::max(tvs[i2], tvs[i3]));
			float radTol = mThickness + std::max(mTolerance, tv * 1.2f);

			float dist = mClosestPoints.mTriangleInfos[j].distance;
			if (dist < radTol && dist >= 0)
			{
				// this is the j-th cloth triangle
				if (&mesh == clothMesh)
				{
					// do nothing
				}
				else
					AddTriContact(i1, i2, i3, mClosestPoints.mTriangleInfos[j].closestPtMesh,
						mClosestPoints.mTriangleInfos[j].normal, mClosestPoints.mTriangleInfos[j].baryCoords, Vector3(),
					mClosestPoints.mTriangleInfos[j].vertex, &mesh);
			}
		}

		for (int j = 0; j < mClosestPoints.mEdgeInfos.size(); j++)
		{
			const auto& info = mClosestPoints.mEdgeInfos[j];
			int e1 = info.edge1;
			int i1 = mEdges[e1].i1;
			int i2 = mEdges[e1].i2;

			float dist = info.distance;
			if (dist < radTol && dist >= 0)
			{
				// this is the j-th cloth edge
				if (&mesh == clothMesh)
				{
					const Vector2& coords1 = info.coordsSegm;
					const Vector2& coords2 = info.coordsMesh;
					{
						int e2 = info.edge;
						const Mesh::Edge& edge2 = mesh.edges[e2];
						SelfContact selfEdge;
						selfEdge.i1 = i1;
						selfEdge.i2 = i2;
						selfEdge.i3 = edge2.i1;
						selfEdge.i4 = edge2.i2;
						selfEdge.w1 = coords1.y;
						selfEdge.w2 = coords2.y;
						selfEdge.normal = info.normal;
						selfEdge.side = info.side;
						AddSelfEdge(selfEdge);
					}
				}
				else
					AddEdgeContact(i1, i2, info.closestPtMesh, info.normal,
						info.coordsSegm, Vector3(), info.edge, &mesh);
			}
		}
	}

	void ClothModel::WallCollisions(const Aabb3& walls)
	{
		// TODO: accelerate; triangle collisions
		const float r = mThickness + mTolerance;
		Vector3 v;
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			const Particle& particle = mParticles[i];
			if (particle.pos.Y() - r <= walls.min.Y())
				AddContact(i, Vector3(particle.pos.X(), walls.min.Y(), particle.pos.Z()), Vector3(0, 1, 0), v);
			//if (particle.pos.Y() + r >= WALL_TOP)
			//	AddContact((ParticleIdx)i, Vector3(particle.pos.X(), WALL_TOP, particle.pos.Z()), Vector3(0, -1, 0));
			if (particle.pos.X() - r <= walls.min.X())
				AddContact(i, Vector3(walls.min.X(), particle.pos.Y(), particle.pos.Z()), Vector3(1, 0, 0), v);
			if (particle.pos.X() + r >= walls.max.X())
				AddContact(i, Vector3(walls.max.X(), particle.pos.Y(), particle.pos.Z()), Vector3(-1, 0, 0), v);
			if (particle.pos.Z() - r <= walls.min.Z())
				AddContact(i, Vector3(particle.pos.X(), particle.pos.Y(), walls.min.Z()), Vector3(0, 0, 1), v);
			if (particle.pos.Z() + r >= walls.max.Z())
				AddContact(i, Vector3(particle.pos.X(), particle.pos.Y(), walls.max.Z()), Vector3(0, 0, -1), v);
		}
	}

	void ClothModel::SphereCollisions(Vector3 sphPos, float sphRad)
	{
		const float radTol = mThickness + mTolerance;
		Vector3 n, p;
		float dSqr;
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			if (mParticles[i].invMass != 0 && IntersectSphereSphere(sphPos, sphRad, mParticles[i].pos, radTol, n, p, dSqr))
			{
				float dist = sqrtf(dSqr);
				int idx = AddContact(i, p, n, Vector3::Zero());
				if (dist < mThickness + sphRad)
					mContacts[idx].depth = mThickness + sphRad - dist;
			}
		}
	}

	// duplicated code
	void ClothModel::ComputeTree(int flags, int maxLevel, float tol)
	{
		PROFILE_SCOPE("ComputeTree");

		// delete the old tree
		if (mTree)
			delete mTree;

		Aabb3 bounds;
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			bounds.min = vmin(bounds.min, mParticles[i].pos);
			bounds.max = vmax(bounds.max, mParticles[i].pos);
		}
		AabbTree* root = new AabbTree;
		root->box = bounds;
		if (flags & ATF_TRIANGLES)
		{
			root->triangles.resize(mTriangles.size());
			for (size_t tri = 0; tri < root->triangles.size(); tri++)
			{
				int i1 = mTriangles[tri].i1;
				int i2 = mTriangles[tri].i2;
				int i3 = mTriangles[tri].i3;
				Vector3 x1 = mParticles[i1].prev;
				Vector3 x2 = mParticles[i2].prev;
				Vector3 x3 = mParticles[i3].prev;
				Vector3 y1 = mParticles[i1].pos;
				Vector3 y2 = mParticles[i2].pos;
				Vector3 y3 = mParticles[i3].pos;

				Aabb3 box;
				box.Add(x1);
				box.Add(x2);
				box.Add(x3);
				// swept box
				box.Add(y1);
				box.Add(y2);
				box.Add(y3);
				box.Extrude(tol);

				root->triangles[tri].idx = (uint32)tri;
				root->triangles[tri].box = box;
			}
		}
		if (flags & ATF_VERTICES)
		{
			root->vertices.resize(mParticles.size());
			for (size_t i = 0; i < mParticles.size(); i++)
			{
				Aabb3 ptBox;
				ptBox.Add(mParticles[i].prev);
				ptBox.Add(mParticles[i].pos);
				ptBox.Extrude(tol);

				root->vertices[i].idx = (uint32)i;
				root->vertices[i].box = ptBox;
			}
		}
		if (flags & ATF_EDGES)
		{
			root->edges.resize(mEdges.size());
			for (size_t i = 0; i < mEdges.size(); i++)
			{
				int i1 = mEdges[i].i1;
				int i2 = mEdges[i].i2;
				Vector3 x1 = mParticles[i1].pos;
				Vector3 x2 = mParticles[i2].pos;
				Vector3 y1 = mParticles[i1].prev;
				Vector3 y2 = mParticles[i2].prev;

				Aabb3 box;
				box.Add(x1);
				box.Add(x2);
				box.Add(y1);
				box.Add(y2);
				box.Extrude(tol);

				root->edges[i].idx = (uint32)i;
				root->edges[i].box = box;
			}
		}

		if (maxLevel <= 0)
		{
			mTree = root;
			return;
		}

		SplitNode(root, 1, maxLevel, flags, tol);

		mTree = root;
	}

	// duplicated code
	void ClothModel::SplitNode(AabbTree* root, int level, int maxLevel, int flags, float tol)
	{
		const int maxPrims = 30;
		if (root->triangles.size() < maxPrims
			&& root->vertices.size() < maxPrims
			&& root->edges.size() < maxPrims)
			return;

		// choose longest extent and splitting plane
		Vector3 ext = root->box.GetExtent();
		int dir = ext.GetMaxComponent();
		float center = 0.5f * (root->box.min[dir] + root->box.max[dir]);

		root->left = new AabbTree;
		root->right = new AabbTree;

		if (flags & ATF_TRIANGLES)
		{
			for (int i = 0; i < (int)root->triangles.size(); i++)
			{
				const BoxInfo& info = root->triangles[i];
				const Aabb3& box = info.box;

				if (box.min[dir] <= center)
				{
					root->left->triangles.push_back(info);
					root->left->box.Add(box);
				}
				if (box.max[dir] > center)
				{
					root->right->triangles.push_back(info);
					root->right->box.Add(box);
				}
			}
			root->triangles.clear();
		}

		if ((flags & ATF_EDGES) != 0)
		{
			for (size_t i = 0; i < root->edges.size(); i++)
			{
				const BoxInfo& info = root->edges[i];
				if (info.box.min[dir] <= center)
				{
					root->left->edges.push_back(info);
					root->left->box.Add(info.box);
				}
				if (info.box.max[dir] > center)
				{
					root->right->edges.push_back(info);
					root->right->box.Add(info.box);
				}
			}
			root->edges.clear();
		}

		if (flags & ATF_VERTICES)
		{
			for (int i = 0; i < (int)root->vertices.size(); i++)
			{
				const BoxInfo& info = root->vertices[i];
				if (info.box.min[dir] <= center)
				{
					root->left->vertices.push_back(info);
					root->left->box.Add(info.box);
				}
				if (info.box.max[dir] > center)
				{
					root->right->vertices.push_back(info);
					root->right->box.Add(info.box);
				}
			}
			root->vertices.clear();
		}

		if (root->left->triangles.empty() && root->left->vertices.empty() && root->left->edges.empty())
		{
			delete root->left;
			root->left = NULL;
		}
		if (root->right->triangles.empty() && root->right->vertices.empty() && root->right->edges.empty())
		{
			delete root->right;
			root->right = NULL;
		}

		if (level < maxLevel)
		{
			if (root->left)
				SplitNode(root->left, level + 1, maxLevel, flags, tol);
			if (root->right)
				SplitNode(root->right, level + 1, maxLevel, flags, tol);
		}
	}

	template<typename SDFType>
	void ClothModel::SDFCollisions(const SDFType& sdf, const Mesh* mesh,
		bool handleInside, // not so great for vertex collisions (but balanced by edge collisions); verty stable for Buddha when false
		bool handleCurrent, // give it up? we're using it for the threshold now; a bit better disabled for Buddha
		bool handleBorder) // better false for stability (Buddha); but better true for no missed collisions (old info?)
	{
		PROFILE_SCOPE("SDF collisions");

		if (mCollFlags & CF_VERTICES)
		{
			#pragma omp parallel for
			for (int i = 0; i < mParticles.size(); i++)
			{
				Vector3 p, n;
				int tri = mCacheVT[i];
				float dist = sdf.QueryPoint(mParticles[i].prev, p, n, tri);
				// we are not limiting to the interior of the triangle, so we can catch VE or VV cases that may resurface later

				// compute the threshold
				float tv = (mParticles[i].prev - mParticles[i].pos).Length();
				float radTol = mThickness + std::max(mTolerance, tv * 1.1f);

				// check if the distance to the sphere surface is less than our threshold
				if (dist < radTol && (dist >= 0 || handleInside))
				#pragma omp critical
				{
					AddContact(i, p, n, Vector3::Zero(), tri, mesh);
				}
				else if (handleCurrent)
				{
					Vector3 p1, n1;
					float dist1 = sdf.QueryPoint(mParticles[i].pos, p1, n1, tri);

					if (dist1 < radTol && (dist1 >= 0 || handleInside))
					#pragma omp critical
					{
						AddContact(i, p1, n1, Vector3::Zero(), tri, mesh);
					}
				}
			}
		}

		float eps = 0.02f; // threshold to cut barycentric coordinate - needs to be tweaked

		handleBorder |= (mCollFlags & CF_VERTICES) == 0;

		if (mCollFlags & CF_EDGES)
		{
			#pragma omp parallel for
			for (int i = 0; i < mEdges.size(); i++)
			{
				int i1 = mEdges[i].i1;
				int i2 = mEdges[i].i2;
				Vector3 p = mParticles[i1].prev;
				Vector3 q = mParticles[i2].prev;

				Vector2 coords;
				Vector3 cp, n;
				int edge = mCacheEE[i];
				float dist = sdf.QuerySegment(p, q, cp, n, coords, edge);

				// compute the threshold
				float tv1 = (mParticles[i1].prev - mParticles[i1].pos).Length();
				float tv2 = (mParticles[i2].prev - mParticles[i2].pos).Length();
				float tv = std::max(tv1, tv2);
				float radTol = mThickness + std::max(mTolerance, tv * 1.1f);

				if (dist < radTol && (dist >= 0 || handleInside) &&
					((coords.x > eps && coords.x < 1.f - eps && coords.y > eps && coords.y < 1.f - eps) || handleBorder))
				{
					#pragma omp critical
					AddEdgeContact(i1, i2, cp, n, coords, Vector3::Zero(), edge, mesh);
				}
				else if (handleCurrent)
				{
					p = mParticles[i1].pos;
					q = mParticles[i2].pos;

					dist = sdf.QuerySegment(p, q, cp, n, coords, edge);

					if (dist < radTol && (dist >= 0 || handleInside) &&
						((coords.x > eps && coords.x < 1.f - eps && coords.y > eps && coords.y < 1.f - eps) || handleBorder))
					{
						#pragma omp critical
						AddEdgeContact(i1, i2, cp, n, coords, Vector3::Zero(), edge, mesh);
					}
				}
			}
		}

		// we must handle borders for triangles, otherwise we can miss a closest point
		// but only if edge collisions are disabled, otherwise the edges are caught there (but not the vertices?)
		handleBorder |= (mCollFlags & CF_EDGES) == 0;
		eps = 0.001f; // different border epsilon  for triangles

		if (mCollFlags & CF_TRIANGLES)
		{
			#pragma omp parallel for
			for (int i = 0; i < mTriangles.size(); i++)
			{
				int i1 = mTriangles[i].i1;
				int i2 = mTriangles[i].i2;
				int i3 = mTriangles[i].i3;

				const Particle& p1 = mParticles[i1];
				const Particle& p2 = mParticles[i2];
				const Particle& p3 = mParticles[i3];

				const Vector3& v1 = p1.prev;
				const Vector3& v2 = p2.prev;
				const Vector3& v3 = p3.prev;

				Vector3 closestPt, normal, coords;
				int vtx = mCacheTV[i];
				float dist = sdf.QueryTriangle(v1, v2, v3, closestPt, normal, coords, vtx);
				// 'closestPt' is a vertex on the mesh

				// compute the threshold
				float tv1 = (mParticles[i1].prev - mParticles[i1].pos).Length();
				float tv2 = (mParticles[i2].prev - mParticles[i2].pos).Length();
				float tv3 = (mParticles[i3].prev - mParticles[i3].pos).Length();
				float tv = std::max(tv1, std::max(tv2, tv3));
				float radTol = mThickness + std::max(mTolerance, tv * 1.1f);

				// Testing the previous positions and including negative distance give the best results so far
				if (dist < radTol && (dist >= 0 || handleInside) &&
					((coords.x > eps && coords.y > eps && coords.z > eps) || handleBorder))
				#pragma omp critical
				{
					int idx = AddTriContact(i1, i2, i3, closestPt, normal, coords, Vector3::Zero(), vtx, mesh);
				}
				else if (handleCurrent)
				{
					const Vector3& w1 = p1.pos;
					const Vector3& w2 = p2.pos;
					const Vector3& w3 = p3.pos;

					dist = sdf.QueryTriangle(w1, w2, w3, closestPt, normal, coords, vtx);
					// 'closestPt' is a vertex on the mesh

					// Testing the previous positions and including negative distance give the best results so far
					if (dist < radTol && (dist >= 0 || handleInside) &&
						((coords.x > eps && coords.y > eps && coords.z > eps) || handleBorder))
					#pragma omp critical
					{
						int idx = AddTriContact(i1, i2, i3, closestPt, normal, coords, Vector3::Zero(), vtx, mesh);
					}
				}
			}
		}
	}

}