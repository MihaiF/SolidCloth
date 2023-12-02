#include "ClothPatch.h"
#include "Engine/Types.h"
#include "Geometry/AabbTree.h"
#include <Engine/Profiler.h>
#include "ClothModel.h"
#include "CollisionWorld.h"

#include <stack>

#define EDGE_CCD

using namespace Geometry;

namespace Physics
{
	void ClothModel::DetectCollisions()
	{
		// Compute the cloth own AABB tree
		if (mCollFlags & (CF_VERTICES | CF_TRIANGLES))
		{
			int flags = ATF_VERTICES;
			if (mCollFlags & CF_TRIANGLES)
				flags |= ATF_TRIANGLES;
			if (mCollFlags & CF_SELF)
				flags |= ATF_TRIANGLES | ATF_EDGES;
			ComputeTree(flags, 10, mThickness);
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
	}

	void ClothModel::ExternalCollisions()
	{
		UpdateMesh(mOwnerPatch->GetMesh(), true, Vector3(), true);

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
				if (mesh->tree == NULL || mesh->invalidate)
				{
					// Compute the mesh AABB tree (if needed)
					mesh->invalidate = false;
					if (mesh->tree)
						delete mesh->tree;
					int flags = ATF_TRIANGLES;
					if (mCollFlags & CF_TRIANGLES)
						flags |= ATF_VERTICES;
					if (mCollFlags & CF_EDGES)
						flags |= ATF_EDGES;
					mesh->tree = ComputeMeshTree(*mesh->mesh, flags, 10, mThickness);
				}
				
				//SDFCollisions(MeshSDF(*mesh->mesh, mesh->tree));
				MeshCollisions(*mesh);
			}
		}
	}

	void ClothModel::MeshCollisions(const CollisionMesh& collMesh)
	{
		std::vector<float> tvs(mParticles.size());
		float maxTv = 0;
		for (int i = 0; i < mParticles.size(); i++)
		{
			tvs[i] = (mParticles[i].pos - mParticles[i].prev).Length();
			if (tvs[i] > maxTv)
				maxTv = tvs[i];
		}

		float radTol = mThickness + std::max(mTolerance, maxTv * 1.1f);
		mClosestPoints.SetParams(mCollFlags);
		mClosestPoints.ClosestPoints(mOwnerPatch->GetMesh(), *collMesh.mesh, mTree, collMesh.tree, radTol, true);
		//mClosestPoints.ClosestPoints(mOwnerPatch->GetMesh(), *collMesh.mesh, collMesh.tree, true);

		for (int j = 0; j < mClosestPoints.mVertexInfos.size(); j++)
		{
			// compute the threshold
			float radTol = mThickness + std::max(mTolerance, tvs[j] * 1.1f);
			
			float dist = mClosestPoints.mVertexInfos[j].distance;
			if (dist < radTol && dist >= 0)
			{
				AddContact(j, mClosestPoints.mVertexInfos[j].closestPtMesh, mClosestPoints.mVertexInfos[j].normal, Vector3());
			}
		}

		for (int j = 0; j < mClosestPoints.mTriangleInfos.size(); j++)
		{
			int i1 = mTriangles[j].i1;
			int i2 = mTriangles[j].i2;
			int i3 = mTriangles[j].i3;
			
			// compute the threshold
			float tv = std::max(tvs[i1], std::max(tvs[i2], tvs[i3]));
			float radTol = mThickness + std::max(mTolerance, tv * 1.1f);
			
			float dist = mClosestPoints.mTriangleInfos[j].distance;
			if (dist < radTol && dist >= 0)
			{
				// this is the j-th triangle
				AddTriContact(i1, i2, i3, mClosestPoints.mTriangleInfos[j].closestPtMesh, 
					mClosestPoints.mTriangleInfos[j].normal, mClosestPoints.mTriangleInfos[j].baryCoords, Vector3());
			}
		}

		for (int j = 0; j < mClosestPoints.mEdgeInfos.size(); j++)
		{
			int i1 = mEdges[j].i1;
			int i2 = mEdges[j].i2;

			// compute the threshold
			float tv = std::max(tvs[i1], tvs[i2]);
			float radTol = mThickness + std::max(mTolerance, tv * 1.1f);

			float dist = mClosestPoints.mEdgeInfos[j].distance;
			if (dist < radTol && dist >= 0)
			{
				// this is the j-th edge
				AddEdgeContact(i1, i2, mClosestPoints.mEdgeInfos[j].closestPtMesh, mClosestPoints.mEdgeInfos[j].normal,
					mClosestPoints.mEdgeInfos[j].coordsSegm, Vector3());
			}
		}
	}

	void ClothModel::WallCollisions(const Aabb3& walls)
	{
		// TODO: accelerate; triangle collisions
		const float r = mThickness + mTolerance;
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			const Particle& particle = mParticles[i];
			if (particle.pos.Y() - r <= walls.min.Y())
				AddContact(i, Vector3(particle.pos.X(), walls.min.Y(), particle.pos.Z()), Vector3(0, 1, 0), Vector3::Zero());
			//if (particle.pos.Y() + r >= WALL_TOP)
			//	AddContact((ParticleIdx)i, Vector3(particle.pos.X(), WALL_TOP, particle.pos.Z()), Vector3(0, -1, 0));
			if (particle.pos.X() - r <= walls.min.X())
				AddContact(i, Vector3(walls.min.X(), particle.pos.Y(), particle.pos.Z()), Vector3(1, 0, 0), Vector3::Zero());
			if (particle.pos.X() + r >= walls.max.X())
				AddContact(i, Vector3(walls.max.X(), particle.pos.Y(), particle.pos.Z()), Vector3(-1, 0, 0), Vector3::Zero());
			if (particle.pos.Z() - r <= walls.min.Z())
				AddContact(i, Vector3(particle.pos.X(), particle.pos.Y(), walls.min.Z()), Vector3(0, 0, 1), Vector3::Zero());
			if (particle.pos.Z() + r >= walls.max.Z())
				AddContact(i, Vector3(particle.pos.X(), particle.pos.Y(), walls.max.Z()), Vector3(0, 0, -1), Vector3::Zero());
		}
	}

	void ClothModel::SphereCollisions(const Vector3& sphPos, float sphRad)
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

	void ClothModel::TestTreeNodeT(const CollisionMesh& collMesh, const AabbTree* node, size_t i)
	{
		const float radTol = mThickness + mTolerance;
		const Vector3 extrude(radTol);
		Vector3 normal, p;
		float d;
		Vector3 v = mParticles[i].prev;
		if (PointInAabb3D(node->box.min, node->box.max, v))
		{
			for (size_t j = 0; j < node->triangles.size(); j++)
			{
				int k = node->triangles[j].idx * 3;
				const uint16 i1 = collMesh.mesh->indices[k];
				const uint16 i2 = collMesh.mesh->indices[k + 1];
				const uint16 i3 = collMesh.mesh->indices[k + 2];
				Vector3 v1 = collMesh.mesh->vertices[i1];
				Vector3 v2 = collMesh.mesh->vertices[i2];
				Vector3 v3 = collMesh.mesh->vertices[i3];

				Vector3 minV = vmin(vmin(v1, v2), v3) - extrude;
				Vector3 maxV = vmax(vmax(v1, v2), v3) + extrude;
				if (!PointInAabb3D(minV, maxV, v)) continue;

				Vector3 n = (v2 - v1).Cross(v3 - v1); // TODO: store normals
				n.Normalize();
				// if coming from inside skip
				if (n.Dot(mParticles[i].prev - v1) < 0)
					continue;
				Math::BarycentricCoords coords;
				//Vector3 dir = mParticles[i].pos - mParticles[i].prev;
				bool intersect;			
				intersect = IntersectSphereTriangle(v, radTol, v1, v2, v3, normal, p, d, coords);
				//if (!intersect)
				//	intersect = IntersectSweptSphereTriangle(mParticles[i].prev, radTol, dir, v1, v2, v3, p, n, d, coords); // TODO: use velocity instead of prevPos
				if (intersect)
				{
					AddContact(i, p, normal, Vector3::Zero());
				}
			}
		}

		if (node->left)
			TestTreeNodeT(collMesh, node->left, i);
		if (node->right)
			TestTreeNodeT(collMesh, node->right, i);
	}

	void ClothModel::TestTreeNodeV(const Mesh* mesh, const AabbTree* node, size_t j)
	{
		const float radTol = mThickness + mTolerance;
		const Vector3 extrude(radTol);
		Vector3 n, p;
		float d;

		const uint16 i1 = mTriangles[j].i1;
		const uint16 i2 = mTriangles[j].i2;
		const uint16 i3 = mTriangles[j].i3;
		Vector3 v1 = mParticles[i1].prev;
		Vector3 v2 = mParticles[i2].prev;
		Vector3 v3 = mParticles[i3].prev;

		// build AABB
		Vector3 minV = vmin(vmin(v1, v2), v3) - extrude;
		Vector3 maxV = vmax(vmax(v1, v2), v3) + extrude;
		Aabb3 box(minV, maxV);

		if (AabbOverlap3D(node->box, box))
		{
			for (size_t i = 0; i < node->vertices.size(); i++)
			{
				uint32 idx = node->vertices[i].idx;
				Vector3 v = mesh->vertices[idx];

				if (!PointInAabb3D(minV, maxV, v))
					continue;

				Math::BarycentricCoords bar;
				bool intersect;			
				intersect = IntersectSphereTriangle(v, radTol, v1, v2, v3, n, p, d, bar);
				if (intersect)
				{
					if (n.Dot(mesh->normals[idx]) < 0) // prevents hooking
						n.Flip();
					TriContact contact;
					contact.normal = n;
					contact.point = v;
					contact.i1 = i1;
					contact.i2 = i2;
					contact.i3 = i3;
					contact.w1 = bar.x;
					contact.w2 = bar.y;
					contact.w3 = bar.z;
					mTriContacts.push_back(contact);
				}
			}
		}

		if (node->left)
			TestTreeNodeV(mesh, node->left, j);
		if (node->right)
			TestTreeNodeV(mesh, node->right, j);
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
		const int maxPrims = 40;
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
	void ClothModel::SDFCollisions(const SDFType& sdf,
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
				int vtx = mCacheVT[i];
				float dist = sdf.QueryPoint(mParticles[i].prev, p, n, vtx);
				// we are not limiting to the interior of the triangle, so we can catch VE or VV cases that may resurface later
				
				// compute the threshold
				float tv = (mParticles[i].prev - mParticles[i].pos).Length();
				float radTol = mThickness + std::max(mTolerance, tv * 1.1f);

				// check if the distance to the sphere surface is less than our threshold
				if (dist < radTol && (dist >= 0 || handleInside))
				#pragma omp critical
				{
					int idx = AddContact(i, p, n, Vector3::Zero());
				}
				else if (handleCurrent)
				{
					Vector3 p1, n1;
					float dist1 = sdf.QueryPoint(mParticles[i].pos, p1, n1, vtx);

					if (dist1 < radTol && (dist1 >= 0 || handleInside))
					#pragma omp critical
					{
						int idx = AddContact(i, p1, n1, Vector3::Zero());
					}
				}
			}
		}

		const float eps = 0.001f; // threshold to cut barycentric coordinate - needs to be tweaked

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
					AddEdgeContact(i1, i2, cp, n, coords, Vector3::Zero());
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
						AddEdgeContact(i1, i2, cp, n, coords, Vector3::Zero());
					}

				}
			}
		}

		// we must handle borders for triangles, otherwise we can miss a closest point
		// but only if edge collisions are disabled, otherwise the edges are caught there (but not the vertices?)
		handleBorder |= (mCollFlags & CF_EDGES) == 0;
		
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
				int tri = mCacheTV[i];
				float dist = sdf.QueryTriangle(v1, v2, v3, closestPt, normal, coords, tri);
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
					int idx = AddTriContact(i1, i2, i3, closestPt, normal, coords, Vector3::Zero());
				}
				else if (handleCurrent)
				{
					const Vector3& w1 = p1.pos;
					const Vector3& w2 = p2.pos;
					const Vector3& w3 = p3.pos;

					dist = sdf.QueryTriangle(w1, w2, w3, closestPt, normal, coords, tri);
					// 'closestPt' is a vertex on the mesh

					// Testing the previous positions and including negative distance give the best results so far
					if (dist < radTol && (dist >= 0 || handleInside) && 
						((coords.x > eps && coords.y > eps && coords.z > eps) || handleBorder))
					#pragma omp critical
					{
						int idx = AddTriContact(i1, i2, i3, closestPt, normal, coords, Vector3::Zero());
					}
				}
			}
		}
	}

}