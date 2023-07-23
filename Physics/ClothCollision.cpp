#include "ClothPatch.h"
#include "Engine/Types.h"
#include "Geometry/AabbTree.h"
#include <Engine/Profiler.h>
#include "ClothModel.h"

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
			ComputeTree(flags, 10, mThickness);
		}

		ExternalCollisions();
	}

	void ClothModel::ExternalCollisions()
	{
		mTriContacts.clear();
		for (size_t i = 0; i < mCollidables.size(); i++)
		{
			if ((mCollidables[i]->mType == CT_WALLS) && (mCollFlags & CF_WALLS))
			{
				const Walls* walls = (const Walls*)mCollidables[i].get();
				WallCollisions(walls->mBox);
			}
			else if (mCollidables[i]->mType == CT_SPHERE)
			{
				const Sphere* sph = (const Sphere*)mCollidables[i].get();
				SDFCollisions(SphereSDF(*sph));
			}
			else if (mCollidables[i]->mType == CT_CAPSULE)
			{
				const Capsule* cap = (const Capsule*)mCollidables[i].get();
				SDFCollisions(CapsuleSDF(*cap));
			}
			else if (mCollidables[i]->mType == CT_SDF)
			{
				const CollisionSDF* sdf = (const CollisionSDF*)mCollidables[i].get();
				SDFCollisions(GridSDF(*sdf->sdf));
			}
			else if (mCollidables[i]->mType == CT_MESH)
			{
				CollisionMesh* mesh = (CollisionMesh*)mCollidables[i].get();
				if (mesh->tree == NULL || mesh->invalidate)
				{
					// Compute the mesh AABB tree (if needed)
					mesh->invalidate = false;
					if (mesh->tree)
						delete mesh->tree;
					int flags = ATF_TRIANGLES;
					if (mCollFlags & CF_TRIANGLES)
						flags |= ATF_VERTICES;
					mesh->tree = ComputeMeshTree(*mesh->mesh, flags, 10, mThickness);
				}
				
				//SDFCollisions(MeshSDF(*mesh->mesh, mesh->tree));
				MeshCollisions(*mesh, Vector3());
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
				AddContact(i, Vector3(particle.pos.X(), walls.min.Y(), particle.pos.Z()), Vector3(0, 1, 0));
			//if (particle.pos.Y() + r >= WALL_TOP)
			//	AddContact((ParticleIdx)i, Vector3(particle.pos.X(), WALL_TOP, particle.pos.Z()), Vector3(0, -1, 0));
			if (particle.pos.X() - r <= walls.min.X())
				AddContact(i, Vector3(walls.min.X(), particle.pos.Y(), particle.pos.Z()), Vector3(1, 0, 0));
			if (particle.pos.X() + r >= walls.max.X())
				AddContact(i, Vector3(walls.max.X(), particle.pos.Y(), particle.pos.Z()), Vector3(-1, 0, 0));
			if (particle.pos.Z() - r <= walls.min.Z())
				AddContact(i, Vector3(particle.pos.X(), particle.pos.Y(), walls.min.Z()), Vector3(0, 0, 1));
			if (particle.pos.Z() + r >= walls.max.Z())
				AddContact(i, Vector3(particle.pos.X(), particle.pos.Y(), walls.max.Z()), Vector3(0, 0, -1));
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
				int idx = AddContact(i, p, n);
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
					AddContact(i, p, normal);
				}
			}
		}

		if (node->left)
			TestTreeNodeT(collMesh, node->left, i);
		if (node->right)
			TestTreeNodeT(collMesh, node->right, i);
	}

#define NARROW_PHASE
#define NARROW_PHASE_TRIS

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

	void ClothModel::TestTrees(const Mesh* mesh, AabbTree* node1, AabbTree* node2)
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
				TestTrees(mesh, node1->left, node2);
			if (node1->right)
				TestTrees(mesh, node1->right, node2);
		}
		else if (node2->triangles.empty() && node2->vertices.empty())
		{
			if (node2->left)
				TestTrees(mesh, node1, node2->left);
			if (node2->right)
				TestTrees(mesh, node1, node2->right);
		}
		else
		{
			const float radTol = mThickness + mTolerance;
			const Vector3 extrude(radTol);

			if ((mCollFlags & CF_VERTICES) && !node1->vertices.empty() && !node2->triangles.empty())
			{
				Vector3 bndMin = node2->box.min - extrude;
				Vector3 bndMax = node2->box.max + extrude;
				for (size_t i = 0; i < node1->vertices.size(); i++)
				{
					int idx = node1->vertices[i].idx;
					const Vector3& v = mParticles[idx].prev;
					if (!PointInAabb3D(bndMin, bndMax, v)) // isn't this redundant?
						continue;
					for (size_t j = 0; j < node2->triangles.size(); j++)
					{
						int k = node2->triangles[j].idx * 3;
#ifdef NARROW_PHASE
						PrimitivePair pair;
						pair.idx1 = idx;
						pair.idx2 = k;
						mPotentialContacts.push_back(pair);
#else
						// TODO: use same code
						const uint16 i1 = mesh->indices[k];
						const uint16 i2 = mesh->indices[k + 1];
						const uint16 i3 = mesh->indices[k + 2];
						const Vector3& v1 = mesh->vertices[i1];
						const Vector3& v2 = mesh->vertices[i2];
						const Vector3& v3 = mesh->vertices[i3];

						Vector3 minV = vmin(vmin(v1, v2), v3) - extrude;
						Vector3 maxV = vmax(vmax(v1, v2), v3) + extrude;
						if (!PointInAabb3D(minV, maxV, v)) continue;

						BarycentricCoords coords;
						bool intersect = IntersectSphereTriangle(v, radTol, v1, v2, v3, n, p, d, coords);
						if (intersect)
						{
							n = coords.u * mesh->normals[mesh->indices[k]] + 
								coords.v * mesh->normals[mesh->indices[k + 1]] +
								coords.w * mesh->normals[mesh->indices[k + 2]];
							mParticles[idx].collided = AddContact(idx, p, n) + 1;
						}
#endif
					}
				}
			}

			if (mCollFlags & CF_TRIANGLES)
			{
				for (size_t j = 0; j < node1->triangles.size(); j++)
				{
					int tri = node1->triangles[j].idx;
					const uint16 i1 = mTriangles[tri].i1;
					const uint16 i2 = mTriangles[tri].i2;
					const uint16 i3 = mTriangles[tri].i3;
					Vector3 v1 = mParticles[i1].prev;
					Vector3 v2 = mParticles[i2].prev;
					Vector3 v3 = mParticles[i3].prev;

					Vector3 n1 = (v2 - v1).Cross(v3 - v1);
					n1.Normalize();

					//// build AABB
					//Vector3 minV = vmin(vmin(v1, v2), v3);// - extrude;
					//Vector3 maxV = vmax(vmax(v1, v2), v3);// + extrude;

					for (size_t i = 0; i < node2->vertices.size(); i++)
					{
						int idx = node2->vertices[i].idx;
						Vector3 v = mesh->vertices[idx];

						//float tol = (mParticles[i1].pos - v1).Length();
						//Vector3 ext(mThickness + tol);

						//if (!PointInAabb3D(minV - ext, maxV + ext, v))
						//	continue;
#ifdef NARROW_PHASE_TRIS
						PrimitivePair pair;
						pair.idx1 = tri;
						pair.idx2 = node2->vertices[i].idx;
						mPotentialTriContacts.push_back(pair);
#else
						if (n1.Dot(v - v1) < 0)
							continue;

						BarycentricCoords bar;
						bool intersect;			
						// TODO: use squares
						//float tol = mesh->velocities[idx].Length() + 
							//max((mParticles[i1].pos - v1).Length(), max((mParticles[i2].pos - v2).Length(), (mParticles[i3].pos - v3).Length()));
						Vector3 n, p;
						float d;
						intersect = IntersectSphereTriangle(v, mThickness + mTolerance, v1, v2, v3, n, p, d, bar);
						if (intersect)
						{
							//if (n.Dot(mesh->normals[idx]) < 0) // prevents hooking
							//{
							//	//n.Flip();
							//	n = mesh->normals[idx];
							//}
							n = n1;
							n.Flip();
							TriContact contact;
							contact.normal = n;
							contact.point = v;
							contact.i1 = i1;
							contact.i2 = i2;
							contact.i3 = i3;
							contact.w1 = bar.u;
							contact.w2 = bar.v;
							contact.w3 = bar.w;
							if (!mesh->velocities.empty())
								contact.vel = mesh->velocities[node2->vertices[i]];
							else
								contact.vel.SetZero();
							mTriContacts.push_back(contact);
						}
#endif
					}
				}
			}
		}
	}

	void ClothModel::ClothVertexVsMeshTriangle(const Mesh* mesh, int vertexIndex, int triangleIndex)
	{
		// TODO: use SDF abstraction
		const Vector3& v = mParticles[vertexIndex].prev;
		const uint16 i1 = mesh->indices[triangleIndex];
		const uint16 i2 = mesh->indices[triangleIndex + 1];
		const uint16 i3 = mesh->indices[triangleIndex + 2];
		const Vector3& v1 = mesh->vertices[i1];
		const Vector3& v2 = mesh->vertices[i2];
		const Vector3& v3 = mesh->vertices[i3];
		const Vector3& w = mParticles[vertexIndex].pos;

		float tol = (w - v).Length();
		float radTol = mThickness + tol + mTolerance;
		const Vector3 extrude(radTol);

		Vector3 minV = vmin(vmin(v1, v2), v3) - extrude;
		Vector3 maxV = vmax(vmax(v1, v2), v3) + extrude;
		if (!PointInAabb3D(minV, maxV, v))
			return;

		Vector3 n1 = (v2 - v1).Cross(v3 - v1);
		n1.Normalize();
		// if coming from inside skip
		BarycentricCoords coords;
		Vector3 n, p;
		float d;
		bool intersect = false;
		if (n1.Dot(v - v1) > 0)
			intersect = IntersectSphereTriangle(v, radTol, v1, v2, v3, n, p, d, coords);

		if (intersect)
		#pragma omp critical
		{
			if (mParticles[vertexIndex].collided)
			{
				int ctIdx = mParticles[vertexIndex].collided - 1;
				float depthNew = n.Dot(v - p);
				float depthOld = mContacts[ctIdx].normal.Dot(v - mContacts[ctIdx].point);
				if (depthNew < depthOld)
				{
					mContacts[ctIdx].point = p;
					mContacts[ctIdx].normal = n;
				}
			}
			else
			{
				int cid = AddContact(vertexIndex, p, n);
				if (!mesh->velocities.empty())
				{
					Vector3 vel = coords.x * mesh->velocities[mesh->indices[triangleIndex]] +
						coords.y * mesh->velocities[mesh->indices[triangleIndex + 1]] +
						coords.z * mesh->velocities[mesh->indices[triangleIndex + 2]];
					mContacts[cid].vel = vel;
				}
				mParticles[vertexIndex].collided = cid + 1;
			}
		}
	}

	void ClothModel::MeshCollisions(const CollisionMesh& collMesh, const Vector3& meshOffset)
	{
		PROFILE_SCOPE("MeshCollisions");
		mPotentialContacts.clear();
		mPotentialTriContacts.clear();
		{
			PROFILE_SCOPE("Tree traversal");
			TestTrees(collMesh.mesh, mTree, collMesh.tree);
		}

		if (1/*!mUseCL*/)
		{
#ifdef NARROW_PHASE
			PROFILE_SCOPE("Narrow phase");
			const Mesh* mesh = collMesh.mesh;
			for (size_t i = 0; i < GetNumParticles(); i++)
			{
				mParticles[i].collided = false;
			}
			#pragma omp parallel for
			for (int i = 0; i < (int)mPotentialContacts.size(); i++)
			{
				int k = mPotentialContacts[i].idx2;
				int idx = mPotentialContacts[i].idx1;
				ClothVertexVsMeshTriangle(mesh, idx, k);
			}
#endif
			// TODO: can process the two lists in parallel (a single for?)

#ifdef NARROW_PHASE_TRIS
			//const Mesh* mesh = collMesh.mesh;
			// triangle contacts
			#pragma omp parallel for
			for (int i = 0; i < (int)mPotentialTriContacts.size(); i++)
			{
				int tri = mPotentialTriContacts[i].idx1;
				const uint16 i1 = mTriangles[tri].i1;
				const uint16 i2 = mTriangles[tri].i2;
				const uint16 i3 = mTriangles[tri].i3;
				const Vector3& v1 = mParticles[i1].prev;
				const Vector3& v2 = mParticles[i2].prev;
				const Vector3& v3 = mParticles[i3].prev;
				const Vector3& w1 = mParticles[i1].pos;
				const Vector3& w2 = mParticles[i2].pos;
				const Vector3& w3 = mParticles[i3].pos;

				float tol = (w1 - v1).Length();
				float radTol = mThickness + tol;
				int idx = mPotentialTriContacts[i].idx2;
				Vector3 v = mesh->vertices[idx];

				Vector3 minV = vmin(vmin(v1, v2), v3);
				Vector3 maxV = vmax(vmax(v1, v2), v3);
				Vector3 ext(mThickness + tol);

				if (!PointInAabb3D(minV - ext, maxV + ext, v))
					continue;

				Vector3 n1 = (v2 - v1).Cross(v3 - v1);
				n1.Normalize();
				
				if (n1.Dot(v - v1) < 0)
					continue;

				BarycentricCoords bar;
				Vector3 n, p;
				float d;
				bool intersect;			
				intersect = IntersectSphereTriangle(v, radTol, v1, v2, v3, n, p, d, bar);
				if (!intersect)
				{
					intersect = IntersectSphereTriangle(v, radTol, w1, w2, w3, n, p, d, bar);
				}
				if (intersect)
				{
					//if (mesh->normals[idx].Dot(n1) < 0)
					n1.Flip();
					n = n1;
					TriContact contact;
					contact.normal = n;
					contact.point = v;
					contact.i1 = i1;
					contact.i2 = i2;
					contact.i3 = i3;
					contact.w1 = bar.x;
					contact.w2 = bar.y;
					contact.w3 = bar.z;
					if (!mesh->velocities.empty())
						contact.vel = mesh->velocities[idx];
					else
						contact.vel.SetZero();
					#pragma omp critical
					mTriContacts.push_back(contact);
				}
			}
#endif // NARROW_PHASE_TRIS
		}
		//else if (!mPotentialContacts.empty())
		//{
		//	PROFILE_SCOPE("CL NarrowPhase");
		//	// TODO: only one function
		//	collCL.CopyBuffers(mPotentialContacts, mParticles);
		//	collCL.NarrowPhase(radTol);
		//	collCL.ReadBuffers(mPotentialContacts.size(), mContacts);
		//}
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
	void ClothModel::SDFCollisions(const SDFType& sdf)
	{
		PROFILE_SCOPE("SDF collisions");
		
		const bool handleInside = true;
		const bool handleCurrent = true;

		const float radTol = mThickness + mTolerance;

		#pragma omp parallel for
		for (int i = 0; i < mParticles.size(); i++)
		{
			if (mParticles[i].invMass == 0)
				continue;

			Vector3 p, n;
			float dist = sdf.QueryPoint(mParticles[i].prev, p, n);

			// check if the distance to the sphere surface is less than our threshold
			if (dist < radTol && (dist >= 0 || handleInside))
			#pragma omp critical
			{
				int idx = AddContact(i, p, n);
				if (dist < mThickness) // if the distance is actually smaller than the cloth thickness
					mContacts[idx].depth = mThickness - dist; // then report a non-zero penetration depth
			}
			else if (handleCurrent)
			{
				Vector3 p1, n1;
				float dist1 = sdf.QueryPoint(mParticles[i].pos, p1, n1);

				if (dist1 < radTol && (dist1 >= 0 || handleInside))
				#pragma omp critical
				{
					int idx = AddContact(i, p1, n1);
					if (dist1 < mThickness) // if the distance is actually smaller than the cloth thickness
						mContacts[idx].depth = mThickness - dist1; // then report a non-zero penetration depth
				}
			}
		}

		//Printf("penetrated: %d / %d, inside: %d / %d, intersect: %d\n", countPrev, countPos, countPrevIn, countPosIn, intersections);

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

				const Vector3& v1 = p1.pos;
				const Vector3& v2 = p2.pos;
				const Vector3& v3 = p3.pos;

				Vector3 closestPt, normal, coords;
				float dist = sdf.QueryTriangle(v1, v2, v3, closestPt, normal, coords);

				if (dist < radTol && (dist >= 0 || handleInside))
				#pragma omp critical
				{
					int idx = AddTriContact(i1, i2, i3, closestPt, normal, coords);
					if (dist < mThickness) // if the distance is actually smaller than the cloth thickness
						mTriContacts[idx].depth = mThickness - dist; // then report a non-zero penetration depth
				}
			}
		}
	}

}