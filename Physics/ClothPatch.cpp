#include "ClothPatch.h"
#include "ClothPBD.h"
#include <Engine/Engine.h>

using namespace Geometry;
using namespace Math;

namespace Physics
{
	ClothPatch::ClothPatch(const CollisionWorld& world) : 
		mIsQuadMesh(true),
		mNumSteps(1), 
		mMesh(new Mesh()), 
		mCollWorld(world),
		mCollisionHandler(world)
	{ 
		mModel = new ClothModelPBD(this);
		mCollisionHandler.SetClothModel(mModel);
	}

	ClothPatch& ClothPatch::operator =(const ClothPatch& other)
	{
		mIsQuadMesh = other.mIsQuadMesh;
		mMesh = other.mMesh;
		mPosition = other.mPosition;
		mMethod = other.mMethod;
		mNumSteps = other.mNumSteps;
		if (mModel)
			delete mModel;

		if (mMethod == METHOD_PBD)
			mModel = new ClothModelPBD(this);
		else
			ASSERT(false);

		*mModel	= *other.mModel;
		mCollisionHandler.SetClothModel(mModel);
		return *this;
	}

	ClothPatch::~ClothPatch() { delete mModel; }

	void ClothPatch::Init(const Mesh& mesh, const Vector3& offset, bool attached)
	{
		if (mesh.vertices.size() == 0 || mesh.indices.size() == 0)
			return;

		mIsQuadMesh = false;
		*mMesh = mesh;

		// TODO: abolish this! and remove the map
		// remove duplicate vertices
		mModel->Clear();
		std::vector<int>& map = mModel->mMap;
		map.resize(mesh.vertices.size());
		std::fill(map.begin(), map.end(), -1);
		for (size_t i = 0; i < mesh.vertices.size(); i++)
		{
			if (map[i] >= 0)
				continue;
			for (size_t j = i + 1; j < mesh.vertices.size(); j++)
			{
				if ((mesh.vertices[i] - mesh.vertices[j]).LengthSquared() == 0)
				{
					map[j] = (int)mModel->GetNumParticles();
				}
			}
			map[i] = (int)mModel->GetNumParticles();
			mModel->AddParticle(mesh.vertices[i] + offset, (attached && !mesh.colors.empty() && mesh.colors[i].X() != 0) ? 0.f : 1.f, 
				mesh.uvs.empty() ? Vector2(0) : mesh.uvs[i]);
		}

		// build unique edges array
		std::vector<Mesh::Edge> duplicates(mesh.indices.size() - 1);
		for (size_t i = 0; i < mesh.indices.size() - 1; i++)
		{
			// construct edge
			int i1 = map[mesh.indices[i]];
			int i2 = map[mesh.indices[i + 1]];
			if ((i + 1) % 3 == 0)
			{
				i2 = map[mesh.indices[i - 2]];
				if (!mesh.uvs.empty())
				{
					mModel->AddTriangle(i2, map[mesh.indices[i - 1]], i1,
						mesh.uvs[mesh.indices[i - 2]], mesh.uvs[mesh.indices[i - 1]], mesh.uvs[mesh.indices[i]]);
				}
				else
				{
					mModel->AddTriangle(i2, map[mesh.indices[i - 1]], i1);
				}
			}
			if (i1 > i2)
				std::swap(i1, i2);
			Mesh::Edge edge(i1, i2);
			edge.t1 = i / 3;
			duplicates[i] = edge;
			//ASSERT(i1 != i2); // FIXME
		}
		size_t n = mesh.indices.size();
		if (!mesh.uvs.empty())
		{
			mModel->AddTriangle(map[mesh.indices[n - 3]], map[mesh.indices[n - 2]], map[mesh.indices[n - 1]],
				mesh.uvs[mesh.indices[n - 3]], mesh.uvs[mesh.indices[n - 2]], mesh.uvs[mesh.indices[n - 1]]);
		}
		else
		{
			mModel->AddTriangle(map[mesh.indices[n - 3]], map[mesh.indices[n - 2]], map[mesh.indices[n - 1]]);
		}
		// sort it so we can find duplicates
		std::sort(duplicates.begin(), duplicates.end(), Geometry::CompareEdges);

		mModel->AddLink(duplicates[0].i1, duplicates[0].i2, mModel->GetStretchStiffness());
		Mesh::Edge currEdge = duplicates[0];
		for (size_t i = 1; i < duplicates.size(); i++)
		{
			if (duplicates[i].i1 == duplicates[i - 1].i1 && duplicates[i].i2 == duplicates[i - 1].i2)
			{
				currEdge.t2 = duplicates[i].t1;
				continue;
			}
			mModel->AddLink(duplicates[i].i1, duplicates[i].i2, mModel->GetStretchStiffness());
			mMesh->edges.push_back(currEdge);
			mModel->AddEdge(currEdge);
			mModel->AddBendConstraint(currEdge, false);
			currEdge = duplicates[i];
		}
		mMesh->edges.push_back(currEdge);
		mModel->AddEdge(currEdge);
		mModel->AddBendConstraint(currEdge, false);

		mModel->Init();
	}

	void ClothPatch::Init(int divX, int divY, float inc, Vector3 offset, bool horizontal, bool attached)
	{
		mMesh->Clear();

		mIsQuadMesh = true;
		const int numParticles = divX * divY;
		mModel->Clear();
		mModel->SetNumParticles(numParticles);

		float scale = 1.f;

		float x = -0.5f * divX * inc;
		const float startY = x;
		const float incUVX = 1.f / divX; 
		const float incUVY = 1.f / divY;
		const float stretch = mModel->GetStretchStiffness();
		const float shear = mModel->GetShearStiffness();
		const float bend = mModel->GetBendStiffness();
		const float incline = GetRandomReal01();
		Printf("incline: %.20f\n", incline);
		mMesh->vertices.resize(numParticles);
		mMesh->uvs.resize(numParticles);
		for (int j = 0, base = 0; j < divX; j++, base += divY)
		{
			if (horizontal)
				mModel->GetParticle(base).pos = (Vector3(x, 0, startY) + offset) * scale;
			else
				mModel->GetParticle(base).pos = (Vector3(x, startY, 0) + offset) * scale;
			mModel->GetParticle(base).prev = mModel->GetParticle(base).pos;
			mModel->GetParticle(base).vel.SetZero();
			mModel->GetParticle(base).invMass = 1;
			Vector2 uv(j * incUVX, 0);
			mModel->GetParticle(base).uv.Set(x, 0);
			mMesh->vertices[base] = mModel->GetParticle(base).pos;
			mMesh->uvs[base] = uv;
			if (base >= divY)
			{
				mModel->AddLink(base - divY, base, stretch);
				mModel->AddLink(base - divY + 1, base, shear);
				if (base >= 2 * divY && !mModel->GetDihedral())
					mModel->AddLink(base - 2 * divY, base, bend);
			}
			for (int i = 1; i < divY; i++) 
			{
				int idx = base + i;
				if (horizontal)
					mModel->GetParticle(idx).pos = (Vector3(x, 0, startY + inc * i) + offset) * scale;
				else
					mModel->GetParticle(idx).pos = (Vector3(x, startY + inc * i, i * incline) + offset) * scale;
				mModel->GetParticle(idx).prev = mModel->GetParticle(idx).pos;
				mModel->GetParticle(idx).vel.SetZero();
				mModel->GetParticle(idx).invMass = 1;
				Vector2 uv(j * incUVX, i * incUVY);
				mModel->GetParticle(idx).uv.Set(x, inc * i);
				mMesh->vertices[idx] = mModel->GetParticle(idx).pos;
				mMesh->uvs[idx] = uv;
				if (idx > base)
				{
					mModel->AddLink(idx - 1, idx, stretch);
					if (idx > base + 1 && !mModel->GetDihedral())
						mModel->AddLink(idx - 2, idx, bend);
					if (idx >= divY)
					{
						mModel->AddLink(idx - divY, idx, stretch);
						if (i < divY - 1)
							mModel->AddLink(idx - divY + 1, idx, shear);
						if (j > 1 && !mModel->GetDihedral())
							mModel->AddLink(idx - 2 * divY, idx, bend);
					}
					// shear links
					if (idx > divY)
					{
						mModel->AddLink(idx - divY - 1, idx, shear);
						mModel->AddQuad(idx - divY - 1, idx - divY, idx, idx - 1);
						if ((i & 1) == (j & 1))
						{
							mModel->AddTriangle(idx - divY - 1, idx, idx - divY);
							mModel->AddTriangle(idx - divY - 1, idx - 1, idx);
							mMesh->AddTriangle(idx - divY - 1, idx, idx - divY);
							mMesh->AddTriangle(idx - divY - 1, idx - 1, idx);
						} else 
						{
							mModel->AddTriangle(idx - divY, idx - 1, idx);
							mModel->AddTriangle(idx - 1, idx - divY, idx - divY - 1);
							mMesh->AddTriangle(idx - divY, idx - 1, idx);
							mMesh->AddTriangle(idx - 1, idx - divY, idx - divY - 1);
						}
					}
				}
			}
			if (attached && (j == 0 || j == divX - 1))
			{
				mModel->GetParticle(base + divY - 1).invMass = 0.f;
			}
			x += inc;
		}

		mMesh->ComputeNormals();
		mMesh->ConstructEdges(); // construct (representative) triangles too
		mMesh->ConstructVertexOneRings();
		mMesh->ConstructEdgeOneRings();
		// TODO: give up on these edges, use the mesh directly
		for (int i = 0; i < mMesh->edges.size(); i++)
		{
			mModel->AddEdge(mMesh->edges[i]);
			mModel->AddBendConstraint(mMesh->edges[i], false);
		}
		mModel->Init();

		mPrevMesh.reset(new Mesh());
		*mPrevMesh = *mMesh;
	}

	static inline Vector3 CatmullRom(float s, const Vector3 p[])
	{
		Vector3 a = -0.5f * p[0] + 1.5f * p[1] - 1.5f * p[2] + 0.5f * p[3];
		Vector3 b =         p[0] - 2.5f * p[1] +  2.f * p[2] - 0.5f * p[3];
		Vector3 c = -0.5f * p[0] +               0.5f * p[2];
		Vector3 d = p[1];
		float s2 = s * s;
		float s3 = s2 * s;
		return a * s3 + b * s2 + c * s + d;
	}

	inline int IX(int x, int y, int n)
	{
		x = clamp(x, 0, n - 1);
		y = clamp(y, 0, n - 1);
		return x + y * n;
	}

	bool ClothPatch::UpdateMesh()
	{
		mModel->UpdateMesh(*mMesh, mIsQuadMesh, mPosition);
		mMesh->ComputeNormals();
		if (mPrevMesh)
			mPrevMesh->ComputeNormals();
		return mModel->CheckSignal();
	}

	void ClothPatch::ComputeMass()
	{
		const float density = 0.01f;
		std::vector<float> masses(mModel->GetNumParticles(), 0);
		for (size_t i = 0; i < mMesh->indices.size(); i += 3)
		{
			int i1 = mMesh->indices[i];
			int i2 = mMesh->indices[i + 1];
			int i3 = mMesh->indices[i + 2];
			const Vector3& v1 = mMesh->vertices[i1];
			const Vector3& v2 = mMesh->vertices[i2];
			const Vector3& v3 = mMesh->vertices[i3];
			Vector3 e1 = v2 - v1;
			Vector3 e2 = v3 - v2;
			Vector3 e3 = v1 - v3;
			float area = density * 0.5f * cross(e1, e3).Length();
			e1.Normalize();
			e2.Normalize();
			e3.Normalize();
			float t1 = acos(-(e1 * e3));
			float t2 = acos(-(e1 * e2));
			float t3 = acos(-(e2 * e3));
			masses[mModel->mMap[i1]] += t1 * area / PI;
			masses[mModel->mMap[i2]] += t2 * area / PI;
			masses[mModel->mMap[i3]] += t3 * area / PI;
		}
		mMaxMass = -1;
		for (size_t i = 0; i < mModel->GetNumParticles(); i++)
		{
			if (mModel->GetParticle(i).invMass != 0)
				mModel->GetParticle(i).invMass = 1.f / masses[i];
			if (masses[i] > mMaxMass)
				mMaxMass = masses[i];
		}
	}

	void ClothPatch::SetMethod(MethodType val)
	{
		mMethod = val;
		auto oldModel = mModel;
		if (mMethod == METHOD_PBD)
			mModel = new ClothModelPBD(*mModel);
		else
			ASSERT(false);
		mCollisionHandler.SetClothModel(mModel);
		delete oldModel;
	}

	bool ClothPatch::Step(float dt)
	{
		const int n = mNumSteps;
		const float h = dt / n;
		for (int i = 0; i < n; i++)
		{
			mModel->Step(h);
			if (mModel->CheckSignal())
				return true;
		}

		return false;
	}

	void ClothPatch::ComputeStrainMap(std::vector<Vector3>& colors)
	{
		for (size_t i = 0; i < GetModel().GetNumParticles(); i++)
			colors[i].SetZero();
		float maxStrain = 0;
		float minStrain = 20; // FIXME
		for (size_t i = 0; i < GetModel().GetNumTris(); i++)
		{
			Physics::Triangle tri = GetModel().GetTriangle(i);
			const Physics::Particle& p1 = GetModel().GetParticle(tri.i1);
			const Physics::Particle& p2 = GetModel().GetParticle(tri.i2);
			const Physics::Particle& p3 = GetModel().GetParticle(tri.i3);
			Vector3 dx1 = (p2.pos - p1.pos);
			Vector3 dx2 = (p3.pos - p1.pos);

			Vector3 wu = tri.invDet * (tri.dv2 * dx1 - tri.dv1 * dx2);
			Vector3 wv = tri.invDet * (-tri.du2 * dx1 + tri.du1 * dx2);

			tri.euu = 0.5f * ((wu * wu) - 1);
			tri.evv = 0.5f * ((wv * wv) - 1);
			tri.euv = wu * wv;

			float strain = sqrtf(tri.euu * tri.euu + tri.evv * tri.evv + tri.euv * tri.euv);
			maxStrain = max(strain, maxStrain);
			minStrain = min(strain, minStrain);

			float s = strain;

			Vector3 e1 = dx1;
			Vector3 e2 = p3.pos - p2.pos;
			Vector3 e3 = -dx2;
			e1.Normalize();
			e2.Normalize();
			e3.Normalize();
			float t1 = acos(-(e1 * e3));
			float t2 = acos(-(e1 * e2));
			float t3 = acos(-(e2 * e3));

			s *= 10.f;
			Vector3 col(s, 0, s);
			colors[tri.i1] += (t1 / PI) * col;
			colors[tri.i2] += (t2 / PI) * col;
			colors[tri.i3] += (t3 / PI) * col;
		}
	}
}