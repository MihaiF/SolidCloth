#include "ClothModel.h"
#include "ClothPatch.h"
#include <Engine/Engine.h>

using namespace Geometry;

namespace Physics
{
	ClothModel::ClothModel(ClothPatch* owner) 
		: mOwnerPatch(owner)
		, mThickness(0.3f)
		, mUnit(10)
		, mTolerance(0.2f)
		, mShearStiff(5e8f)
		, mBendStiff(5.f)
		, mStretchStiff(2e10f)
		, mPoisson(0.5f)
		, mFriction(0.4f)
		, mAlpha(0.55f)
		, mArea(10)
		, mNumIterations(10)
		, mSolver(SOLVER_GAUSS_SEIDEL)
		, mCollFlags(0)
		, mDihedral(true)
		, mFEM(false)
		, mUseCL(false)
		, mTree(nullptr)
	{
		mMouseSpring.active = false;
	}

	void ClothModel::SetPositions(const Vector3* positions)
	{
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			mParticles[i].pos = positions[i];
		}
	}

	void ClothModel::SetMasses(const float* masses)
	{
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			mParticles[i].invMass = masses[i] == 0.f ? 0.f : 1.f / masses[i];
		}
	}

	void ClothModel::ClearCollidables()
	{
		mCollidables.clear();
	}
	
	void ClothModel::AddTriangle(size_t i1, size_t i2, size_t i3, Vector2 uv1, Vector2 uv2, Vector2 uv3)
	{
		Triangle tri;
		tri.i1 = (uint32)i1;
		tri.i2 = (uint32)i2;
		tri.i3 = (uint32)i3;
		Vector2 d1 = uv2 - uv1;
		Vector2 d2 = uv3 - uv1;
		tri.du1 = d1.GetX();
		tri.du2 = d2.GetX();
		tri.dv1 = d1.GetY();
		tri.dv2 = d2.GetY();
		tri.det = (tri.du1 * tri.dv2 - tri.du2 * tri.dv1);
		tri.invDet = 1.f / tri.det;

		Vector3 dx1 = mParticles[i2].pos - mParticles[i1].pos;
		Vector3 dx2 = mParticles[i3].pos - mParticles[i1].pos;
		Vector3 wu = tri.invDet * (tri.dv2 * dx1 - tri.dv1 * dx2);
		Vector3 wv = tri.invDet * (-tri.du2 * dx1 + tri.du1 * dx2);
		tri.lu0 = wu.Length();
		tri.lv0 = wv.Length();
		tri.su = 1.f / tri.lu0;
		tri.sv = 1.f / tri.lv0;
		tri.dot = (wu * wv);

		tri.area = 0.5f * tri.det;
		tri.lu2 = tri.lu0 * tri.lu0;
		tri.lv2 = tri.lv0 * tri.lv0;

		tri.du12 = tri.du1 - tri.du2;
		tri.dv12 = tri.dv1 - tri.dv2;

		tri.collided = false;

		mTriangles.push_back(tri);
	}

	void ClothModel::AddBendConstraint(const Mesh::Edge& e, bool useLink)
	{
		if (e.t2 < 0)
			return;
		Vector3 n1 = cross(mParticles[mTriangles[e.t1].i2].pos - mParticles[mTriangles[e.t1].i1].pos, 
			mParticles[mTriangles[e.t1].i3].pos - mParticles[mTriangles[e.t1].i1].pos);
		float l1 = n1.Length();
		n1.Normalize();
		Vector3 n2 = cross(mParticles[mTriangles[e.t2].i2].pos - mParticles[mTriangles[e.t2].i1].pos, 
			mParticles[mTriangles[e.t2].i3].pos - mParticles[mTriangles[e.t2].i1].pos);
		float l2 = n2.Length();
		n2.Normalize();

		if (!useLink)
		{
			BendConstraint bc;
			bc.i1 = e.i1;
			bc.i2 = e.i2;
			bc.i3 = (uint32)mTriangles[e.t1].GetOppositeVertex(e.i1, e.i2);
			bc.i4 = (uint32)mTriangles[e.t2].GetOppositeVertex(e.i1, e.i2);
			bc.theta0 = acosf(n1 * n2);
			bc.l1 = l1;
			bc.l2 = l2;
			mBends.push_back(bc);
		}
		else
		{
			int i3 = (uint32)mTriangles[e.t1].GetOppositeVertex(e.i1, e.i2);
			int i4 = (uint32)mTriangles[e.t2].GetOppositeVertex(e.i1, e.i2);
			AddLink(i3, i4, mBendStiff);
		}
	}

	void ClothModel::PrintInfo()
	{
		// compute and print constraint error
		float err = 0;
		float vel = 0;
		float elastic = 0;
		for (size_t i = 0; i < mLinks.size(); i++)
		{
			Link& pair = mLinks[i];
			Vector3 n = mParticles[pair.i1].pos - mParticles[pair.i2].pos;
			float len = n.Length();
			float dl = len - pair.len;
			err += dl;
			elastic += 0.5f * pair.stiffness * dl * dl;
		}

		// compute and print kinetic energy
		float kin = 0;
		float pot = 0;
		Vector3 lin; // linear momentum
		Vector3 ang; // angular momentum
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			if (mParticles[i].invMass != 0)
			{
				kin += 0.5f * mParticles[i].vel.LengthSquared() / mParticles[i].invMass;
				pot += -(gravity * mUnit * mParticles[i].pos) / mParticles[i].invMass;
				Vector3 p = mParticles[i].vel * (1.f / mParticles[i].invMass);
				lin += p;
				ang += cross(mParticles[i].pos, p);
			}
		}
		float total = kin + pot + elastic;
		//Printf("%f, %f, %f, %f\n", err, kin, pot, elastic);
		//Printf("%f,%f\n", err, total);
		Printf("%f\n", ang.Length());

		mFrames++;
		if (mFrames > 2000)
			Engine::getInstance()->Quit();
	}

	void ClothModel::HandleMouseSpring(float h)
	{
		if (mMouseSpring.active)
		{
			Vector3 delta = mParticles[mMouseSpring.i1].pos - mMouseSpring.point;
			float len = delta.Length();
			delta.Scale(1 / len);
			Vector3 disp = mMouseSpring.stiffness * (len - mMouseSpring.len) * delta;
			mParticles[mMouseSpring.i1].pos -= h * disp;
			mParticles[mMouseSpring.i1].vel -= disp;
		}
	}

	void ClothModel::AddCollidable(std::shared_ptr<Collidable>& coll)
	{
		mCollidables.push_back(coll);
		//if (coll->mType == CT_MESH)
		//{
		//	collCL.PrepareBuffers(mParticles, *((CollisionMesh*)coll.get())->mesh);
		//}
	}

	void ClothModel::ResetConstraints()
	{
		for (size_t i = 0; i < mLinks.size(); i++)
		{
			mLinks[i].lambda = 0;
			mLinks[i].disp.SetZero();
		}
		for (size_t i = 0; i < mContacts.size(); i++)
		{
			mContacts[i].lambda = 0;
			mContacts[i].lambdaF = 0;
		}
		for (size_t i = 0; i < mTriContacts.size(); i++)
		{
			mTriContacts[i].lambda = 0;
			mTriContacts[i].lambdaF = 0;
		}
	}
}