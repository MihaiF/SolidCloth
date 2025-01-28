#include "ClothPBD.h"
#include "ClothPatch.h"
#include <Engine/Profiler.h>

using namespace Math;

namespace Physics
{
	float ClothModelPBD::SolveLinks(float h, float omega)
	{
		const float h2 = h * h;
		const float invH = 1.f / h;
		const float k = h2 * mArea * mUnit;
		float error = 0;
		for (size_t i = 0; i < mLinks.size(); i++)
		{
			Link& link = mLinks[i];
			Particle& p1 = mParticles[link.i1];
			Particle& p2 = mParticles[link.i2];

			if (p1.invMass + p2.invMass == 0)
				continue;

			float mu = 1.f / (p1.invMass + p2.invMass);

			Vector3 delta = p1.pos - p2.pos;
			const float len0 = link.len;
			float len = delta.Length();
			if (len == 0 || std::isnan(len) || std::isinf(len))
				continue;

			float err = len - len0;
			delta.Scale(1 / len);

			float epsilon = len0 / (k * link.stiffness);
			float s = 1.f / (1.f + mu * epsilon); 
			float lambda = err * mu * s;
			link.lambda += lambda;

			link.error = s * err;
			if (fabs(link.error) > error)
				error = fabs(link.error);

			Vector3 fc = omega * lambda * delta;
			p1.pos -= fc * p1.invMass;
			p2.pos += fc * p2.invMass;
		}

		return error;
	}

	void ClothModelPBD::SolveTriangles(float h)
	{
		//PROFILE_SCOPE("FEM");
		const float h2 = h * h;
		const float epsilonUV = 2 * (1 + mPoisson)  / (max(mShearStiff, 0.1f) * mUnit * mUnit); // 1 / shear modulus
		for (size_t k = 0; k < mTriangles.size(); k++)
		{
			Triangle& tri = mTriangles[k];
			Particle& p1 = mParticles[tri.i1];
			Particle& p2 = mParticles[tri.i2];
			Particle& p3 = mParticles[tri.i3];
			Vector3 dx1 = p2.pos - p1.pos;
			Vector3 dx2 = p3.pos - p1.pos;

			Vector3 wu = tri.invDet * (tri.dv2 * dx1 - tri.dv1 * dx2);
			Vector3 wv = tri.invDet * (-tri.du2 * dx1 + tri.du1 * dx2);

			// TODO: relaxation factors for stretch
			float err, diag, lambda;
			Vector3 q1, q2, q3;

			// warp
			q2 = (tri.invDet * tri.dv2) * wu;
			q3 = (-tri.invDet * tri.dv1) * wu;
			q1 = -q2 - q3;
			diag = (p1.invMass * q1 * q1 + p2.invMass * q2 * q2 + p3.invMass * q3 * q3);
			err = 0.5f * (wu * wu - tri.lu2);
			lambda = - err / diag;
			p1.pos += p1.invMass * lambda * q1;
			p2.pos += p2.invMass * lambda * q2;
			p3.pos += p3.invMass * lambda * q3;

			// weft
			q2 = (-tri.invDet * tri.du2) * wv;
			q3 = (tri.invDet * tri.du1) * wv;
			q1 = -q2 - q3;
			diag = (p1.invMass * q1 * q1 + p2.invMass * q2 * q2 + p3.invMass * q3 * q3);
			err = 0.5f * (wv * wv - tri.lv2);
			lambda = - err / diag;
			p1.pos += p1.invMass * lambda * q1;
			p2.pos += p2.invMass * lambda * q2;
			p3.pos += p3.invMass * lambda * q3;

			// shearing
			q2 = tri.dv2 * wv - tri.du2 * wu;
			q3 = -tri.dv1 * wv + tri.du1 * wu;
			q1 = -q2 - q3;

			diag = tri.invDet * (p1.invMass * q1 * q1 + p2.invMass * q2 * q2 + p3.invMass * q3 * q3);
			err = wu * wv;
			float s = 1.f / (1.f + tri.lu0 * tri.lv0 * epsilonUV / (diag * h2 * tri.area));
			lambda = -s * err / diag;
			p1.pos += p1.invMass * lambda * q1;
			p2.pos += p2.invMass * lambda * q2;
			p3.pos += p3.invMass * lambda * q3;
		}
	}

	float EvaluateAngle(Vector3 x1, Vector3 x2, Vector3 x3, Vector3 x4, float& cosine)
	{
		Vector3 x12 = x2 - x1; // the edge
		float l = x12.Length();

		Vector3 x13 = x3 - x1;
		Vector3 n1 = cross(x12, x13);
		float l1 = n1.Length();
		Vector3 x14 = x4 - x1;
		Vector3 n2 = cross(x14, x12);
		float l2 = n2.Length();

		if (l1 == 0 || l2 == 0)
		{
			cosine = 1;
			return 0;
		}

		float il1 = 1.f / l1;
		float il2 = 1.f / l2;

		Vector3 n1n = il1 * n1;
		Vector3 n2n = il2 * n2;

		cosine = dot(n1n, n2n); // cosine of angle

		cosine = min(1.f, max(-1.f, cosine));

		return acosf(cosine);
	}

	float EvaluateGradient(Vector3 x1, Vector3 x2, Vector3 x3, Vector3 x4, float cosine, 
		Vector3& q1, Vector3& q2, Vector3& q3, Vector3& q4)
	{
		Vector3 x12 = x2 - x1; // the edge
		float l = x12.Length();

		Vector3 x13 = x3 - x1;
		Vector3 n1 = cross(x12, x13);
		float l1 = n1.Length();
		Vector3 x14 = x4 - x1;
		Vector3 n2 = cross(x14, x12);
		float l2 = n2.Length();

		float h1 = l1 / l;
		float h2 = l2 / l;
		float he = 0.5f * (h1 + h2);

		float il1 = 1.f / l1;
		float il2 = 1.f / l2;

		// TODO: get the from angle function
		Vector3 n1n = il1 * n1;
		Vector3 n2n = il2 * n2;

		// project on each other's plane
		Vector3 n1p = n1n - n2n * cosine;
		Vector3 n2p = n2n - n1n * cosine;

		float sine = sqrt(1 - cosine * cosine);

		q3 = (il1 / sine / he) * cross(x12, n2p);
		q4 = (-il2 / sine / he) * cross(x12, n1p);
		q2 = (-il1 / sine / he) * cross(x13, n2p) + (il2 / sine / he) * cross(x14, n1p);
		q1 = -q2 - q3 - q4;

		return he;
	}

	float ClothModelPBD::SolveBends(float h)
	{
		//PROFILE_SCOPE("SolveBendsPBD");
		float error = 0;
		for (size_t i = 0; i < mBends.size(); i++)
		{
			BendConstraint& bc = mBends[i];

			const Vector3& x1 = mParticles[bc.i1].pos;
			const Vector3& x2 = mParticles[bc.i2].pos;
			const Vector3& x3 = mParticles[bc.i3].pos;
			const Vector3& x4 = mParticles[bc.i4].pos;

			if (mParticles[bc.i1].invMass + mParticles[bc.i2].invMass + mParticles[bc.i3].invMass + mParticles[bc.i4].invMass == 0)
				continue;

			float cosine;
			float theta = EvaluateAngle(x1, x2, x3, x4, cosine);

 			if (fabs(cosine) >= 1 - 0.001f)
				continue;

			Vector3 q1, q2, q3, q4;
			float he = EvaluateGradient(x1, x2, x3, x4, cosine, q1, q2, q3, q4);
			//EvaluateGradientFD(x1, x2, x3, x4, theta, q1, q2, q3, q4);

			float diag = mParticles[bc.i1].invMass * q1.LengthSquared() +
				mParticles[bc.i2].invMass * q2.LengthSquared() +
				mParticles[bc.i3].invMass * q3.LengthSquared() +
				mParticles[bc.i4].invMass * q4.LengthSquared();
			if (diag == 0)
				continue;
			float s = 0.0005f * mBendStiff;
			
			bc.error = s * theta;
			if (fabs(bc.error) > error)
				error = fabs(bc.error);

			float lambda = -s * theta / he / diag;
			mParticles[bc.i1].pos += lambda * mParticles[bc.i1].invMass * q1;
			mParticles[bc.i2].pos += lambda * mParticles[bc.i2].invMass * q2;
			mParticles[bc.i3].pos += lambda * mParticles[bc.i3].invMass * q3;
			mParticles[bc.i4].pos += lambda * mParticles[bc.i4].invMass * q4;
		}
		return error;
	}

	inline void SolveFriction(Particle& p1, Contact& contact, float depth, float mu)
	{
		if (mu == 0)
			return;

		Vector3 v12 = p1.pos - p1.prev - contact.vel;
		float vnrel = contact.normal.Dot(v12);
		Vector3 vt = v12 - vnrel * contact.normal;
		float vtrel = vt.Length();

		float lambda = -(vnrel + depth);
		contact.lambda += lambda;
		const float limit = mu * contact.lambda;

		if (vtrel > 0.001f)
		{
			float dLambda = vtrel;
			float lambda0 = contact.lambdaF;
			contact.lambdaF = lambda0 + dLambda;
			if (contact.lambdaF >= limit)
				contact.lambdaF = limit;
			dLambda = contact.lambdaF - lambda0;

			vt.Scale(1.f / vtrel); // normalize
			Vector3 p = dLambda * vt;
			p1.pos -= p;
		}
	}

	void ClothModelPBD::Step(float h)
	{
		// Symplectic Euler
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			mParticles[i].prev = mParticles[i].pos;
			if (mParticles[i].invMass == 0)
				continue;
			mParticles[i].vel += h * gravity * mUnit;
			mParticles[i].vel *= mDragCoeff;
			mParticles[i].pos += h * mParticles[i].vel;
		}
		// mouse spring
		if (mMouseSpring.active)
		{
			Vector3 p = mMouseSpring.a1 * mParticles[mMouseSpring.i1].pos +
				mMouseSpring.a2 * mParticles[mMouseSpring.i2].pos +
				mMouseSpring.a3 * mParticles[mMouseSpring.i3].pos;
			Vector3 delta = p - mMouseSpring.point;
			float len = delta.Length();
			delta.Scale(1 / len);
			Vector3 disp = mMouseSpring.stiffness * (len - mMouseSpring.len) * delta;
			mParticles[mMouseSpring.i1].pos -= h * mMouseSpring.a1 * disp;
			mParticles[mMouseSpring.i1].vel -= mMouseSpring.a1 * disp;
			mParticles[mMouseSpring.i2].pos -= h * mMouseSpring.a2 * disp;
			mParticles[mMouseSpring.i2].vel -= mMouseSpring.a2 * disp;
			mParticles[mMouseSpring.i3].pos -= h * mMouseSpring.a3 * disp;
			mParticles[mMouseSpring.i3].vel -= mMouseSpring.a3 * disp;
		}

		// collision detection
		DetectCollisions();

		// reset constraints to zero
		ResetConstraints();

		if (!mUseCL)
		{
			if (mSolver == SOLVER_GAUSS_SEIDEL)
				SolveGS(h);
			else if (mSolver == SOLVER_JACOBI)
				SolveJacobi(h);
			else
				SolveCR(h, 0.1f);
		}
		else
		{
			if (mDihedral)
				SolveBends(h);
#ifdef ENABLE_CL
			// OpenCl
			PROFILE_SCOPE("OpenCL");
			projCL.CopyBuffers(mParticles, mContacts, mSelfTris);
			projCL.SetNumIterations(mNumIterations);
			projCL.Run(mThickness, mFriction);
			projCL.ReadBuffers(mParticles);
#endif
		}

		// update velocities
		const float invH = 1.f / h;
		for (size_t i = 0; i < mParticles.size(); i++)
		{
			if (mParticles[i].invMass == 0)
				continue;
			mParticles[i].vel = invH * (mParticles[i].pos - mParticles[i].prev);
		}
	}

	void ClothModelPBD::SolveGS(float h)
	{
		PROFILE_SCOPE("PBD GS");

		auto collHandler = mOwnerPatch->GetCollisionHandler();
		collHandler.Update();
		
		float cl, cb, cv, ce, ct, sct;
		cl = cb = cv = ce = ct = sct = 0;
		ContactStats sceStats;
		int k = 0;
		while (true)
		{
			if (!mFEM)
			{
				cl = SolveLinks(h, 1.f);
			}
			else
			{
				SolveTriangles(h);
			}
			if (mDihedral && k % mBendFreq == 0)
				cb = SolveBends(h);
			
			cv = collHandler.SolveContactsPosition(h);
			ce = collHandler.SolveEdgeContacts();
			ct = collHandler.SolveTriContacts();
			sct = collHandler.SolveSelfTrisPosition(h);
			sceStats = collHandler.SolveSelfEdgesPosition(h);

			k++;

			const float eps = 0.01f;
			if (cl < 0.05f && cb < 0.005f && sceStats.error < mSelfCollThreshold && sct < mSelfCollThreshold && cv < eps && ce < eps && ct < eps)
				break;

			if (k >= mNumIterations)
				break;			
		}
		mStats.iterations = k;
		mStats.linksError = cl;
		mStats.bendError = cb;
		mStats.selfEEError = sceStats.error;
		mStats.selfEECount = sceStats.count;
		mStats.selfVTError = sct;
	}

	inline float ComputeErrorAndNormal(Link& link, const std::vector<Particle>& particles, float unit)
	{
		Vector3 delta;
		delta = particles[link.i2].pos - particles[link.i1].pos;
		float c = (delta.Length() - link.len) * link.stiffness * unit;
		delta.Normalize();
		link.normal = delta;
		return c;
	}

#define SKIP_ALPHA
#define SKIP_BETA

	void ClothModelPBD::SolveCR(float h, float alpha0)
	{
		PROFILE_SCOPE("Nonlinear Conjugate Residual");

		std::vector<float> r, d, q;
		std::vector<Vector3> y;

		size_t nl = mLinks.size();
		if (nl == 0)
			return;
		if (r.size() != nl)
		{
			r.resize(nl);
			d.resize(nl);
			q.resize(nl);
		}

		float rSqrOld = 0;
		size_t np = mParticles.size();
		if (y.size() != np) y.resize(np);

		memset(&d[0], 0, nl * sizeof(float));
		for (int iter = 0; iter < mNumIterations; iter++)
		{
			// recompute error and normal
			for (size_t i = 0; i < nl; i++)
			{
				r[i] = -ComputeErrorAndNormal(mLinks[i], mParticles, mUnit);
			}

#ifndef SKIP_BETA
			// A-norm Fletcher-Reeves
			// y = J^T * r
			memset(&y[0], 0, np * sizeof(Vector3));
			for (size_t i = 0; i < nl; i++)
			{
				Vector3 add = mLinks[i].normal * r[i];
				y[mLinks[i].i1] += add;
				y[mLinks[i].i2] -= add;
			}
			// r^2 = r A r
			float rSqrNew = 0;
			for (size_t i = 0; i < np; i++)
			{
				rSqrNew += y[i].LengthSquared() * mParticles[i].invMass;
			}

			if (iter > 0)
			{
				// compute new search direction
				float beta = rSqrNew / rSqrOld; // FR
				beta = min(1.f, beta);
				for (size_t i = 0; i < nl; i++)
				{
					d[i] = r[i] + beta * d[i];
				}
			}
			else
			{
				d = r;
			}
			rSqrOld = rSqrNew;
#else
			float beta = (float)iter / (float)(mNumIterations - 1);
			beta = pow(beta, 0.6f);
			for (size_t i = 0; i < nl; i++)
			{
				d[i] = r[i] + beta * d[i];
			}
#endif

#ifndef SKIP_ALPHA
			float den = 0;
			float dot = 0;
			// y = J^T * d
			memset(&y[0], 0, np * sizeof(Vector3));
			for (size_t i = 0; i < nl; i++)
			{
				Vector3 add = mLinks[i].normal * d[i];
				y[mLinks[i].i1] -= add;
				y[mLinks[i].i2] += add;
			}
			// q = J * W  * y = A * d
			// den = q^T * q
			// dot = r^T * q
			for (size_t i = 0; i < nl; i++)
			{
				const int i1 = mLinks[i].i1;
				const int i2 = mLinks[i].i2;
				q[i] = mLinks[i].normal.Dot(y[i2] * mParticles[i2].invMass - y[i1] * mParticles[i1].invMass);

				den += q[i] * q[i];
				dot += r[i] * q[i];
			}

			if (den == 0)
				break;
			float alpha = min(dot / den, alpha0);
#else
			float alpha = alpha0;
#endif
			// apply dlambda = alpha * d
			for (size_t i = 0; i < nl; i++)
			{
				const int i1 = mLinks[i].i1;
				const int i2 = mLinks[i].i2;
				float dLambda = alpha * d[i];
				Vector3 disp = dLambda * mLinks[i].normal;
				mParticles[i1].pos -= mParticles[i1].invMass * disp;
				mParticles[i2].pos += mParticles[i2].invMass * disp;
			}
		}
	}

	void ClothModelPBD::SolveTrianglesEnergy(float h)
	{
		const float h2 = h * h;
		const float nu = mPoisson; // Poisson ratio
		const float young = mStretchStiff * mUnit * mUnit; // Young's modulus
		float kst = young / (1 - nu * nu);
		float ksh = 0.5f * mShearStiff * mUnit * mUnit  / (1 + nu);
		for (size_t k = 0; k < mTriangles.size(); k++)
		{
			Triangle& tri = mTriangles[k];
			Particle& p1 = mParticles[tri.i1];
			Particle& p2 = mParticles[tri.i2];
			Particle& p3 = mParticles[tri.i3];
			Vector3 dx1 = p2.pos - p1.pos;
			Vector3 dx2 = p3.pos - p1.pos;

			Vector3 wu = tri.invDet * (tri.dv2 * dx1 - tri.dv1 * dx2);
			Vector3 wv = tri.invDet * (-tri.du2 * dx1 + tri.du1 * dx2);

			float euu = 0.5f * (wu * wu - 1.f);
			float evv = 0.5f * (wv * wv - 1.f);
			float euv = wu * wv;

			float err, diag, lambda;
			Vector3 q1, q2, q3;
			Vector3 f1, f2, f3;
			float a = 1.f;
			
			// warp
			q2 = (tri.dv2) * wu;
			q3 = (-tri.dv1) * wu;
			q1 = -q2 - q3;
			float cuu = a * kst * (euu + nu * evv);
			f1 += cuu * q1;
			f2 += cuu * q2;
			f3 += cuu * q3;

			// weft
			q2 = (-tri.du2) * wv;
			q3 = (tri.du1) * wv;
			q1 = -q2 - q3;
			float cvv = a * kst * (evv + nu * euu);
			f1 += cvv * q1;
			f2 += cvv * q2;
			f3 += cvv * q3;

			// shearing
			q2 = tri.dv2 * wv - tri.du2 * wu;
			q3 = -tri.dv1 * wv + tri.du1 * wu;
			q1 = -q2 - q3;
			float cuv = a * ksh * euv;
			f1 += cuv * q1;
			f2 += cuv * q2;
			f3 += cuv * q3;

			diag = p1.invMass * f1 * f1 + p2.invMass * f2 * f2 + p3.invMass * f3 * f3;
			err = 0.5f * tri.area * (cuu * euu + cvv * evv + cuv * euv);
			lambda = - err / diag;

			p1.pos += p1.invMass * lambda * f1;
			p2.pos += p2.invMass * lambda * f2;
			p3.pos += p3.invMass * lambda * f3;
		}
	}

	void ClothModelPBD::Init()
	{
#ifdef ENABLE_CL
		if (mUseCL)
		{
			const float h = 0.008f; // FIXME
			const float h2 = h * h;
			const float k = h2 * mArea * mUnit;
			for (size_t i = 0; i < mLinks.size(); i++)
			{
				Link& link = mLinks[i];
				Particle& p1 = mParticles[link.i1];
				Particle& p2 = mParticles[link.i2];

				float mu = 1.f / (p1.invMass + p2.invMass);
				float epsilon = link.len / (k * link.stiffness);
				link.relaxation = 1.f / (1.f + mu * epsilon); 
			}

			projCL.Init();
			projCL.PrepareBuffers(mParticles, mLinks);

			collCL.Init();
		}
#endif

		// build incidence data
		//mVertexIncidence.resize(mParticles.size());
		//for (size_t i = 0; i < mLinks.size(); i++)
		//{
		//	mVertexIncidence[mLinks[i].i1].push_back(i);
		//	mVertexIncidence[mLinks[i].i2].push_back(i);
		//}
		//mEdgeIncidence.resize(mLinks.size());
		//for (size_t i = 0; i < mLinks.size(); i++)
		//{
		//	int i1 = mLinks[i].i1;
		//	for (size_t j = 0; j < mVertexIncidence[i1].size(); j++)
		//	{
		//		size_t e = mVertexIncidence[i1][j];
		//		//if (e > i)
		//			mEdgeIncidence[i].push_back(e);
		//	}
		//	int i2 = mLinks[i].i2;
		//	for (size_t j = 0; j < mVertexIncidence[i2].size(); j++)
		//	{
		//		size_t e = mVertexIncidence[i2][j];
		//		//if (e > i)
		//			mEdgeIncidence[i].push_back(e);
		//	}
		//}
	}

	void ClothModelPBD::SolveLinksJacobi(float h, float omega)
	{
		const float h2 = h * h;
		const float k = h2 * mArea * mUnit;
		for (size_t i = 0; i < mLinks.size(); i++)
		{
			Link& link = mLinks[i];
			Particle& p1 = mParticles[link.i1];
			Particle& p2 = mParticles[link.i2];

			float mu = 1.f / (p1.invMass + p2.invMass);

			Vector3 delta = p1.pos - p2.pos;
			const float len0 = link.len;
			float len = delta.Length();
			delta.Scale(1 / len);
			float err = len - len0;

			float epsilon = len0 / (k * link.stiffness);
			float s = 1.f / (1.f + mu * epsilon); 
			float lambda = err * mu * s;
			//link.lambda += lambda;

			link.disp = omega * lambda * delta;
		}

		for (size_t i = 0; i < mLinks.size(); i++)
		{
			Link& link = mLinks[i];
			Particle& p1 = mParticles[link.i1];
			Particle& p2 = mParticles[link.i2];
			
			p1.pos -= link.disp * p1.invMass;
			p2.pos += link.disp * p2.invMass;
		}
	}

	void ClothModelPBD::SolveJacobi(float h)
	{
		PROFILE_SCOPE("Jacobi PBD");
		for (int k = 0; k < mNumIterations; ++k)
		{
			// TODO: incidence degree
			SolveLinksJacobi(h, 0.5f);
		}
	}
}