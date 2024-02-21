#include "ClothCollisionHandler.h"
#include "Engine/Profiler.h"
#include "ClothModel.h"

#define UPDATE_CLOSEST_POINT

namespace Physics
{
	ClothCollisionHandler::ClothCollisionHandler(const CollisionWorld& world) : mModel(nullptr), mCollWorld(world)
	{
	}

	void ClothCollisionHandler::SetClothModel(ClothModel* model)
	{
		mModel = model;
	}

	void ClothCollisionHandler::Update()
	{
		mVertexLambda.resize(mModel->GetNumParticles());
		mVertexLambdaF.resize(mModel->GetNumParticles());
		std::fill(mVertexLambda.begin(), mVertexLambda.end(), 0.f);
		std::fill(mVertexLambdaF.begin(), mVertexLambdaF.end(), 0.f);

		mEdgeLambda.resize(mModel->GetEdges().size());
		mEdgeLambdaF.resize(mModel->GetEdges().size());
		std::fill(mEdgeLambda.begin(), mEdgeLambda.end(), 0.f);
		std::fill(mEdgeLambdaF.begin(), mEdgeLambdaF.end(), 0.f);

		mTriLambda.resize(mModel->GetNumTris());
		mTriLambdaF.resize(mModel->GetNumTris());
		std::fill(mTriLambda.begin(), mTriLambda.end(), 0.f);
		std::fill(mTriLambdaF.begin(), mTriLambdaF.end(), 0.f);
	}

	bool ClothCollisionHandler::HandleContactsVelocity(float h)
	{
		if (numVelocityIterations == 0)
			return false;

		bool allCollisionsSolved = false;
		int iter;
		for (iter = 0; iter < numVelocityIterations; iter++)
		{
			// static/unary contacts
			SolveContactsVelocity(h);
		}

		// integrate the positions
		for (size_t i = 0; i < mModel->GetNumParticles(); i++)
		{
			auto& particle = mModel->GetParticle(i);
			if (particle.invMass == 0)
				continue;
			particle.pos = particle.prev + h * particle.vel;
		}
		return allCollisionsSolved;
	}

	void ClothCollisionHandler::HandleContactsPosition(float h)
	{
		if (numPositionIterations == 0)
			return;

		for (int iter = 0; iter < numPositionIterations; iter++)
		{
			// static/unary contacts
			SolveContactsPosition(h);
		}

		// update velocities
		const float invH = 1.f / h;
		for (size_t i = 0; i < mModel->GetNumParticles(); i++)
		{
			Particle& p = mModel->GetParticle(i);
			if (p.invMass == 0)
				continue;
			p.vel = invH * (p.pos - p.prev);
		}
	}

	void ClothCollisionHandler::SolveContactsVelocity(float h)
	{
		const float invH = 1.0f / h;
		float mu = mModel->GetFriction();
		for (size_t i = 0; i < mModel->GetNumContacts(); i++)
		{
			Contact& contact = mModel->GetContact(i);
			Particle& p1 = mModel->GetParticle(contact.idx);
			Vector3 delta = p1.pos - contact.point;
#ifdef UPDATE_DEPTH
			float len = delta.Dot(contact.normal);
			if (len < mModel->GetThickness())
				contact.depth = mModel->GetThickness() - len;
#endif
			Vector3 vrel = p1.vel; // TODO: contact velocity
			delta = contact.normal;
			float vn = vrel.Dot(delta);
			float err = -vn + baumgarte * contact.depth * invH;

			float lambda0 = contact.lambda;
			contact.lambda += err;
			if (contact.lambda < 0)
				contact.lambda = 0;
			float dLambda = contact.lambda - lambda0;
			delta.Scale(dLambda);
			p1.vel += delta;

			// friction
			Vector3 vt = vrel - vn * contact.normal;
			float slip = vt.Length();
			if (fabs(slip) > 0.001f)
			{
				Vector3 tanDir = (1.f / slip) * vt;
				float limit = mu * contact.lambda;

				float lambdaF0 = contact.lambdaF;
				contact.lambdaF += slip;
				if (contact.lambdaF > limit)
					contact.lambdaF = limit;
				float dLambdaF = contact.lambdaF - lambdaF0;

				p1.vel -= dLambdaF * tanDir;
			}
		}
	}

	float ClothCollisionHandler::SolveContactsPosition(float h)
	{
		PROFILE_SCOPE("Solve Contacts PBD");
		const float invH = 1.f / h;
		const float mu = mModel->GetFriction();
		float thickness = mModel->GetThickness();
		float error = 0;
		for (size_t i = 0; i < mModel->GetNumContacts(); i++)
		{
			Contact& contact = mModel->GetContact(i);
			Particle& p1 = mModel->GetParticle(contact.idx);
#ifdef UPDATE_CLOSEST_POINT
			if (contact.mesh != nullptr)
			{
				// TODO: Voronoi region test
				Vector3 coords, cp;
				int tri = contact.feature;
				int i1 = contact.mesh->indices[tri * 3];
				int i2 = contact.mesh->indices[tri * 3 + 1];
				int i3 = contact.mesh->indices[tri * 3 + 2];
				Vector3 a = contact.mesh->vertices[i1];
				Vector3 b = contact.mesh->vertices[i2];
				Vector3 c = contact.mesh->vertices[i3];
				int region = Geometry::ClosestPtPointTriangle(p1.pos, a, b, c, cp, coords);
				contact.point = cp;
				contact.normal = Geometry::ComputeTriangleNormal(*contact.mesh, tri, coords, false);
			}
#endif
			Vector3 delta = p1.pos - contact.point;
			float len = delta.Dot(contact.normal);
			Vector3 disp = contact.normal;
			if (len > thickness)
				continue;
			
			float depth = thickness - len;
			if (fabs(depth) > error) // maximum error
				error = fabs(depth);
			float dLambda = depth;
			contact.lambda += dLambda;
			disp.Scale(dLambda);

			// friction
			Vector3 v12 = p1.pos - p1.prev - contact.vel;
			float vnrel = contact.normal.Dot(v12);
			Vector3 vt = v12 - vnrel * contact.normal;
			float vtrel = vt.Length();

			const float limit = mu * contact.lambda;

			if (vtrel > 0.001f)
			{
				float dLambdaF = vtrel;
				float lambdaF0 = contact.lambdaF;
				contact.lambdaF = lambdaF0 + dLambdaF;
				if (contact.lambdaF >= limit)
					contact.lambdaF = limit;
				dLambdaF = contact.lambdaF - lambdaF0;

				vt.Scale(1.0f / vtrel); // normalize
				disp += -dLambdaF * vt;
			}

			// apply resulting displacement
			p1.pos += disp;
			p1.vel += invH * disp; // for solvers that do not update velocity at the end
		}

		return error;
	}

	float ClothCollisionHandler::SolveTriContacts()
	{
		const float mu = mModel->GetFriction();
		float error = 0;
		for (size_t i = 0; i < mModel->GetNumTriContacts(); i++)
		{
			TriContact& contact = mModel->GetTriContact(i);
			Particle& p1 = mModel->GetParticle(contact.i1);
			Particle& p2 = mModel->GetParticle(contact.i2);
			Particle& p3 = mModel->GetParticle(contact.i3);
			
#ifndef UPDATE_CLOSEST_POINT
			Vector3 p = contact.w1 * p1.pos + contact.w2 * p2.pos + contact.w3 * p3.pos;
#else
			// TODO: check if still in Voronoi region
			Vector3 coords;
			Vector3 p;
			Geometry::ClosestPtPointTriangle(contact.point, p1.pos, p2.pos, p3.pos, p, coords);
			// TODO: check if any barycentric coordinate is close to zero (on the border)
			contact.w1 = coords.x;
			contact.w2 = coords.y;
			contact.w3 = coords.z;
			Vector3 n = -(p2.pos - p1.pos).Cross(p3.pos - p1.pos); // triangle normal (for now)
			n.Normalize();
			contact.normal = n; // the normal from closest points is a disaster (creates penetrations)
			if (contact.mesh != nullptr && contact.feature >= 0)
			{
				// TODO: figure out why I can't use the mesh normal directly
				Vector3 meshNormal = contact.mesh->normals[contact.feature];
				if (n.Dot(meshNormal) < 0)
					contact.normal = -n;
			}
#endif

			float len0 = mModel->GetThickness();
			float len = contact.normal.Dot(p - contact.point - contact.vel);
			if (len > len0)
				continue;

			float s = 1.f / (contact.w1 * contact.w1 / p1.invMass +
				contact.w2 * contact.w2 / p2.invMass + contact.w3 * contact.w3 / p3.invMass);
			float depth = len0 - len;
			if (depth > error)
				error = depth;
			float dLambda = s * depth;
			contact.lambda += dLambda;
			Vector3 disp = dLambda * contact.normal;

			if (mu != 0)
			{
				Vector3 p0 = contact.w1 * p1.prev +
					contact.w2 * p2.prev +
					contact.w3 * p3.prev;
				Vector3 v12 = p - p0 - contact.vel;
				float vnrel = contact.normal.Dot(v12);
				Vector3 vt = v12 - vnrel * contact.normal;
				float vtrel = vt.Length();

				const float limit = mu * contact.lambda;

				if (vtrel > 0.001f)
				{
					float dLambdaF = vtrel;
					float lambda0F = contact.lambdaF;
					contact.lambdaF = lambda0F + dLambdaF;
					if (contact.lambdaF >= limit)
						contact.lambdaF = limit;
					dLambda = contact.lambdaF - lambda0F;

					vt.Scale(1.f / vtrel); // normalize
					Vector3 p = -s * dLambda * vt;
					p1.pos += p * (contact.w1 * p1.invMass);
					p2.pos += p * (contact.w2 * p2.invMass);
					p3.pos += p * (contact.w3 * p3.invMass);
				}
			}

			p1.pos += disp * (contact.w1 * p1.invMass);
			p2.pos += disp * (contact.w2 * p2.invMass);
			p3.pos += disp * (contact.w3 * p3.invMass);
		}

		return error;
	}

	float ClothCollisionHandler::SolveEdgeContacts()
	{
		const float mu = mModel->GetFriction();
		float error = 0;
		for (size_t i = 0; i < mModel->GetNumEdgeContacts(); i++)
		{
			EdgeContact& contact = mModel->GetEdgeContact(i);
			Particle& p1 = mModel->GetParticle(contact.i1);
			Particle& p2 = mModel->GetParticle(contact.i2);

			Vector3 p = contact.w1 * p1.pos + contact.w2 * p2.pos;
#ifdef UPDATE_CLOSEST_POINT
			if (contact.mesh != nullptr)
			{
				// TODO: Voronoi region check
				float s1, t;
				int edge = contact.feature;
				int i1 = contact.mesh->edges[edge].i1;
				int i2 = contact.mesh->edges[edge].i2;
				Vector3 a = contact.mesh->vertices[i1];
				Vector3 b = contact.mesh->vertices[i2];
				Geometry::ClosestPtSegmSegm(a, b, p1.pos, p2.pos, s1, t, contact.point, p);
				contact.w1 = 1.0f - t;
				contact.w2 = t;
			}
#endif

			float len0 = mModel->GetThickness();
			float len = contact.normal.Dot(p - contact.point - contact.vel);
			if (len > len0)
				continue;

			ASSERT(contact.w1 + contact.w2 > 0);
			float s = 1.f / (contact.w1 * contact.w1 / p1.invMass +	contact.w2 * contact.w2 / p2.invMass);
			float depth = len0 - len;
			if (depth > error)
				error = depth;
			float dLambda = s * depth;
			contact.lambda += dLambda;
			Vector3 disp = dLambda * contact.normal;

			if (mu != 0)
			{
				Vector3 p0 = contact.w1 * p1.prev +
					contact.w2 * p2.prev;
				Vector3 v12 = p - p0 - contact.vel;
				float vnrel = contact.normal.Dot(v12);
				Vector3 vt = v12 - vnrel * contact.normal;
				float vtrel = vt.Length();

				const float limit = mu * contact.lambda;

				if (vtrel > 0.001f)
				{
					float dLambdaF = vtrel;
					float lambda0F = contact.lambdaF;
					contact.lambdaF = lambda0F + dLambdaF;
					if (contact.lambdaF >= limit)
						contact.lambdaF = limit;
					dLambda = contact.lambdaF - lambda0F;

					vt.Scale(1.f / vtrel); // normalize
					Vector3 p = -s * dLambda * vt;
					p1.pos += p * (contact.w1 * p1.invMass);
					p2.pos += p * (contact.w2 * p2.invMass);
				}
			}

			p1.pos += disp * (contact.w1 * p1.invMass);
			p2.pos += disp * (contact.w2 * p2.invMass);
		}

		return error;
	}

	float ClothCollisionHandler::SolveSelfTrisPosition(float h)
	{
		const float shrink = .9f;
		const float soften = .9f;
		float len0 = mModel->GetThickness() * shrink;
		float mu = mModel->GetFriction();
		float error = 0;
		for (size_t i = 0; i < mModel->GetNumSelfTris(); i++)
		{
			SelfContact& contact = mModel->GetSelfTriangle(i);
			Particle& p1 = mModel->GetParticle(contact.i1);
			Particle& p2 = mModel->GetParticle(contact.i2);
			Particle& p3 = mModel->GetParticle(contact.i3);
			Particle& p4 = mModel->GetParticle(contact.i4);

			Vector3 p, coords;
			int region = Geometry::ClosestPtPointTriangle(p4.pos, p1.pos, p2.pos, p3.pos, p, coords);
			contact.w1 = coords.x;
			contact.w2 = coords.y;
			contact.w3 = coords.z;
			//Vector3 p = contact.w1 * p1.pos + contact.w2 * p2.pos + contact.w3 * p3.pos;
			Vector3 delta = p4.pos - p;

			Vector3 n = delta;
			//if (region == Geometry::TR_FACE_INTERIOR || delta.Length() < 1e-5f)
				n = (p2.pos - p1.pos).Cross(p3.pos - p1.pos);
			n.Normalize();
			if (n.Dot(contact.normal) < 0)
				n = -n;

			float len = n.Dot(delta);

			if (len > len0)
				continue;

			float invMass = contact.w1 * contact.w1 * p1.invMass + contact.w2 * contact.w2 * p2.invMass +
				contact.w3 * contact.w3 * p3.invMass + p4.invMass;
			float s = 1.f / invMass;
			float err = len - len0; // negative!
			float absErr = fabs(err);
			if (absErr > error)
				error = absErr;
			float dLambda = soften * s * err;

			// the normal displacement
			Vector3 disp = dLambda * n;
			contact.lambda += -dLambda / h;

			//Vector3 vel = contact.w1 * p1.vel + contact.w2 * p2.vel + contact.w3 * p3.vel;
			//Vector3 deltaV = vel - p4.vel;
			//float vrel = contact.normal.Dot(deltaV);

			// friction
			//Vector3 vt = deltaV - vrel * contact.normal;
			//float slip = vt.Length();
			//if (fabs(slip) > 0.001f)
			//{
			//	Vector3 tanDir = (1.f / slip) * vt;
			//	float limit = mu * contact.lambda;

			//	float lambdaF0 = contact.lambdaF;
			//	contact.lambdaF += slip * s;
			//	if (contact.lambdaF > limit)
			//		contact.lambdaF = limit;
			//	float dLambdaF = contact.lambdaF - lambdaF0;

			//	Vector3 impF = dLambdaF * tanDir;
			//	disp += -h * impF;
			//}

			p1.pos += disp * (contact.w1 * p1.invMass);
			p2.pos += disp * (contact.w2 * p2.invMass);
			p3.pos += disp * (contact.w3 * p3.invMass);
			p4.pos -= disp * (p4.invMass);
		}
		return error;
	}


	float ClothCollisionHandler::SolveSelfEdgesPosition(float h)
	{
		float shrink = .9f;
		float soften = .9f;
		float thickness = mModel->GetThickness() * shrink;
		float mu = mModel->GetFriction();
		float error = 0;
		for (size_t i = 0; i < mModel->GetNumSelfEdges(); i++)
		{
			SelfContact& contact = mModel->GetSelfEdge(i);
			Particle& p1 = mModel->GetParticle(contact.i1);
			Particle& p2 = mModel->GetParticle(contact.i2);
			Particle& p3 = mModel->GetParticle(contact.i3);
			Particle& p4 = mModel->GetParticle(contact.i4);

			Vector3 p, q;
			float t1, t2;
			Geometry::ClosestPtSegmSegm(p1.pos, p2.pos, p3.pos, p4.pos, t1, t2, p, q);
			contact.w1 = 1 - t1;
			contact.w2 = 1 - t2;

			float omw1 = 1.f - contact.w1;
			float omw2 = 1.f - contact.w2;

			//Vector3 p = contact.w1 * p1.pos + omw1 * p2.pos;
			//Vector3 q = contact.w2 * p3.pos + omw2 * p4.pos;
			Vector3 delta = q - p;
			Vector3 n = delta;
			n.Normalize();
			if (n.Dot(contact.normal) < 0)
			{
				n = -n;
			}

			float len = n.Dot(delta);
			if (len > thickness)
				continue;
			float invMass = contact.w1 * contact.w1 * p1.invMass + omw1 * omw1 * p2.invMass
				+ contact.w2 * contact.w2 * p3.invMass + omw2 * omw2 * p4.invMass;
			float s = 1.f / invMass;
			
			float err = len - thickness;
			float absErr = fabs(err);
			if (absErr > error)
				error = absErr;
			float dLambda = -soften * s * err;
			contact.lambda += dLambda / h;

			// the normal displacement
			Vector3 disp = dLambda * n;

			// friction
			//Vector3 vp = p1.vel + contact.w1 * (p2.vel - p1.vel);
			//Vector3 vq = p3.vel + contact.w2 * (p4.vel - p3.vel);
			//Vector3 deltaV = vp - vq;
			//float vrel = contact.normal.Dot(deltaV);
			//Vector3 vt = deltaV - vrel * contact.normal;
			//float slip = vt.Length();
			//if (fabs(slip) > 0.001f)
			//{
			//	Vector3 tanDir = (1.f / slip) * vt;
			//	float limit = mu * contact.lambda;

			//	float lambdaF0 = contact.lambdaF;
			//	contact.lambdaF += slip * s;
			//	if (contact.lambdaF > limit)
			//		contact.lambdaF = limit;
			//	float dLambdaF = contact.lambdaF - lambdaF0;

			//	// the friction impulse
			//	Vector3 impF = dLambdaF * tanDir;
			//	disp += impF * h;
			//}

			p1.pos -= disp * (omw1 * p1.invMass);
			p2.pos -= disp * (contact.w1 * p2.invMass);
			p3.pos += disp * (omw2 * p3.invMass);
			p4.pos += disp * (contact.w2 * p4.invMass);
		}
		return error;
	}

}