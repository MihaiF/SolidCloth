#include "ClothCollisionHandler.h"
#include "Engine/Profiler.h"
#include "ClothModel.h"

namespace Physics
{
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

	void ClothCollisionHandler::DetectAndHandle(float h)
	{
		PROFILE_SCOPE("Decoupled collisions");

		// self-collision iteration
		uint32 numParticles = (uint32)mModel->GetNumParticles();
		int outerIter;
		for (outerIter = 0; outerIter < numSelfCollIterations; outerIter++)
		{
			// we detect the collisions, that will be put in specific arrays
			// we know/assume that the current positions are the ones after a cloth step and the previous positions are the ones before the cloth step
			mModel->DetectCollisions();

			// now solve the contacts using a velocity approach
			// for now use, the the same number of iterations as for PBD
			mModel->ResetConstraints();
			HandleContactsVelocity(h);
			HandleContactsPosition(h);
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

	void ClothCollisionHandler::SolveContactsPosition(float h)
	{
		PROFILE_SCOPE("Solve Contacts PBD");
		const float invH = 1.f / h;
		const float mu = mModel->GetFriction();
		float thickness = mModel->GetThickness();
		for (size_t i = 0; i < mModel->GetNumContacts(); i++)
		{
			Contact& contact = mModel->GetContact(i);
			Particle& p1 = mModel->GetParticle(contact.idx);
			Vector3 delta = p1.pos - contact.point;
			float len = delta.Dot(contact.normal);
			Vector3 disp = contact.normal;
			float depth = len - thickness;
			if (len > thickness)
				continue; // TODO: clamp lambda instead?
			
			float dLambda = -depth;
			contact.lambda += dLambda;
			disp.Scale(dLambda);

			// friction
			Vector3 v12 = p1.vel;
			float vnrel = contact.normal.Dot(v12);
			Vector3 vt = v12 - vnrel * contact.normal;
			float vtrel = vt.Length();

			const float limit = mu * invH * contact.lambda;

			if (vtrel > 0.001f)
			{
				float dLambdaF = vtrel;
				float lambdaF0 = contact.lambdaF;
				contact.lambdaF = lambdaF0 + dLambdaF;
				if (contact.lambdaF >= limit)
					contact.lambdaF = limit;
				dLambdaF = contact.lambdaF - lambdaF0;

				vt.Scale(1.0f / vtrel); // normalize
				disp += -h * dLambdaF * vt;
			}

			// apply resulting displacement
			p1.pos += disp;
			p1.vel += invH * disp; // for solvers that do not update velocity at the end
		}
	}

	void ClothCollisionHandler::CheckContactsPosition()
	{
		int count = 0;
		int inside = 0;
		float error = 0;
		float thickness = mModel->GetThickness();
		for (size_t i = 0; i < mModel->GetNumContacts(); i++)
		{
			Contact& contact = mModel->GetContact(i);
			Particle& p1 = mModel->GetParticle(contact.idx);
			Vector3 delta = p1.pos - contact.point;
			// compute distance to plane
			float dist = delta.Dot(contact.normal);
			Vector3 disp = contact.normal;
			if (dist > thickness)
				continue;
			
			float depth = dist - thickness;
			error += fabsf(depth);
			count++;
			if (dist < 0)
				inside++;
		}
		Printf("active: %d, inside: %d, error: %g\n", count, inside, error);
	}
}