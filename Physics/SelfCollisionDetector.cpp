#include "SelfCollisionDetector.h"
#include "Engine/Profiler.h"
#include "ClothTypes.h"
#include "ClothModel.h"
#include "Geometry/AabbTree.h"
#include "Physics/ElTopoWrapper.h"
#include <stack>

//#define USE_EL_TOPO

using namespace Geometry;
using namespace Math;

namespace Physics
{
	void SelfCollisionsDetector::Detect()
	{
		PROFILE_SCOPE("Detect SelfCollision");
		Midphase();
		Narrowphase();
	}

	void SelfCollisionsDetector::EdgeEdgeDetection()
	{
		//PROFILE_SCOPE("Detect self edge-edge");
		
		#pragma omp parallel for
		for (int k = 0; k < (int)mEdgeEdgeCandidates.size(); k++)
		{
			int e1 = mEdgeEdgeCandidates[k].idx1;
			int e2 = mEdgeEdgeCandidates[k].idx2;

			int i1 = mModel->GetEdge(e1).i1;
			int i2 = mModel->GetEdge(e1).i2;
			const Vector3& x1 = mModel->GetParticle(i1).prev;
			const Vector3& x2 = mModel->GetParticle(i2).prev;
			const Vector3& y1 = mModel->GetParticle(i1).pos;
			const Vector3& y2 = mModel->GetParticle(i2).pos;
			Vector3 v1 = y1 - x1;
			Vector3 v2 = y2 - x2;

			int j1 = mModel->GetEdge(e2).i1;
			int j2 = mModel->GetEdge(e2).i2;

			// skip if edges are adjacent
			if (i1 == j1 || i1 == j2 || i2 == j1 || i2 == j2)
				continue;

			const Vector3& x3 = mModel->GetParticle(j1).prev;
			const Vector3& x4 = mModel->GetParticle(j2).prev;
			const Vector3& y3 = mModel->GetParticle(j1).pos;
			const Vector3& y4 = mModel->GetParticle(j2).pos;
			Vector3 v3 = y3 - x3;
			Vector3 v4 = y4 - x4;

			EdgeEdgeTest(x1, x2, x3, x4, v1, v2, v3, v4, i1, i2, j1, j2, e1, e2);
		}
	}

	void SelfCollisionsDetector::Midphase()
	{
		PROFILE_SCOPE("SelfTree");
		mVertexTriangleCandidates.clear();
		mEdgeEdgeCandidates.clear();

		// traverse the tree
		std::stack<AabbTree*> stack;
		stack.push(mModel->GetTree());
		while (!stack.empty())
		{
			AabbTree* node = stack.top();
			stack.pop();
			if (node->left)
				stack.push(node->left);
			if (node->right)
				stack.push(node->right);

			// triangle-vertex checks
			for (size_t j = 0; j < node->triangles.size(); j++)
			{
				int jj = node->triangles[j].idx; // the triangle index
				const uint32 j1 = mModel->GetTriangle(jj).i1;
				const uint32 j2 = mModel->GetTriangle(jj).i2;
				const uint32 j3 = mModel->GetTriangle(jj).i3;

				// test all triangles in this leaf against all vertices in the leaf
				for (size_t ii = 0; ii < node->vertices.size(); ii++)
				{
					int i = node->vertices[ii].idx; // the vertex index

					// make sure the vertex is not part of the triangle
					if (i == j1 || i == j2 || i == j3 || mModel->GetParticle(i).invMass == 0)
						continue;

					if (AabbOverlap3D(node->triangles[j].box, node->vertices[ii].box))
					{
						PrimitivePair pair;
						pair.idx1 = jj;
						pair.idx2 = i;
						mVertexTriangleCandidates.push_back(pair);
					}
				}
			}

			// edge-edge checks
			for (int i = 0; i < (int)node->edges.size() - 1; i++)
			{
				for (size_t j = i + 1; j < node->edges.size(); j++)
				{
					int e1 = node->edges[i].idx;
					int e2 = node->edges[j].idx;

					if (e1 >= e2)
						std::swap(e1, e2);

					int i1 = mModel->GetEdge(e1).i1;
					int i2 = mModel->GetEdge(e1).i2;
					int i3 = mModel->GetEdge(e2).i1;
					int i4 = mModel->GetEdge(e2).i2;

					// TODO: zero inv mass
					if (i1 == i3 || i2 == i4 || i1 == i4 || i2 == i3)
						continue;

					if (AabbOverlap3D(node->edges[i].box, node->edges[j].box))
					{
						PrimitivePair pair;
						pair.idx1 = e1;
						pair.idx2 = e2;
						mEdgeEdgeCandidates.push_back(pair);
					}
				}
			}
		}
	}

	void SelfCollisionsDetector::Narrowphase()
	{
		PROFILE_SCOPE("SelfNarrow");
		// go through all potential pairs (in parallel) and check for both discrete and continuous collisions 
		#pragma omp parallel for
		for (int k = 0; k < (int)mVertexTriangleCandidates.size(); k++)
		{
			int idx1 = mVertexTriangleCandidates[k].idx1;
			int idx2 = mVertexTriangleCandidates[k].idx2;

			uint32 i1 = mModel->GetTriangle(idx1).i1;
			uint32 i2 = mModel->GetTriangle(idx1).i2;
			uint32 i3 = mModel->GetTriangle(idx1).i3;

			const Vector3& x1 = mModel->GetParticle(i1).prev;
			const Vector3& x2 = mModel->GetParticle(i2).prev;
			const Vector3& x3 = mModel->GetParticle(i3).prev;
			const Vector3& y1 = mModel->GetParticle(i1).pos;
			const Vector3& y2 = mModel->GetParticle(i2).pos;
			const Vector3& y3 = mModel->GetParticle(i3).pos;
			Vector3 v1 = y1 - x1;
			Vector3 v2 = y2 - x2;
			Vector3 v3 = y3 - x3;

			const Vector3& x4 = mModel->GetParticle(idx2).prev;
			const Vector3& y4 = mModel->GetParticle(idx2).pos;
			Vector3 v4 = y4 - x4;

			VertexTriangleTest(x1, x2, x3, x4, v1, v2, v3, v4, i1, i2, i3, idx2);
		}

		EdgeEdgeDetection();
	}

	void SelfCollisionsDetector::VertexTriangleTest(const Vector3& x1, const Vector3& x2, const Vector3& x3, const Vector3& x4,
		const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& v4,
		int i1, int i2, int i3, int i4)
	{
		//float tol = mModel->GetTolerance();
		
		// discrete test on previous positions
		Vector3 n, p;
		float dSqr; // distance squared
		BarycentricCoords coords;
		bool intersect = false; // IntersectSphereTriangle(x4, mModel->GetThickness(), x1, x2, x3, n, p, dSqr, coords);

		if (!intersect)
		{
			// CCD test now
#ifndef USE_EL_TOPO
			intersect = VertexTriangleCCD(x1, x2, x3, x4, v1, v2, v3, v4, i1, i2, i3, i4, n, coords);
#else
			Vec3st tri(i1, i2, i3);
			Vec3st sorted_tri = sort_triangle(tri);
			i1 = sorted_tri[0];
			i2 = sorted_tri[1];
			i3 = sorted_tri[2];

			double w1, w2, w3, d;
			Vec3d normal;
			intersect = point_triangle_collision(V2ETV(x4), V2ETV(x4 + v4), i4,
				V2ETV(x1), V2ETV(x1 + v1), i1,
				V2ETV(x2), V2ETV(x2 + v2), i2,
				V2ETV(x3), V2ETV(x3 + v3), i3,
				w1, w2, w3, normal, d);
			
			if (intersect)
			{
				coords.u = (float)w1;
				coords.v = (float)w2;
				coords.w = (float)w3;
				n = ETV2V(normal);
			}
#endif
		}
		
		//if (!intersect)
		//{
		//	// discrete test on current positions
		//	intersect = IntersectSphereTriangle(x4 + v4, mModel->GetThickness(), x1 + v1, x2 + v2, x3 + v3, n, p, dSqr, coords);
		//}

		if (intersect)
		{
			SelfContact c;
			c.i1 = i1;
			c.i2 = i2;
			c.i3 = i3;
			c.i4 = i4;
			c.w1 = coords.x;
			c.w2 = coords.y;
			c.w3 = coords.z;
			c.normal = n;
			c.isCCD = true;
			c.side = SignedVolume(x4, x1, x2, x3);
			#pragma omp critical
			mModel->AddSelfTriangle(c);
		}
	}

	bool SelfCollisionsDetector::VertexTriangleCCD(const Vector3& x1, const Vector3& x2, const Vector3& x3, const Vector3& x4,
		const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& v4,
		int i1, int i2, int i3, int i4, Vector3& normal, BarycentricCoords& barycentric)
	{
		float toi; // time of impact
		bool coplanar = SolveCubic(x1, x2, x3, x4, v1, v2, v3, v4, toi);
		// if we have a hit, i.e. the points have become coplanar
		if (coplanar)
		{
			ASSERT(toi >= 0 && toi <= 1);

			Vector3 z1 = x1 + toi * v1;
			Vector3 z2 = x2 + toi * v2;
			Vector3 z3 = x3 + toi * v3;
			Vector3 z4 = x4 + toi * v4;

			BarycentricCoords coords;
			Vector3 p;
			int region = ClosestPtPointTriangle(z4, z1, z2, z3, p, coords);
			Vector3 delta = z4 - p;
			const float eps = 0.1f;// 0.5f * mModel->GetThickness();
			// check if the point is really close to the triangle
			if (delta.LengthSquared() < eps * eps)
			{
				// what side was the point initially on?
				Vector3 triNormal = cross(x2 - x1, x3 - x1);
				triNormal.Normalize();
				float side = triNormal.Dot(x4 - x1);

				// now choose the triangle normal direction according to it
				Vector3 n = cross(z2 - z1, z3 - z1);
				n.Normalize();
				if (side < 0) n.Flip();

				normal = n;
				barycentric = coords;
				return true;
			}
		}
		return false;
	}

	float SelfCollisionsDetector::EvaluateFun_Coplanarity(float t, const Vector3& x1, const Vector3& x2, const Vector3& x3, const Vector3& x4,
		const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& v4)
	{
		Vector3 z1 = x1 + t * v1;
		Vector3 z2 = x2 + t * v2;
		Vector3 z3 = x3 + t * v3;
		Vector3 z4 = x4 + t * v4;

		// evaluate function
		Vector3 n1 = cross(z2 - z1, z3 - z1);
		float vol = n1.Dot(z4 - z1);
		return vol;
	}
	
	bool SelfCollisionsDetector::SolveCubic(const Vector3& x1, const Vector3& x2, const Vector3& x3, const Vector3& x4,
		const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& v4, float& toi)
	{
		// compute the value of the function at the endpoints of [0, 1]
		float f0 = EvaluateFun_Coplanarity(0.f, x1, x2, x3, x4, v1, v2, v3, v4);

		// if they are already coplanar, they are probably not going to intersect
		if (fabs(f0) < 0.001f)
			return false;

		// compute the cubic coefficients
		Vector3 v10 = v1 - v4;
		Vector3 v20 = v2 - v4;
		Vector3 v30 = v3 - v4;

		Vector3 x10 = x1 - x4;
		Vector3 x20 = x2 - x4;
		Vector3 x30 = x3 - x4;

		bool coplanar = false;

		float a = triple(v10, v20, v30);
		float b = triple(x10, v20, v30) + triple(v10, x20, v30) + triple(v10, v20, x30);
		float c = triple(x10, x20, v30) + triple(x10, v20, x30) + triple(v10, x20, x30);
		if (fabs(a) < 0.001f)
		{
			if (fabs(b) < 0.001f)
				return false; // we're not handling this case (yet)
			float d = triple(x10, x20, x30);
			coplanar = CheckDegenerateCase(b, c, d, toi);
		}
		else
		{
			CCDFunction fun = &SelfCollisionsDetector::EvaluateFun_Coplanarity;
			// analyze inflexion points
			float tInfl1, tInfl2;
			int numInflPts = FindInflexionPoints(a, b, c, tInfl1, tInfl2);
			if (numInflPts == 0)
			{
				// no inflextion point inside [0, 1]
				coplanar = CheckInterval(fun, 0, 1, x1, x2, x3, x4, v1, v2, v3, v4, toi);
			}
			else if (numInflPts == 1)
			{
				if (tInfl1 < 0 || tInfl1 > 1)
				{
					// no inflextion point inside [0, 1]
					coplanar = CheckInterval(fun, 0, 1, x1, x2, x3, x4, v1, v2, v3, v4, toi);
				}
				else
				{
					coplanar = CheckInterval(fun, 0, tInfl1, x1, x2, x3, x4, v1, v2, v3, v4, toi) ||
						CheckInterval(fun, tInfl1, 1, x1, x2, x3, x4, v1, v2, v3, v4, toi);
				}
			}
			else
			{
				if (tInfl1 > 1 || tInfl2 < 0 || (tInfl1 < 0 && tInfl2 > 1))
				{
					// no inflextion point inside [0, 1]
					coplanar = CheckInterval(fun, 0, 1, x1, x2, x3, x4, v1, v2, v3, v4, toi);
				}
				else if (tInfl1 < 0 && tInfl2 < 1)
				{
					// only t2 is in the interval
					ASSERT(tInfl2 > 0);
					coplanar = CheckInterval(fun, 0, tInfl2, x1, x2, x3, x4, v1, v2, v3, v4, toi) ||
						CheckInterval(fun, tInfl2, 1, x1, x2, x3, x4, v1, v2, v3, v4, toi);
				}
				else if (tInfl1 < 1 && tInfl2 > 1)
				{
					// only t1 is in the interval
					ASSERT(tInfl1 > 0);
					coplanar = CheckInterval(fun, 0, tInfl1, x1, x2, x3, x4, v1, v2, v3, v4, toi) ||
						CheckInterval(fun, tInfl1, 1, x1, x2, x3, x4, v1, v2, v3, v4, toi);
				}
				else if (tInfl1 > 0 && tInfl2 < 1)
				{
					// both of them are in the interval
					ASSERT(tInfl1 < 1 && tInfl2 > 0);
					coplanar = CheckInterval(fun, 0, tInfl1, x1, x2, x3, x4, v1, v2, v3, v4, toi) ||
						CheckInterval(fun, tInfl1, tInfl2, x1, x2, x3, x4, v1, v2, v3, v4, toi) ||
						CheckInterval(fun, tInfl2, 1, x1, x2, x3, x4, v1, v2, v3, v4, toi);
				}
			}
		}
		if (coplanar)
		{
			ASSERT(toi >= 0 && toi <= 1);
		}
		return coplanar;
	}

	bool SelfCollisionsDetector::CheckDegenerateCase(float b, float c, float d, float& toi)
	{
		// the equation is quadratic
		float discr = c * c - 4 * b * d;
		if (discr < 0)
			return false;

		if (discr == 0)
		{
			float t = -0.5f * c / b;
			if (t < 0 || t > 1)
				return false;
			toi = t;
			return true;
		}

		float discrSqrt = sqrt(discr);
		float t1 = 0.5f * (-c - discrSqrt) / b;
		float t2 = 0.5f * (-c + discrSqrt) / b;
		float t = std::min(t1, t2);
		if (t < 0 || t > 1)
			return false;
		
		toi = t;
		ASSERT(toi >= 0 && toi <= 1);
		return true;
	}

	// returns the number of inflexion points and outputs their values
	int SelfCollisionsDetector::FindInflexionPoints(float a, float b, float c, float& tInfl1, float& tInfl2)
	{
		ASSERT(fabs(a) > 0.001f);

		float discr = b * b - 3 * a * c;
		if (discr < 0)
			return 0;

		if (discr == 0)
		{
			tInfl1 = tInfl2 = -b / (3 * a);
			return 1;
		}

		float discrSqrt = sqrt(discr);
		tInfl1 = (-b - discrSqrt) / (3 * a);
		tInfl2 = (-b + discrSqrt) / (3 * a);

		if (tInfl2 < tInfl1)
			std::swap(tInfl1, tInfl2);
		ASSERT(tInfl1 < tInfl2);

		return 2;
	}

	bool SelfCollisionsDetector::CheckInterval(CCDFunction fun, float ta, float tb, const Vector3& x1, const Vector3& x2, const Vector3& x3, const Vector3& x4,
		const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& v4, float& tx)
	{
		// interval variables
		float tx0 = ta;
		float tx1 = tb;
		float fx0 = (this->*fun)(tx0, x1, x2, x3, x4, v1, v2, v3, v4);
		float fx1 = (this->*fun)(tx1, x1, x2, x3, x4, v1, v2, v3, v4);

		// we assume we know there are no inflexion points inside this interval
		if (fx0 * fx1 > 0)
			return false; // there is no solution in this interval

		// TODO: secant method
		// bisect for a time after which the function crosses the 0 line
		tx = tx0; // time of impact (or just before it)
		for (int l = 0; l < 40; l++) // TODO: stop criterion only
		{
			float tmid = 0.5f * (tx0 + tx1);
			float fmid = (this->*fun)(tmid, x1, x2, x3, x4, v1, v2, v3, v4);
			if (fabs(fmid) < 0.01f) // TODO: epsilon
			{
				// we have a solution
				tx = tmid;
				break;
			}

			// now look for the interval closer to 0 that contains the solution
			if (fx0 * fmid < 0)
			{
				tx1 = tmid;
				fx1 = fmid;
				tx = tx0;
			}
			else if (fmid * fx1 < 0)
			{
				tx0 = tmid;
				fx0 = fmid;
				tx = tmid;
			}
			else
			{
				// means we have no solution in either interval
				return false;
			}
		}

		ASSERT(tx >= 0 && tx <= 1);
		return true;
	}

	void SelfCollisionsDetector::EdgeEdgeTest(const Vector3& x1, const Vector3& x2, const Vector3& x3, const Vector3& x4,
		const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& v4,
		int i1, int i2, int i3, int i4, int e1, int e2)
	{
		Vector3 n;
		// discrete collision for previous positions
		//float thickness = mModel->GetThickness();
		//Vector3 c1, c2;
		float s, t;
		//ClosestPtSegmSegm(x1, x2, x3, x4, s, t, c1, c2);
		//float dSqr = (c1 - c2).LengthSquared();
		//Vector3 n = c2 - c1;
		//if (n.Length() < 1e-6f)
		//{
		//	Vector3 e1 = x2 - x1;
		//	Vector3 e2 = x4 - x3;
		//	n = cross(e1, e2);
		//}
		//n.Normalize();
		bool intersect = false; // dSqr <= thickness * thickness;

		if (!intersect)
		{
			// CCD edge-edge
#ifndef USE_EL_TOPO
			intersect = EdgeEdgeCCD(x1, x2, x3, x4, v1, v2, v3, v4, i1, i2, i3, i4, n, s, t);
#else
			double w1, w2, d;
			Vec3d normal;
			intersect = segment_segment_collision(V2ETV(x1), V2ETV(x1 + v1), i1,
				V2ETV(x2), V2ETV(x2 + v2), i2,
				V2ETV(x3), V2ETV(x3 + v3), i3,
				V2ETV(x4), V2ETV(x4 + v4), i4, w1, w2, normal, d);
			if (intersect)
			{
				s = (float)w1;
				t = (float)w2;
				n = ETV2V(normal);
			}
#endif
		}

		//if (!intersect)
		//{
		//	// discrete collision for current positions
		//	ClosestPtSegmSegm(x1 + v1, x2 + v2, x3 + v3, x4 + v4, s, t, c1, c2);
		//	n = c2 - c1;
		//	float dSqr = n.LengthSquared();
		//	if (n.Length() < 1e-6f)
		//	{
		//		Vector3 e1 = x2 + v2 - x1 - v1;
		//		Vector3 e2 = x4 + v4 - x3 - v3;
		//		n = cross(e1, e2);
		//	}
		//	n.Normalize();
		//	intersect = dSqr <= thickness * thickness;
		//}
		
		if (intersect)
		{
			SelfContact c;
			c.i1 = i1;
			c.i2 = i2;
			c.i3 = i3;
			c.i4 = i4;
			c.w1 = s;
			c.w2 = t;
			
			Vector3 c1, c2;
			ClosestPtSegmSegm(x1, x2, x3, x4, s, t, c1, c2);
			c.normal = c2 - c1;
			c.normal.Normalize();
			
			c.isCCD = true;
			c.side = SignedVolume(x1, x2, x3, x4);
			c.hash = (e1 & 0xffff) | (e2 << 16);
			#pragma omp critical
			mModel->AddSelfEdge(c);
		}
	}

	bool SelfCollisionsDetector::EdgeEdgeCCD(const Vector3& x1, const Vector3& x2, const Vector3& x3, const Vector3& x4,
		const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& v4,
		int i1, int i2, int i3, int i4, Vector3& normal, float& s, float& t)
	{
		float toi; // time of impact
		bool coplanar = SolveCubic(x1, x2, x3, x4, v1, v2, v3, v4, toi);
		// if we have a hit, i.e. the points have become coplanar
		if (coplanar)
		{
			ASSERT(toi >= 0 && toi <= 1);

			Vector3 z1 = x1 + toi * v1;
			Vector3 z2 = x2 + toi * v2;
			Vector3 z3 = x3 + toi * v3;
			Vector3 z4 = x4 + toi * v4;

			Vector3 c1, c2;
			ClosestPtSegmSegm(z1, z2, z3, z4, s, t, c1, c2);
			float dSqr = (c1 - c2).LengthSquared();
			const float eps = 0.5f * mModel->GetThickness();
			// check if the segments are really close to each other
			if (dSqr <= eps * eps)
			{
				//Vector3 n = cross(z2 - z1, z4 - z3);
				//Vector3 n0 = cross(x2 - x1, x4 - x3);
				Vector3 p = x1 + s * (x2 - x1);
				Vector3 q = x3 + t * (x4 - x3);
				Vector3 delta = q - p;
				normal = delta;
				normal.Normalize();
				return true;
			}
		}
		return false;
	}

}