#include "Collision3D.h"
#include "Engine/Utils.h"

using Math::Vector3;

// from Ericson

namespace Geometry
{
	int ClosestPtSegmSegm(const Vector3& p1, const Vector3& q1, const Vector3& p2, const Vector3& q2,
		float& s, float& t, Vector3& c1, Vector3& c2)
	{
		Vector3 d1 = q1 - p1;
		Vector3 d2 = q2 - p2;
		Vector3 r = p1 - p2;
		float a = d1.LengthSquared();
		float e = d2.LengthSquared();
		float f = d2.Dot(r);

		// TODO: check if degenerate

		float c = d1.Dot(r);
		float b = d1.Dot(d2);
		float denom = a * e - b * b;

		if (denom != 0.f)
			s = Math::clamp((b * f - c * e) / denom, 0.f, 1.f);
		else
			s = 0.f;

		int region = ER_EDGE_INTERIOR;
		t = (b * s + f) / e;

		if (t < 0.f)
		{
			region |= ER_VERTEX_P2;
			t = 0.f;
			s = Math::clamp(-c / a, 0.f, 1.f);
		}
		else if (t > 1.f)
		{
			region |= ER_VERTEX_Q2;
			t = 1.f;
			s = Math::clamp((b - c) / a, 0.f, 1.f);
		}

		// for now, we use 0 and 1 as results of clamping; anything else is considered on the edge
		if (s == 0.f)
			region |= ER_VERTEX_P1;
		if (s == 1.f)
			region |= ER_VERTEX_Q1;

		// epilogue
		c1 = p1 + s * d1;
		c2 = p2 + t * d2;
		
		return region;
	}

	// closest point on triangle (a, b, c) to vertex p
	// outputs the closest point and its barycentric coordinates in bar
	int ClosestPtPointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& bar)
	{
		bar.x = bar.y = bar.z = 0.f;

		// Check if P in vertex region outside A
		Vector3 ab = b - a;
		Vector3 ac = c - a;
		Vector3 ap = p - a;
		float d1 = ab.Dot(ap);
		float d2 = ac.Dot(ap);
		if (d1 <= 0.f && d2 <= 0.f)
		{
			bar.x = 1.f;
			closestPt = a;
			return TR_VERTEX_A;
		}

		// Check if P in vertex region outside B
		Vector3 bp = p - b;
		float d3 = ab.Dot(bp);
		float d4 = ac.Dot(bp);
		if (d3 >= 0.f && d4 <= d3)
		{
			bar.y = 1.f;
			closestPt = b;
			return TR_VERTEX_B;
		}

		// Check if P in edge region of AB, if so return projection
		float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
		{
			bar.y = d1 / (d1 - d3);
			bar.x = 1 - bar.y;
			closestPt = a + bar.y * ab;
			return TR_EDGE_AB;
		}

		// Check if P in vertex region outside C
		Vector3 cp = p - c;
		float d5 = ab.Dot(cp);
		float d6 = ac.Dot(cp);
		if (d6 >= 0.f && d5 <= d6)
		{
			bar.z = 1.f;
			closestPt = c;
			return TR_VERTEX_C;
		}

		// Check if P in edge region of AC
		float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
		{
			bar.z = d2 / (d2 - d6);
			bar.x = 1 - bar.z;
			closestPt = a + bar.z * ac;
			return TR_EDGE_AC;
		}

		// Check if P in edge region of BC
		float va = d3 * d6 - d5 * d4;
		float d43 = d4 - d3;
		float d56 = d5 - d6;
		if (va <= 0.f && d43 >= 0.f && d56 >= 0.f)
		{
			bar.z = d43 / (d43 + d56);
			bar.y = 1 - bar.z;
			closestPt = b + bar.z * (c - b);
			return TR_EDGE_BC;
		}

		// P inside face region
		float denom = 1.f / (va + vb + vc);
		bar.y = vb * denom;
		bar.z = vc * denom;
		bar.x = 1.f - bar.y - bar.z;
		closestPt = a + ab * bar.y + ac * bar.z;
		ASSERT(bar.x > 0 && bar.y > 0 && bar.z > 0);
		return TR_FACE_INTERIOR;
	}

	float ClosestPtSegmTriangle(const Vector3& p, const Vector3& q, const Vector3& a, const Vector3& b, const Vector3& c, float& param, 
		Math::BarycentricCoords& bar, Vector3& closestEdge, Vector3& closestTri)
	{
		float minDist = FLT_MAX;

		float s, t;
		Vector3 c1, c2;
		ClosestPtSegmSegm(p, q, a, b, s, t, c1, c2);
		float dist1 = (c1 - c2).Length();
		if (dist1 < minDist)
		{
			minDist = dist1;
			closestEdge = c1;
			closestTri = c2;
			param = s;
			bar = { 1 - t, t, 0 }; //?
		}

		ClosestPtSegmSegm(p, q, b, c, s, t, c1, c2);
		float dist2 = (c1 - c2).Length();
		if (dist2 < minDist)
		{
			minDist = dist2;
			closestEdge = c1;
			closestTri = c2;
			param = s;
			bar = { 0, 1 - t, t }; //?
		}

		ClosestPtSegmSegm(p, q, c, a, s, t, c1, c2);
		float dist3 = (c1 - c2).Length();
		if (dist3 < minDist)
		{
			minDist = dist3;
			closestEdge = c1;
			closestTri = c2;
			param = s;
			bar = { t, 0 , 1 - t }; //?
		}

		Vector3 coords;
		Vector3 cp4;
		ClosestPtPointTriangle(p, a, b, c, cp4, coords);
		float dist4 = (p - cp4).Length();
		if (dist4 < minDist)
		{
			minDist = dist4;
			closestEdge = p;
			closestTri = cp4;
			param = 0;
			bar = coords;
		}

		Vector3 cp5;
		ClosestPtPointTriangle(q, a, b, c, cp5, bar);
		float dist5 = (q - cp5).Length();
		if (dist5 < minDist)
		{
			minDist = dist5;
			closestEdge = q;
			closestTri = cp5;
			param = 1;
			bar = coords;
		}

		return minDist;
	}

	bool IntersectSphereTriangle(const Vector3& v, float radTol, const Vector3& v1, const Vector3& v2, const Vector3& v3,
		Vector3& normal, Vector3& pos, float& dist, Math::BarycentricCoords& coords)
	{
		Vector3 p;
		ClosestPtPointTriangle(v, v1, v2, v3, p, coords);
		Vector3 delta = v - p;
		float dSqr = delta.LengthSquared();
		dist = dSqr;
		if (dSqr > radTol * radTol)
			return false;
		if (dSqr == 0)
			return false; // TODO: choose a normal
		pos = p;
		delta.Normalize();
		normal = delta;
		return true;
	}

	bool IntersectSphereTriangle1(const Vector3& v, float radTol, const Vector3& v1, const Vector3& v2, const Vector3& v3, const Vector3& ref,
		Vector3& normal, Vector3& pos, float& dist, Math::BarycentricCoords& coords)
	{
		Vector3 p;
		ClosestPtPointTriangle(v, v1, v2, v3, p, coords);
		Vector3 delta = v - p;

		Vector3 dir = ref - p;
		dir.Normalize();
		//dir = dir.Dot(normal) > 0 ? normal : -1.f * normal;
		float d = delta.Dot(dir);
		//if (d > radTol)
		//return false;

		float dSqr = delta.LengthSquared();
		if (dSqr > radTol * radTol && d > 0)
			return false;

		//Vector3 delta1 = v - v1;
		//float d1 = normal.Dot(delta1);
		//Vector3 p1 = v - d1 * normal;
		//coords = Barycentric(v1, v2, v3, p1);
		//if (coords.u >= -0.1f && coords.u <= 1.1f
		//	&& coords.v >= -0.1f && coords.v <= 1.1f
		//	&& coords.w >= -0.1f && coords.w <= 1.1f)
		//{
		//	dSqr = min(dSqr, d1 * d1);
		//}

		pos = p;
		delta.Normalize();
		normal = delta;
		dist = sqrtf(dSqr);
		//dist = fabs(d);
		return true;
	}

	// broken?
	bool IntersectSegmentTriangleOld(const Vector3& p, const Vector3& q, // the segment
		const Vector3& a, const Vector3& b, const Vector3& c, // the triangle
		Math::BarycentricCoords& coords, float& t, Vector3* tn)
	{
		Vector3 ab = b - a;
		Vector3 ac = c - a;
		Vector3 qp = p - q;

		Vector3 n = tn != NULL ? *tn : ab.Cross(ac);

		float d = n.Dot(qp);
		if (d <= 0.f) return false;

		Vector3 ap = p - a;
		t = n.Dot(ap);
		if (t < 0.f || t > d) return false;

		Vector3 e = qp.Cross(ap);
		float v = ac.Dot(e);
		if (v < 0.f || v > d) return false;
		float w = -ab.Dot(e);
		if (w < 0.f || v + w > d) return false;

		float ood = 1.f / d;
		t *= ood;
		coords.y = v * ood;
		coords.z = w * ood;
		coords.x = 1.f - coords.x - coords.z;
		return true;
	}

	float SignedVolume(const Vector3& a0, const Vector3& a1, const Vector3& a2, const Vector3& a3)
	{
		return triple(a1 - a0, a2 - a0, a3 - a0);
	}

	bool SameSign(float a, float b)
	{
		// if one of them is zero, it doesn't matter the sign of the other
		if (a == 0 || b == 0)
			return true;
		// TODO: use an epsilon
		if (a > 0 && b > 0)
			return true;
		if (a < 0 && b < 0)
			return true;
		return false;
	}

	bool IntersectSegmentTriangle(const Vector3& p, const Vector3& q, const Vector3& a, const Vector3& b, const Vector3& c, 
		Math::BarycentricCoords& coords, float& t, Vector3& r, bool isRay)
	{
		// TODO: make sure we treat the parallel case correctly
		Vector3 ab = b - a;
		Vector3 ac = c - a;
		Vector3 qp = p - q;

		Vector3 n = ab.Cross(ac);

		float d = n.Dot(qp);
		if (fabs(d) <= 1e-5f)
			return false;

		// this is the double sided version of the test
		// TODO: reinstate the counter-clockwise triangle test

		const float epsilon = 1e-7f;

		// TODO: ray case

		coords.x = SignedVolume(b, c, p, q);
		//if (coords.u < -epsilon)
		//	return false;
		coords.y = SignedVolume(c, a, p, q);
		//if (coords.v < -epsilon)
		//	return false;
		if (!SameSign(coords.x, coords.y))
			return false;
		coords.z = SignedVolume(a, b, p, q);
		//if (coords.w < -epsilon)
		//	return false;
		if (!SameSign(coords.x, coords.z) || !SameSign(coords.y, coords.z))
			return false;

		float sumTri = coords.x + coords.y + coords.z;

		// this is the case when PQ lies in the plane of the triangle
		// TODO: handle separately
		if (fabs(sumTri) < epsilon)
			return false;

		float invSumTri = 1.f / sumTri;
		coords.x *= invSumTri;
		coords.y *= invSumTri;
		coords.z *= invSumTri;

		// TODO: check that the barycentric coords are really inside

		r = coords.x * a + coords.y * b + coords.z * c;
		Vector3 pq = q - p;
		Vector3 pr = r - p;
		float len = pq.Length();
		t = pr.Dot(pq) / len / len;

		if (t < -epsilon || (t > (1.0f + epsilon) && !isRay))
			return false;
		// TODO: check if points correspond

		return true;
	}

	bool IntersectSweptSphereTriangle(const Vector3& c, float radius, const Vector3& v, // the swept sphere
		const Vector3& v1, const Vector3& v2, const Vector3& v3, // the triangle
		Vector3& pos, Vector3& normal, float& dist, Math::BarycentricCoords& coords)
	{
		Vector3 n = (v2 - v1).Cross(v3 - v1);
		n.Normalize();
		Vector3 d = c - radius * n;

		float t;
		Vector3 p;
		if (!IntersectSegmentPlane(d, d + v, n, v1.Dot(n), t, p))
		{
			return false;
		}

		coords = Barycentric(v1, v2, v3, p);
		if (coords.y >= 0.f && coords.z >= 0.f && coords.x >= 0.f)
		{
			// intersection is inside triangle
			pos = d + t * v;
			normal = n;
			return true;
		}

		// TODO: use the fact that it's in the same plane and/or its barycentric coordinates
		Vector3 q;
		ClosestPtPointTriangle(p, v1, v2, v3, q, coords);
		if (IntersectRaySphere(q, -v, c, radius, t, p) && t <= 1.f)
			//Vector3 r = ClosestPtSegm(q, c, c + v, t);
			//if ((q - r).LengthSquared() < radius * radius)
		{
			pos = q;
			normal = n;
			return true;
		}

		return false;
	}

} // namespace Geometry