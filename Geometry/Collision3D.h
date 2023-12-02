#ifndef COLLISION3D_H
#define COLLISION3D_H

#include <Math/Vector3.h>
#include <Math/Vector2.h>
#include <Engine/Types.h>
#include <vector>
#include <Math/Utils.h>

namespace Geometry
{
	enum CollFlags
	{
		CF_VERTICES = 1,
		CF_TRIANGLES = 2,
		CF_SELF = 4,
		CF_WALLS = 8,
		CF_EDGES = 16,
	};

	enum TriangleRegion
	{
		TR_VERTEX_A,
		TR_VERTEX_B,
		TR_VERTEX_C,
		TR_EDGE_AB,
		TR_EDGE_AC,
		TR_EDGE_BC,
		TR_FACE_INTERIOR,
	};

	enum EdgeRegion
	{
		ER_VERTEX_P2,
		ER_VERTEX_Q2,
		ER_EDGE2_INTERIOR,
	};

	// I think this does not do projection, so it requires that p is in the (a, b, c) plane
	inline Math::BarycentricCoords Barycentric(const Math::Vector3& a, const Math::Vector3& b, const Math::Vector3& c, const Math::Vector3& p)
	{
		Math::Vector3 v0 = b - a;
		Math::Vector3 v1 = c - a;
		Math::Vector3 v2 = p - a;
		float d00 = v0.LengthSquared();
		float d01 = v0.Dot(v1);
		float d11 = v1.LengthSquared();
		float d20 = v2.Dot(v0);
		float d21 = v2.Dot(v1);
		float denom = d00 * d11 - d01 * d01;
		Math::BarycentricCoords ret;
		ret.y = (d11 * d20 - d01 * d21) / denom;
		ret.z = (d00 * d21 - d01 * d20) / denom;
		ret.x = 1.f - ret.y - ret.z;
		return ret;
	}

	// Takes sphere 1 (p1, r1) and sphere 2 (p2, r2) and checks if they overlap.
	// If they do, it outputs the squared distance between the the centers, computes the point on sphere1 (pos)
	// and a normal pointing from 1 to 2 (along the line between the centers)
	inline bool IntersectSphereSphere(const Math::Vector3& p1, float r1, const Math::Vector3& p2, float r2, 
		Math::Vector3& normal, Math::Vector3& pos, float& dSqr)
	{
		const float dist = r1 + r2;
		const float radSqr = dist * dist;
		Math::Vector3 delta = p2 - p1;
		dSqr = delta.LengthSquared();
		if (dSqr >= radSqr)
			return false;
		delta.Normalize();
		normal = delta; // pointing from 1 to 2
		pos = p1 + delta * r1;
		return true;
	}

	// from Ericson
	inline bool IntersectSegmentPlane(const Math::Vector3& a, const Math::Vector3& b, // the segment
		const Math::Vector3& n, float d, // the plane
		float& t, Math::Vector3& q)
	{
		Math::Vector3 ab = b - a;
		t = (d - n.Dot(a)) / n.Dot(ab);
		if (t >= 0.f && t < 1.f)
		{
			q = a + t * ab;
			return true;
		}
		return false;
	}

	inline bool IntersectRayPlane(const Math::Vector3& a, const Math::Vector3& ab, // the segment
		const Math::Vector3& n, float d, // the plane
		float& t, Math::Vector3& q)
	{
		t = (d - n.Dot(a)) / n.Dot(ab);
		if (t >= 0.f)
		{
			q = a + t * ab;
			return true;
		}
		return false;
	}

	inline bool IntersectRaySphere(const Math::Vector3& p, const Math::Vector3& d, // the ray
		const Math::Vector3& center, float radius, // the sphere,
		float& t, Math::Vector3& q)
	{
		Math::Vector3 m = p - center;
		float b = m.Dot(d);
		float c = m.LengthSquared() - radius * radius;
		if (c > 0.f && b > 0.f) return false;
		float discr = b * b - c;
		if (discr < 0.f) return false;
		t = -b - (float)sqrt(discr);
		// if t is negative, ray started inside sphere so clamp it to zero
		if (t < 0.f)
			t = 0.f;
		q = p + t * d;
		return true;
	}

	// Returns the closest point between a point p and segment (a, b), including the parameter t
	inline Math::Vector3 ClosestPtSegm(const Math::Vector3& p, const Math::Vector3& a, const Math::Vector3& b, float& t)
	{
		Math::Vector3 u = b - a;
		Math::Vector3 v = p - a;
		t = u.Dot(v) / u.LengthSquared();
		if (t <= 0.f)
		{
			t = 0.f;
			return a;
		}
		if (t >= 1.f)
		{
			t = 1.f;
			return b;
		}
		return a + t * u;
	}


	// closest points between segments (p1, q1) and (p2, q2)
	// returns region (endpoint or interior)
	int ClosestPtSegmSegm(const Math::Vector3& p1, const Math::Vector3& q1, const Math::Vector3& p2, const Math::Vector3& q2, float& s, float& t, Math::Vector3& c1, Math::Vector3& c2);

	// closest point on triangle (a, b, c) to vertex p
	// outputs the closest point and its barycentric coordinates in bar
	int ClosestPtPointTriangle(Math::Vector3 p, Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3& closestPt, Math::Vector3& bar);

	float ClosestPtSegmTriangle(const Math::Vector3& p, const Math::Vector3& q, const Math::Vector3& a, const Math::Vector3& b, const Math::Vector3& c, float& param, Math::BarycentricCoords& bar,
		Math::Vector3& closest1, Math::Vector3& closest2);

	bool IntersectSphereTriangle(const Math::Vector3& v, float radTol, const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3,
		Math::Vector3& normal, Math::Vector3& pos, float& dist, Math::BarycentricCoords& coords);

	bool IntersectSphereTriangle1(const Math::Vector3& v, float radTol, const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& ref,
		Math::Vector3& normal, Math::Vector3& pos, float& dist, Math::BarycentricCoords& coords);

	bool IntersectSegmentTriangle(const Math::Vector3& p, const Math::Vector3& q, // the segment
		const Math::Vector3& a, const Math::Vector3& b, const Math::Vector3& c, // the triangle
		Math::BarycentricCoords& coords, float& t, Math::Vector3& pt, bool isRay = false);

	inline bool IntersectRayTriangle(const Math::Vector3& p, const Math::Vector3& q, // the segment
		const Math::Vector3& a, const Math::Vector3& b, const Math::Vector3& c, // the triangle
		Math::BarycentricCoords& coords, float& t, Math::Vector3& pt)
	{
		return IntersectSegmentTriangle(p, q, a, b, c, coords, t, pt, true);
	}

	bool IntersectSweptSphereTriangle(const Math::Vector3& c, float radius, const Math::Vector3& v, // the swept sphere
		const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, // the triangle
		Math::Vector3& pos, Math::Vector3& normal, float& dist, Math::BarycentricCoords& coords);

} // namespace Geometry

#endif // COLLISION3D_H