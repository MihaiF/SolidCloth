#pragma once

#include <Math/Vector3.h>

namespace Geometry
{
	// TODO: templated
	struct Aabb3
	{
		Math::Vector3 min, max;
		Aabb3() : min(1e7f), max(-1e7f) { } // init to an invalid box that can be grown through min/max operations
		Aabb3(const Math::Vector3& a, const Math::Vector3& b) : min(a), max(b) { }
		Math::Vector3 GetExtent() const { return max - min; }
		Math::Vector3 GetHalfExtent() const { return 0.5f * (max - min); }
		Math::Vector3 GetCenter() const { return 0.5f * (max + min); }
		void Add(const Aabb3& box)
		{
			min = vmin(min, box.min);
			max = vmax(max, box.max);
		}
		void Add(const Math::Vector3& v)
		{
			min = vmin(min, v);
			max = vmax(max, v);
		}
		void Extrude(float e)
		{
			Math::Vector3 ext(e);
			min -= ext;
			max += ext;
		}

		// code below taken from G3
		float DistanceSquared(Math::Vector3 v)
		{
			float dx = (v.x < min.x) ? min.x - v.x : (v.x > max.x ? v.x - max.x : 0);
			float dy = (v.y < min.y) ? min.y - v.y : (v.y > max.y ? v.y - max.y : 0);
			float dz = (v.z < min.z) ? min.z - v.z : (v.z > max.z ? v.z - max.z : 0);
			return dx * dx + dy * dy + dz * dz;
		}

		float Distance(Math::Vector3 v)
		{
			return sqrtf(DistanceSquared(v));
		}

		float DistanceSquared(Aabb3 box2)
		{
			// compute lensqr( max(0, abs(center1-center2) - (extent1+extent2)) )
			float delta_x = fabsf((box2.min.x + box2.max.x) - (min.x + max.x))
				- ((max.x - min.x) + (box2.max.x - box2.min.x));
			if (delta_x < 0)
				delta_x = 0;
			float delta_y = fabsf((box2.min.y + box2.max.y) - (min.y + max.y))
				- ((max.y - min.y) + (box2.max.y - box2.min.y));
			if (delta_y < 0)
				delta_y = 0;
			float delta_z = fabsf((box2.min.z + box2.max.z) - (min.z + max.z))
				- ((max.z - min.z) + (box2.max.z - box2.min.z));
			if (delta_z < 0)
				delta_z = 0;
			return 0.25f * (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
		}

		float Distance(Aabb3 box2)
		{
			return sqrtf(DistanceSquared(box2));
		}

	};

	inline bool AabbOverlap3D(const Aabb3& bounds1, const Aabb3& bounds2)
	{
		if (bounds1.max.X() < bounds2.min.X())
			return false;
		if (bounds1.max.Y() < bounds2.min.Y())
			return false;
		if (bounds1.max.Z() < bounds2.min.Z())
			return false;
		if (bounds2.max.X() < bounds1.min.X())
			return false;
		if (bounds2.max.Y() < bounds1.min.Y())
			return false;
		if (bounds2.max.Z() < bounds1.min.Z())
			return false;
		return true;
	}

	// TODO: templated version
	inline bool PointInAabb3D(const Math::Vector3& min, const Math::Vector3& max, const Math::Vector3& point)
	{
		return !(point.X() < min.X() || point.X() > max.X()
			|| point.Y() < min.Y() || point.Y() > max.Y()
			|| point.Z() < min.Z() || point.Z() > max.Z());
	}

	inline bool TestSegmentAABB(const Math::Vector3& p0, const Math::Vector3& p1, const Aabb3& b)
	{
		Math::Vector3 c = (b.min + b.max) * 0.5f;
		Math::Vector3 e = b.max - c;
		Math::Vector3 m = (p0 + p1) * 0.5f;
		Math::Vector3 d = p1 - m;
		m = m - c;
		float adx = fabsf(d.X());
		if (fabs(m.X()) > e.X() + adx) return false;
		float ady = fabsf(d.Y());
		if (fabs(m.Y()) > e.Y() + ady) return false;
		float adz = fabsf(d.Z());
		if (fabs(m.Z()) > e.Z() + adz) return false;

		const float eps = 1e-6f;
		adx += eps;
		ady += eps;
		adz += eps;
		if (fabs(m.Y() * d.Z() - m.Z() * d.Y()) > e.Y() * adz + e.Z() * ady) return false;
		if (fabs(m.Z() * d.X() - m.X() * d.Z()) > e.X() * adz + e.Z() * adx) return false;
		if (fabs(m.X() * d.Y() - m.Y() * d.X()) > e.X() * ady + e.Y() * adx) return false;

		return true;
	}

	inline bool IntersectRayAABB(const Math::Vector3& p, const Math::Vector3& d, const Aabb3& a, float& tmin, Math::Vector3& q)
	{
		tmin = 0.f;
		float tmax = 1e10f;
		const float eps = 1e-7f;
		for (int i = 0; i < 3; i++)
		{
			if (abs(d[i]) < eps)
			{
				if (p[i] < a.min[i] || p[i] > a.max[i])
					return false;
			}
			else
			{
				float ood = 1.f / d[i];
				float t1 = (a.min[i] - p[i]) * ood;
				float t2 = (a.max[i] - p[i]) * ood;
				if (t1 > t2)
					std::swap(t1, t2);
				tmin = std::max(tmin, t1);
				tmax = std::min(tmax, t2);
				if (tmin > tmax)
					return false;
			}
		}
		q = p + d * tmin;
		return true;
	}

} // namespace Geometry