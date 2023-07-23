#ifndef COMMON_H
#define COMMON_H

#include <Math/Vector3.h>
#include <Math/Quaternion.h>
#include "Geometry/Collision3D.h"
#include <Geometry/Mesh.h>

namespace Geometry
{
	struct AabbTree;
	class SDF;
}

namespace Physics
{	
	const Math::Vector3 gravity(0, -9.8f, 0);

	enum CollidableType
	{
		CT_INVALID,
		CT_WALLS,
		CT_SLAB,
		CT_SPHERE,
		CT_MESH,
		CT_BOX,
		CT_CAPSULE,
		CT_SDF,
	};
	
	struct Collidable
	{
		Quaternion rot;
		Vector3 center;
		int mType;
		int mUserIndex; // body index
		int mGroupIndex;
		void* mGroup;
		float mTolerance;
		int mUserData;
		Collidable() : mType(CT_INVALID), mUserIndex(-1), mGroupIndex(-1), mUserData(0), mTolerance(0.0f), mGroup(nullptr) { } // TODO: only type param constructor
		virtual ~Collidable() { }
	};

	struct Walls : Collidable
	{
		Geometry::Aabb3 mBox;
		Walls() { mType = CT_WALLS; }
		Walls(const Geometry::Aabb3& box) : mBox(box) { mType = CT_WALLS; }
	};

	struct Slab : Walls
	{
		Slab() { mType = CT_SLAB; }
		Slab(const Geometry::Aabb3& box) : Walls(box) { mType = CT_SLAB; }
	};
		

	struct Sphere : Collidable
	{
		float radius;
		Sphere() : radius(0) { mType = CT_SPHERE; }
		Sphere(const Vector3& c, float r) : radius(r) 
		{
			mType = CT_SPHERE;
			center = c;
		}
	};

	struct Box: Collidable
	{
		Vector3 D; // dimensions; TODO: rename
		Geometry::Mesh mesh; // temp
		Box() { mType = CT_BOX; }
		// TODO: use the mesh one
		Geometry::Aabb3 GetAabb(const Quaternion& q, const Vector3& t) const
		{
			Vector3 v = qRotate(q, mesh.vertices[0]) + t;
			Vector3 v1 = v;
			Vector3 v2 = v;
			for (size_t i = 1; i < mesh.vertices.size(); i++)
			{
				v = qRotate(q, mesh.vertices[i]) + t;
				v1 = vmin(v1, v);
				v2 = vmax(v2, v);
			}
			return Geometry::Aabb3(v1, v2);
		}
	};

	struct Capsule : Collidable
	{
		float hh; // half height (along the y axis)
		float r; // cap radius
		Capsule() { mType = CT_CAPSULE; }

		Capsule(const Vector3& c, float rad, float halfHeight) : r(rad), hh(halfHeight)
		{
			mType = CT_CAPSULE;
			center = c;
		}

		Capsule(const Vector3& c, float rad, float halfHeight, const Quaternion& quat) : r(rad), hh(halfHeight)
		{
			mType = CT_CAPSULE;
			center = c;
			rot = quat;
		}
	};

	struct CollisionMesh : Collidable
	{
		struct Geometry::Mesh* mesh;
		Vector3 offset; // TODO: superseed by center
		Geometry::AabbTree* tree;
		bool invalidate;
		CollisionMesh(Geometry::Mesh* m) : mesh(m), tree(NULL), invalidate(false)
		{
			mType = CT_MESH;
		}
	};

	struct CollisionSDF : Collidable
	{
		Geometry::SDF* sdf;
		Geometry::Mesh isoMesh;

		CollisionSDF(Geometry::SDF* s) : sdf(s)
		{
			mType = CT_SDF;
		}
	};

}

#endif // COMMON_H