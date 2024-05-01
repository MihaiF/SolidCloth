#pragma once // TODO: guard

#include "Geometry/Aabb3.h"
#include "Math/Quaternion.h"

#include <vector>

// *** API ******************
typedef void* scHandle;

struct scClothPatch
{
	void* ptr = nullptr;
	// TODO: inherit from Handle/Pointer class
	// TODO: destructor with memory free
	// TODO: operator ->
};

struct scCollisionWorld
{
	void* ptr = nullptr;
};

struct scMesh
{
	void* ptr = nullptr;
};

struct scModel
{
	void* ptr = nullptr;
};

enum scCollFlags
{
	scCollisionFlagVertices = 1,
	scCollisionFlagTriangles = 2,
	scCollisionFlagSelf = 4,
	scCollisionFlagWalls = 8,
	scCollisionFlagEdges = 16,
};

enum scCollidableType
{
	scCollidableTypeInvalid,
	scCollidableTypeWalls,
	scCollidableTypeSlab,
	scCollidableTypeSphere,
	scCollidableTypeMesh,
	scCollidableTypeBox,
	scCollidableTypeCapsule,
	scCollidableTypeSDF,
};


namespace Geometry
{
	class SDF;
	class Mesh;
	struct AabbTree;
}

#define SCEXP __declspec(dllexport)

extern "C"
{

	// collision world
	SCEXP scCollisionWorld scCreateCollisionWorld();
	SCEXP void scFreeCollisionWorld(scCollisionWorld collWorld);
	SCEXP void scClearCollidables(scCollisionWorld collWorld);
	SCEXP int scGetNumCollidables(scCollisionWorld collWorld);
	SCEXP void scAddCollidable(scCollisionWorld collWorld, scHandle collidable);
	SCEXP scHandle scGetCollidable(scCollisionWorld collWorld, int i);

	// colidables
	SCEXP scHandle scCreateWalls(Geometry::Aabb3 box);
	SCEXP scHandle scCreateSphere(Math::Vector3 center, float radius);
	SCEXP scHandle scCreateCapsule(Math::Vector3 c, float rad, float halfHeight, Math::Quaternion quat);
	SCEXP scHandle scCreateSDF(Geometry::SDF* s);
	SCEXP scHandle scCreateMesh(Geometry::Mesh* m);

	SCEXP int scGetCollidableType(scHandle collidable);
	SCEXP Geometry::AabbTree* scGetMeshTree(scHandle collMesh);
	SCEXP void scGetWallsInfo(scHandle walls, Geometry::Aabb3& box);
	SCEXP void scGetSphereInfo(scHandle sphere, float& radius, Math::Vector3& center);
	SCEXP void scGetCapsuleInfo(scHandle capsule, float& radius, float& halfHeight, Math::Vector3& center, Math::Quaternion& rot); // TODO: struct


	// cloth patch
	SCEXP scClothPatch scCreateClothPatch(scCollisionWorld collWorld);
	SCEXP void scFreeClothPatch(scClothPatch patch);
	SCEXP scModel scGetClothModel(scClothPatch patch);
	SCEXP scMesh scGetClothMesh(scClothPatch patch);
	SCEXP void scInitCloth(scClothPatch patch, int divX, int divY, float inc, Math::Vector3 offset, bool horizontal, bool attached);

	SCEXP void scStepCloth(scClothPatch patch, float dt);
	SCEXP void scUpdateClothMesh(scClothPatch patch);
	SCEXP void scComputeStrainMap(scClothPatch patch, std::vector<Math::Vector3>& colors);

	// cloth model
	SCEXP int scGetNumParticles(scModel model);
	SCEXP void scGetParticlePos(scModel model, int i, Math::Vector3& pos); // TODO: info?
	SCEXP float scGetParticleInvMass(scModel model, int i);

	SCEXP int scGetNumTris(scModel model);
	SCEXP void scGetTriangle(scModel model, int i, int& i1, int& i2, int& i3);
	SCEXP void scGetTriangleInfo(scModel model, int i, float& invDet, Math::Vector2& du, Math::Vector2& dv, Math::Vector2& s); // TODO: struct

	SCEXP int scGetNumQuads(scModel model);
	SCEXP void scGetQuad(scModel model, int i, int& i1, int& i2, int& i3, int& i4);

	SCEXP int scGetNumLinks(scModel model);
	SCEXP void scGetLink(scModel model, int i, int& i1, int& i2);

	SCEXP int scGetNumContacts(scModel model);
	SCEXP void scGetContact(scModel model, int i, Math::Vector3& point, Math::Vector3& normal);
	SCEXP int scGetNumTriContacts(scModel model);
	SCEXP void scGetTriContact(scModel model, int i, Math::Vector3& point, Math::Vector3& normal);

	SCEXP int scGetCollisionFlags(scModel model);
	SCEXP void scSetCollisionFlags(scModel model, int val);
	SCEXP float scGetThickness(scModel model);
	SCEXP void scSetThickness(scModel model, float val);
	SCEXP float scGetTolerance(scModel model);
	SCEXP void scSetTolerance(scModel model, float val);
	SCEXP float scGetFriction(scModel model);
	SCEXP void scSetFriction(scModel model, float val);
	SCEXP int scGetNumIterations(scModel model);
	SCEXP void scSetNumIterations(scModel model, int val);
	SCEXP bool scGetUseFEM(scModel model);
	SCEXP void scSetUseFEM(scModel model, bool val);

	SCEXP Geometry::AabbTree* scGetClothTree(scModel model);

	SCEXP void scAddMouseSpring(scModel model, int i1, int i2, int i3, Math::Vector3 coords, Math::Vector3 p);
	SCEXP void scUpdateMouseSpring(scModel model, Math::Vector3 p);
	SCEXP void scRemoveMouseSpring(scModel model);

	// mesh
	SCEXP int scGetMeshVertexCount(scMesh mesh);
	SCEXP int scGetMeshIndexCount(scMesh mesh);
	SCEXP Math::Vector3* scGetMeshNormalBuffer(scMesh mesh);
	SCEXP Math::Vector3* scGetMeshVertexBuffer(scMesh mesh);
	SCEXP unsigned int* scGetMeshIndexBuffer(scMesh mesh);
}

// **************************

