#ifndef ASSETS_H
#define ASSETS_H

namespace Geometry
{
	struct Mesh;

	class Skeleton;

	void CreateMesh(const struct aiScene* sc, const struct aiNode* nd, Geometry::Mesh& collMesh,
		const Vector3& offset, float scale, bool flipYZ);
	bool LoadMesh(const char* path, Geometry::Mesh& mesh, const Vector3& offset, float scale = 20, bool flipYZ = false);

	void LoadMeshFromOgreXml(const char* path, Geometry::Mesh& mesh);

	bool LoadMeshFromObj(const char* path, Mesh& mesh, bool naive = true);
	bool SaveMeshToObj(const char* path, const Mesh& mesh);

	bool LoadSkeleton(const char* path, Skeleton& skeleton, Mesh* mesh = nullptr);
}

#endif
