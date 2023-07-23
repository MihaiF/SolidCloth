#include <Geometry/Collision3D.h>
#include <Engine/Utils.h>
#include <Engine/xml.h>
#include "Mesh.h"
//#include <Geometry/Skeleton.h>

#ifndef ANDROID_NDK

#include <assimp/cimport.h>        // Plain-C interface
#include <assimp/scene.h>          // Output data structure
#include <assimp/postprocess.h>    // Post processing flags

#endif

#include <fstream>
#include <map>

uint32 crc32(uint32 crc, const void* buf, uint32 size);

namespace Geometry
{
	void CreateMesh(const struct aiScene* sc, const struct aiNode* nd, Mesh& myMesh,
		const Vector3& offset, float scale, bool flipYZ, const aiMatrix4x4& worldMat)
	{
#ifndef ANDROID_NDK
		ASSERT(sc != NULL);

		// accumulate transforms across hierarchy
		aiMatrix4x4 mat = nd->mTransformation * worldMat;

		for (size_t n = 0; n < nd->mNumMeshes; ++n)
		{
			// found one
			const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

			size_t baseIdx = myMesh.vertices.size();
			for (size_t i = 0; i < mesh->mNumVertices; i++)
			{
				const aiVector3D& src = mesh->mVertices[i];
				aiVector3D t = /*nd->mTransformation * */src;
				Vector3 dst(t.x * scale, t.y * scale, t.z * scale);
				if (flipYZ)
					dst.Set(t.x * scale, t.z * scale, t.y * scale);
				myMesh.vertices.push_back(dst + offset);

				if (mesh->mNormals)
				{
					const aiVector3D& srcN = mesh->mNormals[i];
					Vector3 n(srcN.x, srcN.y, srcN.z);
					n.Normalize();
					myMesh.normals.push_back(n);
				}

				if (mesh->GetNumColorChannels() > 0 && mesh->HasVertexColors(0))
				{
					Vector3 col(mesh->mColors[0][i].r, mesh->mColors[0][i].g, mesh->mColors[0][i].b);
					myMesh.colors.push_back(col);
				}

				if (mesh->GetNumUVChannels() > 0 && mesh->HasTextureCoords(0))
				{
					Vector2 uv(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
					myMesh.uvs.push_back(uv);
				}
			}

			for (size_t t = 0; t < mesh->mNumFaces; ++t)
			{
				const struct aiFace* face = &mesh->mFaces[t];
				for (size_t i = 0; i < face->mNumIndices; i += 3)
				{
					size_t index0 = face->mIndices[i] + baseIdx;
					size_t index1 = face->mIndices[i + 1] + baseIdx;
					size_t index2 = face->mIndices[i + 2] + baseIdx;

					//Need this to avoid back face culling issue.
					if (flipYZ) {
						myMesh.indices.push_back((uint32)index0);
						myMesh.indices.push_back((uint32)index2);
						myMesh.indices.push_back((uint32)index1);
					}
					else {
						myMesh.indices.push_back((uint32)index0);
						myMesh.indices.push_back((uint32)index1);
						myMesh.indices.push_back((uint32)index2);
					}
				}
			}

			//return;
		}

		// recurse if no mesh found
		for (size_t n = 0; n < nd->mNumChildren; ++n)
		{
			CreateMesh(sc, nd->mChildren[n], myMesh, offset, scale, flipYZ, mat);
		}
#endif
	}

	bool LoadMesh(const char* path, Mesh& mesh, const Vector3& offset, float scale, bool flipYZ)
	{
		const aiScene* scene = aiImportFile(path, aiProcess_JoinIdenticalVertices | aiProcess_Triangulate);
		if (scene == NULL)
			return false;
		mesh.indices.clear();
		mesh.vertices.clear();
		CreateMesh(scene, scene->mRootNode, mesh, offset, scale, flipYZ, aiMatrix4x4());
		return true;
	}

	void CreatePlane(Mesh& mesh)
	{
		const float size = 200;
		mesh.vertices.push_back(Vector3(size, 0, -size));
		mesh.vertices.push_back(Vector3(-size, 0, -size));
		mesh.vertices.push_back(Vector3(-size, 0, size));
		mesh.vertices.push_back(Vector3(size, 0, size));

		mesh.indices.push_back(0);
		mesh.indices.push_back(1);
		mesh.indices.push_back(2);
		mesh.indices.push_back(0);
		mesh.indices.push_back(2);
		mesh.indices.push_back(3);
	}

	void CreateBox(Mesh& mesh)
	{
		const float size = 200;
		mesh.vertices.push_back(Vector3(size, size, -size));
		mesh.vertices.push_back(Vector3(-size, size, -size));
		mesh.vertices.push_back(Vector3(-size, size, size));
		mesh.vertices.push_back(Vector3(size, size, size));

		mesh.vertices.push_back(Vector3(size, -size, -size));
		mesh.vertices.push_back(Vector3(-size, -size, -size));
		mesh.vertices.push_back(Vector3(-size, -size, size));
		mesh.vertices.push_back(Vector3(size, -size, size));

		mesh.indices.push_back(0);
		mesh.indices.push_back(1);
		mesh.indices.push_back(2);
		mesh.indices.push_back(0);
		mesh.indices.push_back(2);
		mesh.indices.push_back(3);

		mesh.indices.push_back(4);
		mesh.indices.push_back(6);
		mesh.indices.push_back(5);
		mesh.indices.push_back(4);
		mesh.indices.push_back(7);
		mesh.indices.push_back(6);

		mesh.indices.push_back(0);
		mesh.indices.push_back(4);
		mesh.indices.push_back(5);
		mesh.indices.push_back(0);
		mesh.indices.push_back(5);
		mesh.indices.push_back(1);

		mesh.indices.push_back(3);
		mesh.indices.push_back(6);
		mesh.indices.push_back(7);
		mesh.indices.push_back(3);
		mesh.indices.push_back(2);
		mesh.indices.push_back(6);

		mesh.indices.push_back(1);
		mesh.indices.push_back(5);
		mesh.indices.push_back(6);
		mesh.indices.push_back(1);
		mesh.indices.push_back(6);
		mesh.indices.push_back(2);

		mesh.indices.push_back(0);
		mesh.indices.push_back(7);
		mesh.indices.push_back(4);
		mesh.indices.push_back(0);
		mesh.indices.push_back(3);
		mesh.indices.push_back(7);
	}

	void CreateTetrahedron(Mesh& mesh, float r)
	{
		const float h = r / 3;
		const float d = 2 * sqrtf(2.f) * r / 3;
		const float hl = sqrtf(2.f / 3.f) * r;
		const float k = d / 2;

		mesh.vertices.push_back(Vector3(0, r, 0)); // A 0
		mesh.vertices.push_back(Vector3(-hl, -h, k)); // B 1
		mesh.vertices.push_back(Vector3(hl, -h, k)); // C 2
		mesh.vertices.push_back(Vector3(0, -h, -d)); // D 3

		mesh.indices.push_back(0);
		mesh.indices.push_back(1);
		mesh.indices.push_back(2);

		mesh.indices.push_back(0);
		mesh.indices.push_back(2);
		mesh.indices.push_back(3);

		mesh.indices.push_back(0);
		mesh.indices.push_back(3);
		mesh.indices.push_back(1);

		mesh.indices.push_back(1);
		mesh.indices.push_back(3);
		mesh.indices.push_back(2);
	}

	void Subdivide(Mesh& mesh, uint16 i1, uint16 i2, uint16 i3, float r)
	{
		const Vector3& a = mesh.vertices[i1];
		const Vector3& b = mesh.vertices[i2];
		const Vector3& c = mesh.vertices[i3];

		Vector3 m = 0.5f * (a + b);
		Vector3 n = 0.5f * (b + c);
		Vector3 p = 0.5f * (a + c);

		m.Scale(r / m.Length());
		n.Scale(r / n.Length());
		p.Scale(r / p.Length());

		// TODO: don't duplicate edge half vertices
		uint16 i4 = mesh.AddVertex(m);
		uint16 i5 = mesh.AddVertex(n);
		uint16 i6 = mesh.AddVertex(p);

		mesh.AddTriangle(i1, i4, i6);
		mesh.AddTriangle(i4, i2, i5);
		mesh.AddTriangle(i4, i5, i6);
		mesh.AddTriangle(i6, i5, i3);
	}

	void CreateSphere(Mesh& mesh, float r, int level)
	{
		Mesh tet;
		CreateTetrahedron(tet, r);

		mesh.vertices.insert(mesh.vertices.begin(), tet.vertices.begin(), tet.vertices.end());

		for (int iter = 0; iter < level; iter++)
		{
			mesh.indices.clear();
			for (size_t i = 0; i < tet.indices.size(); i += 3)
			{
				Subdivide(mesh, tet.indices[i], tet.indices[i + 1], tet.indices[i + 2], r);
			}
			tet = mesh;
		}
	}

	std::string TrimSpaces(const std::string& input) 
	{
		size_t start = 0;
		while (start < input.size() && input[start] == ' ') 
			start++;
		size_t end = input.size();
		while (end > start && (input[end - 1] == ' ' || input[end - 1] == '\n' || input[end - 1] == '\r')) 
			end--;
		return input.substr(start, end - start);
	}

	std::vector<std::string> TokenSplit(const std::string& input, const char* separators)
	{
		std::vector<std::string> result;
		size_t curr = 0;
		size_t found = 0;
		while ((found = input.find_first_of(separators, curr)) != std::string::npos)
		{
			std::string token = input.substr(curr, found - curr);
			token = TrimSpaces(token);
			//if (token.size() > 0)
				result.push_back(token);
			curr = found + 1;
		}
		std::string token = input.substr(curr);
		token = TrimSpaces(token);
		if (token.size() > 0)
			result.push_back(token);

		return result;
	}

	struct VertexKey
	{
		uint32 vertex, normal, uv;

		const bool operator < (const VertexKey& other) const
		{
			// lexigographic order
			if (vertex == other.vertex)
			{
				if (normal == other.normal)
				{
					return uv < other.uv;
				}
				return normal < other.normal;
			}
			return vertex < other.vertex;
		}
	};

	bool LoadMeshFromObj(const char* path, Mesh& mesh, bool naive)
	{
		mesh.Clear();

		FILE* ptr_file;
		char buf[1000];

		errno_t ret = fopen_s(&ptr_file, path, "r");
		if (ret != 0)
		{
			printf("Couldn't open file %s\n", path);
			return false;
		}

		int nod = 0;

		std::vector<Vector3> vertices;
		std::vector<Vector3> normals;
		std::vector<Vector2> uvs;
		std::vector<uint32> indicesV;
		std::vector<uint32> indicesN;
		std::vector<uint32> indicesT;

		std::map<VertexKey, uint32> map;
		std::vector<uint32> uniqueIdx;
		
		while (fgets(buf, 1000, ptr_file) != NULL)
		{
			char* ptr = buf;
			char* word = buf + 2;

			if (buf[0] == 'v')
			{
				if (buf[1] != 't')
				{
					// read vertex/normal
					Vector3 vec;
					int counter = 0;
					while (ptr = strchr(word, ' '))
					{
						if (ptr == word)
						{
							word++;
							continue;
						}
						*ptr = '\0';
						float x = (float)atof(word);
						vec[counter] = x;
						counter++;
						word = ptr + 1;
					}
					vec[2] = (float)atof(word);
					if (buf[1] == 'n')
						normals.push_back(vec);
					else
						vertices.push_back(vec);
				}
				else
				{
					// read UV coordinate
					Vector2 vec;
					int counter = 0;
					while (ptr = strchr(word, ' '))
					{
						if (ptr == word)
						{
							word++;
							continue;
						}
						*ptr = '\0';
						float x = (float)atof(word);
						vec[counter] = x;
						counter++;
						word = ptr + 1;
					}
					vec[1] = (float)atof(word);
					uvs.push_back(vec);
				}
			}

			if (buf[0] == 'f')
			{
				// read face
				std::string line(buf);
				auto face = TokenSplit(line, " ");
				ASSERT(face[0] == "f");
				for (int i = 0; i < 3; i++)
				{
					std::string str(face[i + 1]);
					auto tokens = TokenSplit(str, "/");

					uint32 v = 0;
					uint32 t = 0;
					uint32 n = 0;

					// vertex index
					v = std::stoi(tokens[0]) - 1;
					indicesV.push_back(v);

					if (tokens.size() > 1)
					{
						// UV
						if (tokens[1] != "")
						{
							t = std::stoi(tokens[1]) - 1;
							indicesT.push_back(t);
						}
						// normal
						if (tokens.size() > 2 && tokens[2] != "")
						{
							n = std::stoi(tokens[2]) - 1;
							indicesN.push_back(n);
						}
					}

					VertexKey key = { v, n, t };
					std::map<VertexKey, uint32>::iterator iter = map.find(key);
					uint32 idx = (uint32) map.size();
					if (iter != map.end())
					{
						idx = iter->second;
					}
					else
					{
						map.insert({ key, idx });
					}
					uniqueIdx.push_back(idx);
				}
			}
		}

		Printf("%d vertices, %d normals, %d uvs\n", vertices.size(), normals.size(), uvs.size());

		mesh.Clear();
		if (naive)
		{
			// naive copy
			mesh.vertices = vertices;
			mesh.normals = normals;
			mesh.indices = indicesV;
		}
		else
		{
			mesh.vertices.resize(map.size());
			mesh.normals.resize(map.size());
			mesh.uvs.resize(map.size());
			for (auto pair : map)
			{
				uint32 idx = pair.second;
				mesh.vertices[idx] = vertices[pair.first.vertex];
				mesh.normals[idx] = normals[pair.first.normal];
				mesh.uvs[idx] = uvs[pair.first.uv];
			}
			mesh.indices = uniqueIdx;
		}

		fclose(ptr_file);
		return true;
	}

	bool SaveMeshToObj(const char* path, const Mesh& mesh)
	{
		std::ofstream file;
		file.open(path, std::ios::out);
		file << "# File created by SolidSim" << std::endl;
		for (size_t i = 0; i < mesh.vertices.size(); i++)
		{
			const Vector3& v = mesh.vertices[i];
			file << "v " << v.x << " " << v.y << " " << v.z << std::endl;
		}
		for (size_t i = 0; i < mesh.normals.size(); i++)
		{
			const Vector3& v = mesh.normals[i];
			file << "vn " << v.x << " " << v.y << " " << v.z << std::endl;
		}
		for (size_t i = 0; i < mesh.GetNumTriangles(); i++)
		{
			int i0 = mesh.indices[i * 3] + 1;
			int i1 = mesh.indices[i * 3 + 1] + 1;
			int i2 = mesh.indices[i * 3 + 2] + 1;
			file << "f " << i0 << "//" << i0 << " " << i1 << "//" << i1 << " " << i2 << "//" << i2 << std::endl;
		}
		return true;
	}

}