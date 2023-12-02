#ifndef GEOMETRY_MESH_H
#define GEOMETRY_MESH_H

#include "Collision3D.h"
#include <Engine/Types.h>
#include <Math/Matrix4.h>
#include <Math/Quaternion.h>
#include <Geometry/Aabb3.h>

namespace Geometry
{
	struct Mesh
	{
		struct Edge
		{
			Vector3 n;
			uint32 i1, i2; // vertex indices
			int t1, t2; // triangles indices (winged edge)
			//Aabb3 box; // hack for AABB tree based self collision
			int count; // multiplicity
			int origIdx;
			bool swapped = false; // whether the edge is aligned with T1 or T2
			Edge() : i1(0), i2(0), t1(-1), t2(-1), count(0) { }
			Edge(int a, int b) : i1(a), i2(b), t1(-1), t2(-1), count(1) { }
		};

		struct Triangle
		{
			uint32 e[3];
			Vector3 n;
		};

		struct BoneWeight
		{
			int id;
			float weight;
			BoneWeight() : id(-1), weight(0) { }
		};
		struct Weights
		{
			enum { MAX_WEIGHTS = 10 };
			BoneWeight bw[MAX_WEIGHTS];
			// TODO: operator[]
		};

		std::vector<Vector3> vertices;
		std::vector<uint32> indices;
		std::vector<Vector3> normals;
		std::vector<Vector3> tangents;
		std::vector<Vector3> bitangents;
		std::vector<Vector2> uvs;
		std::vector<Vector3> colors; // TODO: Vector4
		std::vector<Edge> edges;
		std::vector<Weights> weights;
		std::vector<Vector3> velocities; // remove?
		std::vector<double> pressures; //TODO: to be removed; should be "real" not double
		std::vector<std::vector<int>> vtxOneRings;
		std::vector<std::vector<int>> edgeOneRings;
		std::vector<std::vector<int>> triOneRings;
		std::vector<Triangle> triangles;

		size_t GetNumTriangles() const { return indices.size() / 3; }

		void Clear()
		{
			vertices.clear();
			indices.clear();
			normals.clear();
			tangents.clear();
			bitangents.clear();
			uvs.clear();
			colors.clear();
			edges.clear();
			weights.clear();
			velocities.clear();
			pressures.clear();
			vtxOneRings.clear();
			edgeOneRings.clear();
			triOneRings.clear();
			triangles.clear();
		}

		uint32 AddVertex(const Vector3& v, bool check = false)
		{
			if (check)
			{
				std::vector<Vector3>::iterator it = std::find_if(vertices.begin(), vertices.end(), [&v](const Vector3& t) { return std::abs(t.x - v.x) < 0.00001f && std::abs(t.y - v.y) < 0.00001f && std::abs(t.z - v.z) < 0.00001f; });
				if (it != vertices.end())
				{
					return uint32(std::distance(vertices.begin(), it));
				}
			}
			vertices.push_back(v);
			return (uint32)(vertices.size() - 1);
		}

		void AddTriangle(uint32 a, uint32 b, uint32 c, bool flip = false)
		{
			indices.push_back(a);
			if (!flip)
			{
				indices.push_back(b);
				indices.push_back(c);
			}
			else
			{
				indices.push_back(c);
				indices.push_back(b);
			}
		}

		void ComputeNormals(bool flip = false);

		void ConstructVertexBuffer();

		void Transform(const Matrix4& mat, bool calcVel)
		{
			velocities.resize(vertices.size());
			for (size_t i = 0; i < vertices.size(); i++)
			{
				Vector3 v = mat.Transform(vertices[i]);
				if (calcVel)
					velocities[i] = v - vertices[i];
				vertices[i] = v;
			}
		}

		Vector3 GetCentroid() const
		{
			Vector3 c;
			for (size_t i = 0; i < vertices.size(); i++)
			{
				c += vertices[i];
			}
			c.Scale(1.f / vertices.size());
			return c;
		}

		void GetBoundingSphere(Vector3& c, float& r)
		{
			c = GetCentroid();
			r = 0;
			for (size_t i = 0; i < vertices.size(); i++)
			{
				float len = (vertices[i] - c).Length();
				r = max(r, len);
			}
		}
		
		bool ConstructEdges();

		void ConstructVertexOneRings();

		void ConstructEdgeOneRings();

		void ConstructTriangleOneRings();

		Aabb3 GetAabb() const
		{
			if (vertices.empty())
				return Aabb3();
			Vector3 v = vertices[0];
			Vector3 v1 = v;
			Vector3 v2 = v;
			for (size_t i = 1; i < vertices.size(); i++)
			{
				v = vertices[i];
				v1 = vmin(v1, v);
				v2 = vmax(v2, v);
			}
			return Aabb3(v1, v2);
		}

		Aabb3 GetAabb(const Quaternion& q, const Vector3& t) const
		{
			Vector3 v = qRotate(q, vertices[0]) + t;
			Vector3 v1 = v;
			Vector3 v2 = v;
			for (size_t i = 1; i < vertices.size(); i++)
			{
				v = qRotate(q, vertices[i]) + t;
				v1 = vmin(v1, v);
				v2 = vmax(v2, v);
			}
			return Aabb3(v1, v2);
		}

		Aabb3 GetAabb(const Matrix3& R, const Vector3& t) const
		{
			Vector3 v = R * vertices[0] + t;
			Vector3 v1 = v;
			Vector3 v2 = v;
			for (size_t i = 1; i < vertices.size(); i++)
			{
				v = R * vertices[i] + t;
				v1 = vmin(v1, v);
				v2 = vmax(v2, v);
			}
			return Aabb3(v1, v2);
		}

		void Extrude(float val)
		{
			if (normals.empty())
				ComputeNormals();
			for (size_t i = 0; i < vertices.size(); i++)
			{
				vertices[i] += val * normals[i]; 
			}
		}

		size_t RemoveDuplicatedVertices();
		size_t RemoveDuplicatedNormals();

		void Transform(float scale, const Vector3& offset)
		{
			// first scale, then translate
			for (size_t i = 0; i < vertices.size(); i++)
				vertices[i] = scale * vertices[i] + offset;
		}
	};

	static inline bool CompareEdges(const Mesh::Edge& e1, const Mesh::Edge& e2)
	{
		return e1.i1 < e2.i1 || (e1.i1 == e2.i1 && e1.i2 < e2.i2);
	}

}
#endif // GEOMETRY_MESH_H