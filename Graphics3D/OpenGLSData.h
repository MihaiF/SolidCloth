#pragma once

#include <Math/Vector3.h>
#include <Math/Matrix3.h>
#include <Math/Matrix4.h>
#include <Graphics2D/Texture.h>

#include <vector>

enum ShaderFlags
{
	NONE = 0,
	VERTEX_COLORS = 1,
	OREN_NAYAR = 2,
	TEXTURED = 4,
	MONOCHROME = 8,
	WIREFRAME = 16,
	SHADOWS = 32,
};

struct NormalProgram
{
	GLuint programID;
	GLint MatrixID, ModelMatrixID;
	GLint eyePosID;
	GLint LightPosID;
	GLint LightDirID;
	GLint diffColorID, specColorID;
	GLint glossID;
	GLint flipID;
	GLint flagsID;
	GLint winScaleId;
	GLint shadowVPMatrixID;
};

struct PosNormal // vertex format
{
	Vector3 v, n;
};

class RenderMesh
{
public:
	RenderMesh() { }
	~RenderMesh() { Destroy(); }

	void Create();
	void Destroy();

	void SetVertexData(const std::vector<Vector3>& vertices);
	void SetNormalData(const std::vector<Vector3>& normals);
	void SetColorData(const std::vector<Vector3>& colors);
	void SetUVData(const std::vector<Vector2>& uvs);
	void SetIndexData(const std::vector<uint32>& indices);
	void SetTransform(const Matrix4* mat) { mModel = mat; }
	void SetTexture(const Texture* tex) { mTexture = tex; }

	void PrepareVBOAttributes(bool hasColors, bool hasUVs) const;

	void CreateVAO(bool hasColors, bool hasUVs);

	void Draw(int modelID, int mapID, bool useVAO, bool hasColors) const;

private:
	size_t mNumElements;

	const Matrix4* mModel = nullptr;

	GLuint mVertexVBO = 0;
	GLuint mNormalVBO = 0;
	GLuint mIndexVBO = 0;
	GLuint mColorVBO = 0;
	GLuint mUvVBO = 0;

	GLuint mVAO = 0;

	const Texture* mTexture = nullptr;

	friend class OpenGLSData;
};

class OpenGLSData
{
public:
	bool Init();
	void Resize(int width, int height);
	void PrepareForDraw(const Math::Vector3& lightPos, const Math::Vector3& lightDir, const Vector3& eye);
	void PrepareForShadow();

	void DrawAxes();
	void DrawPlane(const Vector3& pos, float scale, Texture* tex);
	int DrawSphere(const Math::Vector3& pos, float radius);
	void DrawSphereTex(const Math::Vector3& pos, float radius, const Matrix3& rot, Texture* tex);
	void DrawTriangle(const Vector3& a, const Vector3& b, const Vector3& c, bool fill);
	void DrawCube(const Vector3& pos, const Vector3& ext, const Matrix3& rot);
	void DrawCapsule(const Vector3& pos, float radius, float height, const Matrix3& rot);
	void DrawLine(const Vector3& a, const Vector3& b);
	void DrawLines(std::vector<Vector3> points);
	int DrawMesh(const RenderMesh& mesh, bool useVAO = true, bool hasColors = false);
	int DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<uint32>& indices, const Matrix4* model);
	int DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<Vector3>& colors, const std::vector<uint32>& indices, const Matrix4* model);
	int DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<uint32>& indices, const std::vector<Vector2>& uvs, Texture* tex, const Matrix4* model);
	void DrawTetrahedron(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& v3);
	void DrawTetrahedronEdgeAsBBCurve(const Vector3& c1, const Vector3& c2, const size_t nodesperedge, const Vector3* edge);

	void CreateSphere(int divsTheta = 10, int divsPhi = 10);
	void CreatePlane(int divsX = 10, int divsY = 10);
	void CreateCapsule(int divsTheta = 10, int divsPhi = 5, float radius = 1.0f, float height = 1.0f);

	void SetColor(float r, float g, float b);
	void SetFlipNormals(bool val);
	void SetProgram(bool wireframe);
	void UsePickProgram();

	void SetFlags(int val);
	void ResetFlags();
	int GetFlags() const { return flags; }

	const Matrix4& GetProjection() const { return Projection; }
	void SetProjection(const Matrix4& mat) { Projection = mat; }

	const Matrix4& GetView() const { return View; }
	void SetView(const Matrix4& mat) { View = mat; }

	const Matrix4& GetViewShadow() const { return viewShadow; }
	void SetViewShadow(const Matrix4& mat) { viewShadow = mat; }

	Texture& GetShadowMap() { return shadowMap; }
	bool IsShadowPass() const { return isShadowPass; }

	bool GetShadowsActive() const { return shadowsActive; }
	void SetShadowsActive(bool val) { shadowsActive = val; }

	bool InitScreenMap();
	void DrawToFBO();
	void DrawToPickFBO();

	Texture& GetScreenMap() { return mScreenMap; }
	bool GetScreenFBOActive() const { return mScreenFBOActive; }
	void SetScreenFBOActive(bool val) { mScreenFBOActive = val; }

	bool InitPickMap();
	Texture& GetPickMap() { return mPickMap; }
	uint32 ReadPickPixel(unsigned int x, unsigned int y);
	bool GetPickFBOActive() const { return mPickFBOActive; }
	void SetPickFBOActive(bool val) { mPickFBOActive = val; }
	bool IsPickPass() const { return isPickPass; }

private:
	bool InitShadowMap();

	bool LoadNormalProgram();
	bool LoadWireframeProgram();
	bool LoadPickProgram();

private:
	int mWidth, mHeight;
	Matrix4 Projection, View;

	RenderMesh mMesh;
	GLuint vertexbuffer; // TODO: remove
	GLuint normalbuffer; // TODO: remove
	GLuint elementbuffer;
	GLuint colorbuffer;
	GLuint uvbuffer;

	NormalProgram program, wireframeProgram, pickProgram;
	NormalProgram* currProgram = nullptr;

	GLuint linesBuffer;

	RenderMesh mSphere;
	RenderMesh mPlane;
	RenderMesh mCapsule;

	int flags = 0;

	// shadow
	GLuint shadowProgramID;
	GLuint shadowFramebuffer;
	GLuint shadowModelID;
	bool isShadowPass = false;
	Texture shadowMap;
	Matrix4 viewShadow;
	Matrix4 projShadow;
	bool shadowsActive = false;

	Texture mScreenMap;
	GLuint mScreenFBO;
	bool mScreenFBOActive = false;

	int mDrawIndex = 0;
	GLuint mPickFBO = 0;
	Texture mPickMap;
	GLuint depthTexture = 0;
	bool mPickFBOActive = false;
	bool isPickPass = false;
};
