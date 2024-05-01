#ifndef GRAPHICS3D_H
#define GRAPHICS3D_H

#include <Graphics2D/Graphics.h>
#include <Engine/Types.h>
#include <Math/Matrix4.h>
#include <Math/Vector2.h>
#include <Graphics3D/Camera.h>
#include <Graphics3D/OpenGLSData.h>
#include <Graphics2D/Texture.h>

#if (RENDERER == GDIPLUS)
#define RENDERER3D OPENGL
#else
#define RENDERER3D RENDERER
#endif

enum RenderMode
{
	RM_INVISIBLE,
	RM_SHADED,
	RM_WIREFRAME_ON_SHADED,
	RM_WIREFRAME,
	RM_PRIMITIVE_IDS,
};

class Graphics3D
{
public:
	enum CullingMode
	{
		CULL_FRONT = GL_FRONT,
		CULL_BACK = GL_BACK
	};

public: // TODO: private
#ifdef _WIN32
	HWND hWnd;
#endif
	int w, h;

	Camera camera;
	int mouseX, mouseY;
	bool mousePressed;

	float fov, n, f;

#if defined(_WIN32) && (RENDERER3D == OPENGL)
	// TODO: rename
	HGLRC hGLRC;
	HDC hDC;
	
	GLuint sphList;
#endif

#if !defined(FIXED_PIPELINE) && (RENDERER3D == OPENGL)
	OpenGLSData mGLS; // TOD: make PIMPL and private
#endif
	Math::Vector3 color;
	Math::Vector3 lightPos;
	Math::Vector3 lightDir;

	int mRenderMode;

public:
	Graphics3D() : mousePressed(false), fov(60), n(0.1f), f(10000), w(10), h(10) { }
	bool Init(HWND window);
	void DeInit();
	void Resize(int width, int height);
	void Draw();

	void BeginDraw();		
	void EndDraw();
	
	void DrawAxes();
	void MouseDown(int x, int y);
	void MouseMove(int x, int y);
	void MouseUp(int x, int y);
	void MouseWheel(float delta);

	void DrawPlane(const Math::Vector3& plane, float scale, Texture* tex = nullptr);
	int DrawSphere(const Math::Vector3& pos, float radius);
	void DrawSphereTex(const Math::Vector3& pos, float radius, const Math::Matrix3& rot, Texture* tex);
	void DrawTriangle(const Math::Vector3& a, const Math::Vector3& b, const Math::Vector3& c, bool fill);
	void DrawCube(const Math::Vector3& pos, const Math::Vector3& extent, const Math::Matrix3& rot);
	void DrawWireCube(const Math::Vector3& pos, const Math::Vector3& extent, const Math::Matrix3& rot);
	void DrawLine(const Math::Vector3& a, const Math::Vector3& b);
	void DrawLines(const std::vector<Math::Vector3>& points);
	void DrawTetrahedronEdgeAsBBCurve(const Math::Vector3& c1, const Math::Vector3& c2, const size_t nodesperedge, const Math::Vector3* edge);
	void DrawTetrahedron(const Math::Vector3& v0, const Math::Vector3& v1, const Math::Vector3& v2, 
		const Math::Vector3& v3);
	void DrawCapsule(const Math::Vector3& pos, float radius, float height, const Math::Matrix3& rot);

	int DrawMesh(const RenderMesh& mesh);
	int DrawMesh(const std::vector<Math::Vector3>& vertices, const std::vector<Math::Vector3>& normals, 
		const std::vector<uint32>& indices, const Math::Matrix4* model = nullptr);
	int DrawMesh(const std::vector<Math::Vector3>& vertices, const std::vector<Math::Vector3>& normals, 
		const std::vector<Math::Vector3>& colors, const std::vector<uint32>& indices, const Math::Matrix4* model = nullptr);
	int DrawMesh(const std::vector<Math::Vector3>& vertices, const std::vector<Math::Vector3>& normals, 
		const std::vector<uint32>& indices, const std::vector<Math::Vector2>& uvs, Texture* tex, const Math::Matrix4* model = nullptr);

	void SetColor(float r, float g, float b);

	void Viewport();
	void SetContext();

	void SetShadowsActive(bool val);
	void ViewportShadow();
	bool IsShadowPass() const;
	bool IsPickPass() const;

	const Math::Vector3& GetLightPos() const { return lightPos; }
	void SetLightPos(const Math::Vector3& v);
	const Math::Vector3& GetLightDir() const { return lightDir; }
	void SetLightDir(const Math::Vector3& v) { lightDir = v; }
	void SetFlipNormals(bool val);
	void SetFlags(int val);
	void ResetFlags();
	int GetFlags() const;
	void SetCulling(int val);

	void GetScreenCoords(const Math::Vector3& p, float& sx, float& sy);

	void SetRenderMode(int val);

	Math::Vector3 ComputeMousePoint(int x, int y)
	{
		Math::Vector3 mousePt;
		float width = w;
		float height = h;
		mousePt.Set(-1.f + 2.f * (float)x / width, 1.f - 2.f * (float)y / height, -1.f);
		float d = tanf(RADIAN(fov) * 0.5f);
		float aspect = width / height;
		mousePt.X() *= d * aspect;
		mousePt.Y() *= d;

		Math::Matrix4 mat = mGLS.GetView();
		mat = mat.GetInverseTransform();
		mousePt = mat.Transform(mousePt);

		return mousePt;
	}

};

#ifdef FIXED_PIPELINE
inline void Graphics3D::SetColor(float r, float g, float b)
{
	color.Set(r, g, b);
	glColor4f(r, g, b, 1.f);
}
#endif

#endif // GRAPHICS3D_H
