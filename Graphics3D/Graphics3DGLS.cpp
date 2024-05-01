#include <Engine/Base.h>
#include <Engine/Engine.h>

#include <Graphics3D/Graphics3D.h>
#if (RENDERER3D == OPENGL) && !defined(FIXED_PIPELINE)

#include <Engine/Profiler.h>
#include <Graphics2D/Texture.h> // TODO: not really 2D only

#ifdef USE_IMGUI
#include <imgui.h>
#include <imgui_impl_win32.h>
#include <imgui_impl_opengl3.h>
#endif

#include <string>
#include <vector>

using namespace Math;

bool Graphics3D::Init(HWND window)
{
	mousePressed = false;
	lightPos.Set(0, 400, 800);
	lightDir.Set(0, -1, 0);
	if (!mGLS.Init())
		return false;

#ifdef USE_IMGUI
	// ********** ImGui ********************

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	// Setup Dear ImGui style
	//ImGui::StyleColorsDark();
	ImGui::StyleColorsClassic();

	// Setup Platform/Renderer bindings
	ImGui_ImplWin32_Init(window);
	ImGui_ImplOpenGL3_Init("#version 130");
#endif

	return true;
}

void Graphics3D::Draw()
{
	PROFILE_SCOPE("Draw3D");

#ifdef USE_IMGUI
	// TODO: move completely to OpenGLSData and drop some functions
	// ******** ImGui *****************
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();
#endif // USE_IMGUI

	SetContext();

	if (mGLS.GetShadowsActive())
	{
		// draw shadow
		ViewportShadow();
		Engine::getInstance()->OnDraw3D();
		// revert to screen frame buffer
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	Viewport();
	Engine::getInstance()->OnDraw3D();

	if (mGLS.GetScreenFBOActive())
		mGLS.DrawToFBO();

	if (mGLS.GetPickFBOActive())
	{		
		mRenderMode = RM_PRIMITIVE_IDS;
		mGLS.DrawToPickFBO();
		mRenderMode = RM_SHADED;
	}

#ifdef USE_IMGUI

	Engine::getInstance()->OnDrawUI();

	// ImGui Rendering
	//ImGui::ShowDemoWindow();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif
}

bool Graphics3D::IsShadowPass() const
{
	return mGLS.IsShadowPass();
}

bool Graphics3D::IsPickPass() const
{
	return mGLS.IsPickPass();
}

void Graphics3D::SetLightPos(const Vector3& v)
{
	lightPos = v;
	mGLS.SetViewShadow(Matrix4::LookAt(lightPos, Vector3(0, 0, 0), Vector3(0, 1, 0)));
}

void Graphics3D::SetContext()
{
	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	glEnable(GL_CULL_FACE);

	CheckGlError();
}

void Graphics3D::DeInit()
{
	//wglMakeCurrent(NULL, NULL);
	//ReleaseDC(hWnd, hDC) ;
	//wglDeleteContext(hGLRC);

#ifdef USE_IMGUI
	// ImGui Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplWin32_Shutdown();
	if (ImGui::GetCurrentContext())
		ImGui::DestroyContext();
#endif
}

void Graphics3D::Resize(int width, int height)
{
	w = width;
	h = height;
	mGLS.Resize(width, height);
	mGLS.SetProjection(Matrix4::Perspective(fov, (float)w / (float)h, n, f)); // TODO: pay attention to z-buffer accuracy
}

void Graphics3D::SetFlipNormals(bool val)
{
	mGLS.SetFlipNormals(val);
}

void Graphics3D::SetFlags(int val)
{
	mGLS.SetFlags(val);
}

void Graphics3D::ResetFlags()
{
	mGLS.ResetFlags();
}

int Graphics3D::GetFlags() const
{
	return mGLS.GetFlags();
}

void Graphics3D::SetCulling(int val)
{
	glCullFace(val);
}

void Graphics3D::Viewport()
{
	glViewport(0, 0, w, h);
	mGLS.SetView(camera.View());
	mGLS.PrepareForDraw(lightPos, lightDir, camera.GetPosition());
}

void Graphics3D::ViewportShadow()
{
	mGLS.PrepareForShadow();
}

void Graphics3D::BeginDraw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Graphics3D::EndDraw()
{
	SwapBuffers(hDC);
}

void Graphics3D::DrawAxes()
{
	mGLS.DrawAxes();
}

void Graphics3D::MouseDown(int x, int y)
{
	mousePressed = true;
	mouseX = x;
	mouseY = y;
}

void Graphics3D::MouseUp(int x, int y)
{
	mousePressed = false;
}

void Graphics3D::MouseMove(int x, int y)
{
	if (!mousePressed)
		return;
	int dx = x - mouseX;
	int dy = y - mouseY;
	mouseX = x;
	mouseY = y;
	camera.Rotate(dx, dy);
}

void Graphics3D::MouseWheel(float delta)
{
	camera.Translate(delta, 0);
}

void Graphics3D::DrawPlane(const Vector3& pos, float scale, Texture* tex)
{
	mGLS.DrawPlane(pos, scale, tex);
}

int Graphics3D::DrawSphere(const Vector3& pos, float radius)
{
	return mGLS.DrawSphere(pos, radius);
}

void Graphics3D::DrawSphereTex(const Vector3& pos, float radius, const Matrix3& rot, Texture* tex)
{
	mGLS.DrawSphereTex(pos, radius, rot, tex);
}

void Graphics3D::DrawTriangle(const Vector3& a, const Vector3& b, const Vector3& c, bool fill)
{
	mGLS.DrawTriangle(a, b, c, fill);
}

void Graphics3D::DrawCube(const Vector3& pos, const Vector3& ext, const Matrix3& rot)
{
	mGLS.DrawCube(pos, ext, rot);
}

void Graphics3D::DrawCapsule(const Vector3& pos, float radius, float height, const Matrix3& rot)
{
	mGLS.DrawCapsule(pos, radius, height, rot);
}

void Graphics3D::DrawLine(const Vector3& a, const Vector3& b)
{
	mGLS.DrawLine(a, b);
}

void Graphics3D::DrawLines(const std::vector<Vector3>& points)
{
	mGLS.DrawLines(points);
}

void Graphics3D::DrawWireCube(const Vector3& pos, const Vector3& extent, const Matrix3& rot)
{
	// unit cube
	Vector3 p1(1, 1, 1);
	Vector3 p2(1, 1, -1);
	Vector3 p3(1, -1, -1);
	Vector3 p4(1, -1, 1);
	Vector3 p5(-1, 1, 1);
	Vector3 p6(-1, 1, -1);
	Vector3 p7(-1, -1, -1);
	Vector3 p8(-1, -1, 1);

	p1.Scale(extent);
	p2.Scale(extent);
	p3.Scale(extent);
	p4.Scale(extent);
	p5.Scale(extent);
	p6.Scale(extent);
	p7.Scale(extent);
	p8.Scale(extent);

	p1 = 0.5f * rot * p1 + pos;
	p2 = 0.5f * rot * p2 + pos;
	p3 = 0.5f * rot * p3 + pos;
	p4 = 0.5f * rot * p4 + pos;
	p5 = 0.5f * rot * p5 + pos;
	p6 = 0.5f * rot * p6 + pos;
	p7 = 0.5f * rot * p7 + pos;
	p8 = 0.5f * rot * p8 + pos;

	DrawLine(p1, p2);
	DrawLine(p2, p3);
	DrawLine(p3, p4);
	DrawLine(p4, p1);
	
	DrawLine(p5, p6);
	DrawLine(p6, p7);
	DrawLine(p7, p8);
	DrawLine(p8, p5);

	DrawLine(p1, p5);
	DrawLine(p2, p6);
	DrawLine(p3, p7);
	DrawLine(p4, p8);
}

int Graphics3D::DrawMesh(const RenderMesh& mesh)
{
	return mGLS.DrawMesh(mesh);
}

int Graphics3D::DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<uint32>& indices, const Matrix4* model)
{
	return mGLS.DrawMesh(vertices, normals, indices, model);
}

int Graphics3D::DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<Vector3>& colors, const std::vector<uint32>& indices, const Matrix4* model)
{
	return mGLS.DrawMesh(vertices, normals, colors, indices, model);
}

int Graphics3D::DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<uint32>& indices, const std::vector<Vector2>& uvs, Texture* tex, const Matrix4* model)
{
	return mGLS.DrawMesh(vertices, normals, indices, uvs, tex, model);
}

void Graphics3D::DrawTetrahedronEdgeAsBBCurve(const Vector3& c1, const Vector3& c2, const size_t nodesperedge, const Vector3* edge)
{
	mGLS.DrawTetrahedronEdgeAsBBCurve(c1, c2, nodesperedge, edge);
}

void Graphics3D::DrawTetrahedron(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& v3)
{
	mGLS.DrawTetrahedron(v0, v1, v2, v3);
}

void Graphics3D::SetColor(float r, float g, float b)
{
	color.Set(r, g, b);
	mGLS.SetColor(r, g, b);
}

void Graphics3D::GetScreenCoords(const Vector3& point, float& sx, float& sy)
{
	Matrix4 VP = mGLS.GetProjection() * mGLS.GetView();
	Vector4 p(point);
	Vector4 proj = VP * p;
	float px = proj.x / proj.w;
	float py = proj.y / proj.w;
	float pz = proj.z / proj.w;
	if (pz < 0)
		return;
	sx = 0.5f * (1.0f + px) * w;
	sy = 0.5f * (1.0f - py) * h;
}

void Graphics3D::SetRenderMode(int val)
{
	if (mRenderMode == RM_PRIMITIVE_IDS)
		return;

	mRenderMode = val;
	if (val == RM_WIREFRAME)
	{
		mGLS.SetProgram(true);
		SetFlags(GetFlags() | ShaderFlags::MONOCHROME);
	}
	else if (val == RM_WIREFRAME_ON_SHADED)
	{
		mGLS.SetProgram(true);
		int flags = GetFlags();
		flags &= ~ShaderFlags::MONOCHROME;
		SetFlags(flags);
	}
	else if (val == RM_SHADED)
	{
		mGLS.SetProgram(false);
		ResetFlags();
	}
	else if (val == RM_PRIMITIVE_IDS)
	{
		mGLS.UsePickProgram();
	}
}

void Graphics3D::SetShadowsActive(bool val)
{
	mGLS.SetShadowsActive(val);
}

#endif