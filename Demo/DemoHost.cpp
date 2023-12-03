#include "DemoHost.h"
#include "Graphics2D/Graphics2D.h"
#include "Graphics3D/Graphics3D.h"

DemoHost::DemoHost()
{
	canResize = true;
	is3D = true;
	enablePicking = true;
	graphics->SetBgColor(0xff9f9f9f);
}

void DemoHost::OnCreate()
{
	if (is3D)
	{
		graphics3D->camera.SetPosition(Vector3(0, 30, 70));
		graphics3D->SetLightPos(Vector3(400, 800, 0));
	}

	Reset();
}

void DemoHost::Reset()
{
	pausePhysics = true;
	mClothDemo.Create(mDemoType);
	mClothDemo.Init();
}

void DemoHost::OnUpdate(float dt)
{
	if (isKeyPressed('X'))
		drawProfiler = !drawProfiler;
	if (isKeyPressed('F'))
		drawFps = !drawFps;

	if (isKeyPressed('R'))
	{
		Reset();
	}
	if (isKeyPressed('T'))
		pausePhysics = !pausePhysics;
	if (isKeyPressed('Y'))
		stepByStep = !stepByStep;

	if (!pausePhysics)
	{
		mClothDemo.Update(timeStep);
		pausePhysics = stepByStep; // for step by step
	}
}

void DemoHost::OnDraw3D()
{
	mClothDemo.Draw(graphics3D, mShowDebug, mDebugDrawFlags);
}

void DemoHost::OnDrawUI()
{
	if (ImGui::Button("Simulate"))
	{
		pausePhysics = !pausePhysics;
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset"))
	{
		Reset();
	}
	ImGui::Combo("Demo type", &mDemoType, "Default\0Sphere\0Capsule\0Mesh\0SDF\0");
	ImGui::Checkbox("Step by step", &stepByStep);
	ImGui::SliderFloat("Time step", &timeStep, 0.001f, 0.05f);
	ImGui::Checkbox("Show debug", &mShowDebug);
	if (mShowDebug)
	{
		bool showParticles = (mDebugDrawFlags & DDF_PARTICLES) != 0;
		if (ImGui::Checkbox("Show particles", &showParticles))
		{
			if (showParticles)
				mDebugDrawFlags |= DDF_PARTICLES;
			else
				mDebugDrawFlags &= ~DDF_PARTICLES;
		}

		bool showContacts = (mDebugDrawFlags & DDF_CONTACTS) != 0;
		if (ImGui::Checkbox("Show contacts", &showContacts))
		{
			if (showContacts)
				mDebugDrawFlags |= DDF_CONTACTS;
			else
				mDebugDrawFlags &= ~DDF_CONTACTS;
		}

		bool showTriContacts = (mDebugDrawFlags & DDF_TRI_CONTACTS) != 0;
		if (ImGui::Checkbox("Show triangle contacts", &showTriContacts))
		{
			if (showTriContacts)
				mDebugDrawFlags |= DDF_TRI_CONTACTS;
			else
				mDebugDrawFlags &= ~DDF_TRI_CONTACTS;
		}
	}

	if (ImGui::CollapsingHeader("Rendering"))
	{
		Vector3 light = graphics3D->GetLightPos();
		if (ImGui::InputFloat3("Light", light.v))
		{
			graphics3D->SetLightPos(light);
		}
		//ImGui::Checkbox("Light follows camera", &mLightFollowsCamera);
		ImGui::Checkbox("Enable shadows", &drawShadows);
		ImGui::Checkbox("Debug shadows", &drawDebugShadows);
	}

	mClothDemo.DrawUI();
}

void DemoHost::OnMouseMove(int x, int y)
{
	mClothDemo.OnMouseMove(x, y);
}

void DemoHost::OnMouseDown(int x, int y, MouseButton mb)
{
	if (mb == MouseButton::MOUSE_LEFT)
		mClothDemo.OnMouseDown(x, y);
}

void DemoHost::OnMouseUp(int x, int y, MouseButton mb)
{
	mClothDemo.OnMouseUp(x, y);
}


int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	RUN_ENGINE(DemoHost, hInstance, hPrevInstance, lpCmdLine, nCmdShow);
}
