#ifndef ENGINE_H
#define ENGINE_H

#include <Engine/Types.h>
#include <Engine/Utils.h>
#include <Engine/Platform.h>

#include <stdio.h>

class Graphics2D;
class Graphics3D;
class Profiler;

#define MAX_KEYS 256

#define RESOLUTION_X 800
#define RESOLUTION_Y 480

struct IEngineListener
{
	enum MouseButton
	{
		MOUSE_LEFT,
		MOUSE_MIDDLE,
		MOUSE_RIGHT
	};

	virtual void OnUpdate(float dt) { }
	virtual void OnDraw() { }
	virtual void OnDraw3D() { }
	virtual void OnDrawUI() { }
	virtual void OnCreate() { }
	virtual void OnResize() { }
	virtual void OnMouseMove(int x, int y) { }
	virtual void OnMouseDown(int x, int y, MouseButton mb) { }
	virtual void OnMouseUp(int x, int y, MouseButton mb) { }
	virtual void OnMouseWheel(int x, int y, float delta) { }
	virtual void OnMouseLeave() { }
	virtual void OnSecondaryTouch(int x, int y) { }
	virtual void OnSecondaryMove(int x, int y) { }
	virtual void OnSecondaryUp(int x, int y) { }
	// TODO: OnDestroy
};

class Engine : public IEngineListener
{
public:
	enum KeyState
	{
		KEY_UP = 0,
		KEY_RELEASED,
		KEY_PRESSED,
		KEY_DOWN
	};

protected:
	Graphics2D* graphics; // TODO: rename to graphics2D
	Profiler* profiler; // TODO: unique_ptr
	bool drawProfiler;
	bool is3D; // TODO: remove
	int screenWidth, screenHeight;

	Graphics3D* graphics3D;
	bool drawFps;
	bool canResize;
	bool maximized;
	bool fullscreen;
	bool active;

	// TODO: gfx settings
	bool drawShadows;
	bool drawDebugShadows;
	bool drawToTexture;
	bool enablePicking;
	bool debugPicking;

protected:
#ifdef _WIN32		
	HINSTANCE hInst;
	HWND hWnd;
	bool mouseTracking;
	TCHAR* szTitle; // window caption
#endif
	uint8 keyStates[MAX_KEYS];

private:
	static Engine* instance;
	double lastFpsTime, lastFrameTime;
	int frames;
	float fps, avgFrameLen;
	bool paused;
	bool gfxFailed;

public:

	Engine();
	virtual ~Engine();

	static Engine* getInstance() { return instance; }

	void Create();
		
	void UpdateAndDraw();

	// TODO: interface with pure virtual functions

	// override this function and put your drawing functionality in it
	inline uint8 GetKeyState(int keyCode);

	inline bool isKeyPressed(int keyCode);

	inline bool isKeyDown(int keyCode);

	inline bool wasKeyReleased(int keyCode);

	inline void GetMousePosition(POINT& point);

	static inline uint64 GetTimer();
		
	float getScreenWidth() const { return (float)screenWidth; }

	float getScreenHeight() const { return (float)screenHeight; }

	void Resize(int width, int height);

	void Pause() { paused = !paused; }

	Profiler* getProfiler() { ASSERT(profiler != NULL); return profiler; }

	void MouseDown(int x, int y, MouseButton mb);
	void MouseUp(int x, int y, MouseButton mb);
	void MouseMove(int x, int y);
	void MouseWheel(int x, int y, float delta);
	void MouseLeave();

	void Quit();

	void SetActive(bool val) { active = val; }

	void ToggleFullscreen();

	void ResizeWindow(int width, int height);
	HWND GetHandle() const { return hWnd; }
	void DeInit();

#ifdef WIN32
public:

	int Main(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPTSTR lpCmdLine, int nCmdShow);
	void CreateAsChild(HINSTANCE hInstance, HWND hParent, int x, int y, int w, int h);

private:

	ATOM MyRegisterClass(HINSTANCE hInstance);

	BOOL InitInstance(HINSTANCE hInstance, int nCmdShow);
#endif
};

LONG WINAPI MyUnhandledExceptionFilter(struct _EXCEPTION_POINTERS *ExceptionInfo);

#define RUN_ENGINE( ENGINE_CLASS, INST, PREV_INST, CMD_LINE, CMD_SHOW ) \
	SetUnhandledExceptionFilter(MyUnhandledExceptionFilter); \
	ENGINE_CLASS engine; \
	engine.Main(INST, PREV_INST, CMD_LINE, CMD_SHOW); \
	return 0;
	
#ifdef WIN32
#ifdef USE_IMGUI
#	include <imgui.h>
#endif
#include "EngineWin32.inl"
#elif defined(ANDROID_NDK)
#include "EngineAndroid.inl"
#endif

#endif // ENGINE_H
