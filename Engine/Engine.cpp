#include <Engine/Base.h>
#include <Engine/Engine.h>
#include <Graphics2D/Graphics2D.h>
#include <Graphics3D/Graphics3D.h>
#include <Engine/Profiler.h>

#include <float.h>
#ifdef WIN32
#	include <WindowsX.h>
#	include <strsafe.h>
#	include <DbgHelp.h>
#endif

#ifdef USE_IMGUI
// ImGui include
#include <imgui_impl_win32.h>
#include <imgui_impl_opengl3.h>

IMGUI_IMPL_API LRESULT  ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
#endif

#include <ctime>

#define LIMIT_FPS
#define LOG_FILE
//#define FPU_EXCEPTIONS

//extern FILE* out;
Engine* Engine::instance = NULL;
#ifdef ANDROID_NDK
extern char g_sdPath[64];
#endif

void CreateMiniDump(EXCEPTION_POINTERS* pep) 
{
	// Open the file 
	typedef BOOL (*PDUMPFN)( 
		HANDLE hProcess, 
		DWORD ProcessId, 
		HANDLE hFile, 
		MINIDUMP_TYPE DumpType, 
		PMINIDUMP_EXCEPTION_INFORMATION ExceptionParam, 
		PMINIDUMP_USER_STREAM_INFORMATION UserStreamParam, 
		PMINIDUMP_CALLBACK_INFORMATION CallbackParam
		);

	HANDLE hFile = CreateFile("dump.dmp", GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL ); 

	HMODULE h = ::LoadLibrary("DbgHelp.dll");
	PDUMPFN pFn = (PDUMPFN)GetProcAddress(h, "MiniDumpWriteDump");

	if( ( hFile != NULL ) && ( hFile != INVALID_HANDLE_VALUE ) ) 
	{
		// Create the minidump 
		MINIDUMP_EXCEPTION_INFORMATION mdei; 

		mdei.ThreadId           = GetCurrentThreadId(); 
		mdei.ExceptionPointers  = pep; 
		mdei.ClientPointers     = TRUE; 

		MINIDUMP_TYPE mdt       = MiniDumpNormal; 
		BOOL rv = (*pFn)( GetCurrentProcess(), GetCurrentProcessId(), hFile, mdt, (pep != 0) ? &mdei : 0, 0, 0 ); 

		// Close the file 
		CloseHandle( hFile ); 
	}

}

LONG WINAPI MyUnhandledExceptionFilter(struct _EXCEPTION_POINTERS *ExceptionInfo)
{
	CreateMiniDump(ExceptionInfo);
	return EXCEPTION_EXECUTE_HANDLER;
}

Engine::Engine()
	: frames(0)
	, lastFpsTime(0)
	, lastFrameTime(0)
	, fps(0)
	, paused(false)
	, drawProfiler(false)
	, drawFps(false)
	, screenWidth(RESOLUTION_X)
	, screenHeight(RESOLUTION_Y)
	, is3D(false)
	, graphics(NULL)
	, graphics3D(NULL)
	, profiler(NULL)
	, canResize(false)
	, maximized(false)
	, fullscreen(false)
	, active(true)
	, gfxFailed(false)
	, drawShadows(false)
	, drawDebugShadows(false)
	, drawToTexture(false)
	, enablePicking(false)
	, debugPicking(false)
{
	instance = this;
#ifdef WIN32
	mouseTracking = false;
	// TODO: append renderer suffix with preprocessor or std::string
#if (RENDERER == OPENGL)
	szTitle = "GameEngine - OpenGL";
#elif (RENDERER == OPENGLES)
	szTitle = L"GameEngine - OpenGL ES";
#elif (RENDERER == DIRECTX)
	zTitle = L"GameEngine - Direct3D9";
#else //if (RENDERER == GDIPLUS)
	szTitle = L"GameEngine - GDI+";
#endif
#endif // WIN32
	graphics = new Graphics2D();
}

void Engine::Resize(int w, int h)
{
	screenWidth = w;
	screenHeight = h;
	OnResize();
#if (RENDERER == OPENGL)
	if (is3D)
	{
		if (!graphics3D)
			graphics3D = new Graphics3D();
		graphics3D->Resize(w, h);
	}
#endif
	if (graphics)
		graphics->Resize(w, h);
}

Engine::~Engine()
{
	delete graphics;
}

void Engine::Create()
{
	// TODO: return boolean
	memset(keyStates, 0, MAX_KEYS);
#ifdef ANDROID_NDK
	int hWnd = 0;
	char buf[64];
	strcpy(buf, g_sdPath);
	strcat(buf, "/log.txt");
	out = fopen(buf, "wt");
	Printf("out file: %s\n", buf);
#elif defined(LOG_FILE)
	//fopen_s(&out, "log.txt", "wt");
#endif
	if (!graphics->Init(hWnd, hInst))
	{
		Printf("Failed to initialize graphics system\n");
		//Quit();
		//return;
		gfxFailed = true;
	}
	//graphics->Resize(10, 10); // TODO: default values

#if (RENDERER == OPENGL)
	if (is3D)
	{
		if (!graphics3D)
			graphics3D = new Graphics3D();
		if (!gfxFailed)
		{
			if (!graphics3D->Init(hWnd))
			{
				Quit();
				return;
			}
		}
	}
#endif

	profiler = new Profiler();
	OnCreate();
}

void Engine::ToggleFullscreen()
{
#ifdef WIN32
	fullscreen = !fullscreen;
	DWORD wndStyle = fullscreen ? WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN : WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
	if (!fullscreen && canResize)
		wndStyle |= WS_SIZEBOX | WS_MAXIMIZEBOX;;
	SetWindowLongPtr(hWnd, GWL_STYLE, wndStyle);
		
	RECT rc;
	SetRect( &rc, 0, 0, screenWidth, screenHeight);
	AdjustWindowRect( &rc, wndStyle, false );

	if (fullscreen)
		SetWindowPos(hWnd, NULL, 0, 0, (rc.right - rc.left), (rc.bottom - rc.top), 0); // are dimensions needed?

	ShowWindow(hWnd, SW_MAXIMIZE);
	UpdateWindow(hWnd);
#endif
}

void Engine::UpdateAndDraw()
{
#ifdef WIN32
	if (isKeyPressed(VK_SPACE))
	{
		paused = !paused;
	}
	if (isKeyDown(VK_CONTROL) && isKeyPressed('F'))
	{
		ToggleFullscreen();
	}
#endif
	if (paused)
		return;

	// get the frame time and limit it to 16ms
	double stopTime = profiler->GetTimeMs();
	double startTime = lastFrameTime;
	float frameTime = (float)(stopTime - startTime);
#if defined(LIMIT_FPS) && defined(WIN32)
	const float FRAME_TIME = 16.67f;
	if (frameTime < FRAME_TIME)
	{
		while (frameTime < FRAME_TIME)
		{
			//Sleep(FRAME_TIME - frameTime);
			SwitchToThread();
			stopTime = profiler->GetTimeMs();
			frameTime = (float)(stopTime - startTime);
		}
		lastFrameTime = profiler->GetTimeMs();
	}
	else
#endif
	{
		lastFrameTime = stopTime;
	}
	// compute the average frame rate
	frames++;
	double elapsedTime = lastFrameTime - lastFpsTime;
	if (elapsedTime > 400)
	{
		avgFrameLen = (float)elapsedTime / (float)frames;
		fps = 1000.0f / avgFrameLen;
		frames = 0;
		lastFpsTime = lastFrameTime;
	}

	// camera control
	float dz = 0;
	float dx = 0;
	float speed = .25f;
	if (isKeyDown('W'))
		dz = speed;
	else if (isKeyDown('S'))
		dz = -speed;
	if (isKeyDown('A'))
		dx = -speed;
	else if (isKeyDown('D'))
		dx = speed;
	if (dx || dz)
		graphics3D->camera.Translate(dz, dx);

	// TODO: Update function
	OnUpdate((float)(lastFrameTime - startTime));

	if (gfxFailed)
		return;
	
	// the actual drawing
	graphics->BeginDraw();
	profiler->Start(lastFrameTime);

	if (graphics3D)
	{
		graphics3D->SetShadowsActive(drawShadows);
		graphics3D->mGLS.SetScreenFBOActive(drawToTexture);
		graphics3D->mGLS.SetPickFBOActive(enablePicking);
		graphics3D->Draw();
	}

	// draw 2D
	BEGIN_PROFILE("Draw");
	graphics->SetContext();
	graphics->Viewport();

	if (drawShadows && drawDebugShadows)
	{
		// draw shadow map 
		graphics->DrawImage(&graphics3D->mGLS.GetShadowMap(), 0, 0, 0.2f);
	}

	if (drawToTexture)
	{
		// draw screen map
		graphics->DrawImage(&graphics3D->mGLS.GetScreenMap(), 0, 0, 0.2f);
	}

	if (debugPicking)
	{
		// draw picking map
		graphics->DrawImage(&graphics3D->mGLS.GetPickMap(), 0, 0, 0.2f);
	}

	OnDraw(); // virtual call!
	if (drawFps)
	{
		graphics->SetColor(0xffff0000);
		//graphics->DrawFormattedString(10, 10, "FPS: %.1f (%.1fms)", fps, avgFrameLen);
		graphics->DrawFormattedString(10, 20, "%.1f FPS", fps, avgFrameLen);
	}
	if (drawProfiler)
		profiler->Draw(graphics, stopTime);
	END_PROFILE();

	graphics->EndDraw();
	profiler->Stop();
}

void Engine::DeInit()
{
#if (RENDERER == OPENGL)
	if (is3D && graphics3D)
	{
		graphics3D->DeInit();
		delete graphics3D;
	}
#endif
	if (graphics)
	{
		graphics->DeInit(); // TODO: explicit call
	}
	if (profiler)
		delete profiler;
}

void Engine::MouseDown(int x, int y, MouseButton mb)
{
#if (RENDERER == OPENGL)
	if ((mb == MOUSE_RIGHT || mb == MOUSE_MIDDLE) && is3D)
	{
		graphics3D->MouseDown(x, y);
	}
#endif
	OnMouseDown(x, y, mb);
}

void Engine::MouseUp(int x, int y, MouseButton mb)
{
#if (RENDERER == OPENGL)
	if ((mb == MOUSE_RIGHT || mb == MOUSE_MIDDLE) && is3D)
	{
		graphics3D->MouseUp(x, y);
	}
#endif
	OnMouseUp(x, y, mb);
}

void Engine::MouseMove(int x, int y)
{
#if defined(WIN32) && (RENDERER == OPENGL)
	if (!mouseTracking)
	{
		mouseTracking = true;
		TRACKMOUSEEVENT eventTrack;
		eventTrack.cbSize = sizeof(TRACKMOUSEEVENT);
		eventTrack.dwFlags = TME_LEAVE;
		eventTrack.hwndTrack = hWnd;
		eventTrack.dwHoverTime = 0;
		TrackMouseEvent(&eventTrack);
	}

	if (is3D && graphics3D)
	{
		graphics3D->MouseMove(x, y);
	}
	OnMouseMove(x, y);
#endif
}

void Engine::MouseLeave()
{
#ifdef WIN32
	mouseTracking = false;
#endif
	OnMouseLeave();
}

void Engine::MouseWheel(int x, int y, float delta)
{
#if (RENDERER == OPENGL)
	if (is3D)
	{
		graphics3D->MouseWheel(delta);
	}
#endif
	OnMouseWheel(x, y, delta);
}

#ifdef _WIN32

TCHAR* szWindowClass = "GEWndClass"; 		// the main window class name

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
#ifdef USE_IMGUI
	LRESULT ret = ImGui_ImplWin32_WndProcHandler(hWnd, message, wParam, lParam);

	if (ImGui::GetCurrentContext() != nullptr && 
		(ImGui::GetIO().WantCaptureMouse || ImGui::GetIO().WantCaptureKeyboard))
	{
		return 1;
	}
#endif
	
	int w, h;
	switch (message)
	{
		case WM_DESTROY:
			PostQuitMessage(0);
			break;
		case WM_SIZE:
			RECT rect;
			GetClientRect(hWnd, &rect);
			w = rect.right - rect.left;
			h = rect.bottom - rect.top;
			Engine::getInstance()->Resize(w, h);
			break;
		case WM_MOUSEMOVE:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			Engine::getInstance()->MouseMove(xPos, yPos);
		}
		break;
		case WM_LBUTTONDOWN:
		{
			SetFocus(hWnd);
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			Engine::getInstance()->MouseDown(xPos, yPos, Engine::MOUSE_LEFT);
		}
		break;
		case WM_LBUTTONUP:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			Engine::getInstance()->MouseUp(xPos, yPos, Engine::MOUSE_LEFT);
		}
		break;
		case WM_RBUTTONDOWN:
		{
			SetFocus(hWnd);
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			Engine::getInstance()->MouseDown(xPos, yPos, Engine::MOUSE_RIGHT);
		}
		break;
		case WM_RBUTTONUP:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			Engine::getInstance()->MouseUp(xPos, yPos, Engine::MOUSE_RIGHT);
		}
		break;
		case WM_MBUTTONDOWN:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			Engine::getInstance()->MouseDown(xPos, yPos, Engine::MOUSE_MIDDLE);
		}
		break;
		case WM_MBUTTONUP:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			Engine::getInstance()->MouseUp(xPos, yPos, Engine::MOUSE_MIDDLE);
		}
		break;
		case WM_MOUSEWHEEL:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			float delta = GET_WHEEL_DELTA_WPARAM(wParam) / 120.f;
			POINT p;
			p.x = xPos;
			p.y = yPos;
			ScreenToClient(hWnd, &p);
			Engine::getInstance()->MouseWheel(p.x, p.y, delta);
		}
		break;
		case WM_MOUSELEAVE:
			Engine::getInstance()->MouseLeave();
			break;
		case WM_ACTIVATEAPP:
			//Printf("activate app: %d\n", wParam);
			Engine::getInstance()->SetActive(wParam == TRUE);
			break;
		case WM_ERASEBKGND:
			break; // don't erase the bg
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

int Engine::Main(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPTSTR lpCmdLine, int nCmdShow)
{
#ifdef FPU_EXCEPTIONS
	// activate floating point exceptions
	_clearfp();
	unsigned int cw;
	_controlfp_s(&cw, 0, 0);
	cw &=~(/*EM_OVERFLOW | EM_UNDERFLOW |*/ EM_ZERODIVIDE | EM_DENORMAL | EM_INVALID);
	unsigned int cwOriginal;
	_controlfp_s(&cwOriginal, cw, MCW_EM);
#endif

	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	MSG msg;

	if (MyRegisterClass(hInstance) == 0)
		return FALSE;

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	// Main message loop:
	while (true)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) != 0)
		{
			if (msg.message == WM_QUIT)
			{
				break;
			}
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else //if (active)
		{
			// update and draw the frame
			UpdateAndDraw();
		}		
	}
	
	//if (out)
	//	fclose(out);

	DeInit(); // added here instead of destructor

	return (int) msg.wParam;
}

ATOM Engine::MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= 0;
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName	= 0; 
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= 0;

	return RegisterClassEx(&wcex);
}

BOOL Engine::InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	ShowWindow(hWnd, SW_MAXIMIZE);
	UpdateWindow(hWnd);

	hInst = hInstance; // Store instance handle in our global variable
	DWORD wndStyle = fullscreen ? WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN : WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
	if (!fullscreen && canResize)
		wndStyle |= WS_SIZEBOX | WS_MAXIMIZEBOX;

	RECT rc;
	SetRect(&rc, 0, 0, screenWidth, screenHeight);
	AdjustWindowRect(&rc, wndStyle, false );	

	hWnd = CreateWindow(szWindowClass, szTitle, wndStyle,
		CW_USEDEFAULT, CW_USEDEFAULT, (rc.right - rc.left), (rc.bottom - rc.top), NULL, NULL, hInstance, NULL);

	if (!hWnd)
	{
		return FALSE;
	}

	Create();

	if (fullscreen)
		SetWindowPos(hWnd, NULL, 0, 0, (rc.right - rc.left), (rc.bottom - rc.top), 0); // are dimensions needed?
	ShowWindow(hWnd, maximized ? SW_MAXIMIZE : nCmdShow);
	UpdateWindow(hWnd);

	return TRUE;
}

void ErrorExit(LPTSTR lpszFunction) 
{ 
	// Retrieve the system error message for the last-error code

	LPVOID lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError(); 

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | 
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR) &lpMsgBuf,
		0, NULL );

	// Display the error message and exit the process

	lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT, 
		(lstrlen((LPCTSTR)lpMsgBuf) + lstrlen((LPCTSTR)lpszFunction) + 40) * sizeof(TCHAR)); 
	StringCchPrintf((LPTSTR)lpDisplayBuf, 
		LocalSize(lpDisplayBuf) / sizeof(TCHAR),
		TEXT("%s failed with error %d: %s"), 
		lpszFunction, dw, lpMsgBuf); 
	MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK); 

	LocalFree(lpMsgBuf);
	LocalFree(lpDisplayBuf);
	ExitProcess(dw); 
}

void Engine::CreateAsChild(HINSTANCE hInstance, HWND hParent, int x, int y, int w, int h)
{
	hInst = hInstance;
	MyRegisterClass(hInstance);

	hWnd = CreateWindowEx(NULL, 
		szWindowClass,
		szTitle,
		WS_TABSTOP|WS_VISIBLE|WS_CHILD,
		x,
		y,
		w,
		h,
		hParent,
		(HMENU)201, //IDC_MAIN_BUTTON,
		GetModuleHandle(NULL),
		NULL);

	if (hWnd == NULL)
		ErrorExit("CreateWindowEx");

	Create();
}

void Engine::ResizeWindow(int width, int height)
{
	SetWindowPos(hWnd, NULL, 0, 0, width, height, 0);
}

#endif // _WIN32
