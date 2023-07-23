#ifndef PROFILER_H
#define PROFILER_H

#include <Engine/Types.h>
#include <Engine/Utils.h>

#include <vector>
#include <map>
#include <string>

class Graphics2D;

class Profiler
{
private:
	struct ProfileData
	{
		std::vector<float> start;
		std::vector<float> stop;
		uint32 color;
		float total;
		int level;
	};
	struct ProfileTree
	{
		const char* mName;
		ProfileData* mData;
		ProfileTree* mParent;
		std::vector<ProfileTree*> mChildren;

		ProfileTree() : mName(NULL), mData(NULL), mParent(NULL) {}
		// TODO: destructor/delete
	};
	enum { NUM_THREADS = 2 };

private:
	std::vector<ProfileData> profilerTimes[NUM_THREADS];
	std::vector<ProfileData> drawTimes[NUM_THREADS];
	double startTime, prevStartTime;
	std::map<std::string, size_t> nameMap;
#ifdef WIN32
	double clockFreq;
#endif
	int mLevel;
	int mAddCounter;
	ProfileTree mRoot;
	ProfileTree* mCurrentRoot;

public:
	Profiler();
	double GetTimeMs();
	static double GetTimeMsStatic();
	size_t BeginProfile(const char* name, int thread = 0);
	void EndProfile(size_t idx, int thread = 0);
	void Start(double start);
	void Stop();
	void Draw(Graphics2D* graphics, double stopTime);
private:
	int AddProfile(const char* name, uint32 c);
};

class MeasureScope
{
private:
	double start;
	const char* str;
	double* ms_p;
public:
	MeasureScope(const char* name) : str(name), ms_p(nullptr)
	{
		//start = Engine::getInstance()->getProfiler()->GetTimeMs();
		start = Profiler::GetTimeMsStatic();
	}

	MeasureScope(const char* name, double& ms) : str(name)
	{
		//start = Engine::getInstance()->getProfiler()->GetTimeMs();
		start = Profiler::GetTimeMsStatic();
		ms_p = &ms;
	}

	~MeasureScope()
	{
		//double stop = Engine::getInstance()->getProfiler()->GetTimeMs();
		double stop = Profiler::GetTimeMsStatic();
		if (ms_p)
			(*ms_p) = (stop - start);
		float dt = (float)(stop - start);
		if (dt > 1000)
			Printf("%s duration: %f s\n", str, dt / 1000);
		else if (dt > 1)
			Printf("%s duration: %f ms\n", str, dt);
		else
			Printf("%s duration: %f us\n", str, dt * 1000);
	}
};
#define MEASURE_TIME(NAME) MeasureScope ms(NAME)
#define MEASURE_TIME_P(NAME, MS) MeasureScope ms(NAME, MS)

#if !defined(ANDROID_NDK) && !defined(DISABLE_PROFILER) && !defined(LINUX)
#define USE_PROFILER
#endif

#if defined(USE_PROFILER)
#include <Engine/Engine.h>
static size_t _profileIdx;
#define BEGIN_PROFILE(NAME, ...) _profileIdx = Engine::getInstance()->getProfiler()->BeginProfile(NAME, __VA_ARGS__);
#define END_PROFILE(...) Engine::getInstance()->getProfiler()->EndProfile(_profileIdx, __VA_ARGS__);

class ProfileScope
{
private:
	size_t num;
	int thread;
public:
	ProfileScope(const char* name, int t = 0) : thread(t) 
	{
		num = Engine::getInstance()->getProfiler()->BeginProfile(name, thread); 
	}
	~ProfileScope() 
	{
		Engine::getInstance()->getProfiler()->EndProfile(num, thread);
	}
};

#define PROFILE_SCOPE(NAME, ...) ProfileScope ps(NAME, __VA_ARGS__);

#elif defined(ANDROID_NDK)
#define BEGIN_PROFILE(NAME)
#define END_PROFILE()
#define PROFILE_SCOPE(NAME)
#else
#define BEGIN_PROFILE(NAME, ...)
#define END_PROFILE(...)
#define PROFILE_SCOPE(NAME, ...)
#endif

#include <Engine/Profiler.inl>

#endif // PROFILER_H
