#include <Engine/Base.h>

FILE* out = NULL;

// aligned new and delete
#if !defined(_WIN64) && (USE_SSE == 1)

#	pragma warning(disable: 4290)
#	include <new>

void* operator new(size_t size) throw(std::bad_alloc)
{
	if (size == 0)
		size = 1;

	while (true)
	{
		void* ptr = _aligned_malloc(size, 16);
		if (ptr != NULL)
			return ptr;

		new_handler globalHandler = std::set_new_handler(0);
		std::set_new_handler(globalHandler);

		if (globalHandler)
			(*globalHandler)();
		else
			throw std::bad_alloc();
	}
}

void operator delete(void* ptr) throw()
{
	if (ptr == NULL)
		return;
	_aligned_free(ptr);
}
#endif
