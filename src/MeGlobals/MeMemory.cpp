#include <cstdlib>
#if defined(_MSC_VER)
#include <malloc.h>
#endif

#include <MeMemory.h>

namespace {

void* MEAPI create(size_t bytes)
{
    return std::malloc(bytes);
}

void* MEAPI createZeroed(size_t bytes)
{
    return std::calloc(1, bytes);
}

void* MEAPI createAligned(size_t bytes, unsigned int alignment)
{
#if defined(_MSC_VER)
    return _aligned_malloc(bytes, alignment);
#else
    return std::aligned_alloc(alignment, (bytes + alignment - 1) / alignment * alignment);
#endif
}

void MEAPI destroy(void* const block)
{
    std::free(block);
}

void MEAPI destroyAligned(void* const block)
{
#if defined(_MSC_VER)
    _aligned_free(block);
#else
    std::free(block);
#endif
}

void* MEAPI resize(void* const block, size_t bytes)
{
    return std::realloc(block, bytes);
}

} // namespace

extern "C" MeMemoryAPIStruct MeMemoryAPI = {create, createZeroed, createAligned, destroy, destroyAligned, resize};
