#include <MeMemory.h>
#include <MePool.h>

namespace {

void* allocate(size_t bytes, int alignment)
{
    return alignment > 0 ? MeMemoryAPI.createAligned(bytes, alignment) : MeMemoryAPI.create(bytes);
}

void release(void* block, bool aligned)
{
    if (aligned)
        MeMemoryAPI.destroyAligned(block);
    else
        MeMemoryAPI.destroy(block);
}

// The free stack holds every slot, nextFreeStruct splits it into used [0, next) and free [next, poolSize).
void MEAPI fixedDeset(MePool* pool)
{
    MePoolFixed& f = pool->u.fixed;
    char* slots = static_cast<char*>(f.structArray);
    for (int i = 0; i < f.poolSize; ++i)
        f.freeStructStack[i] = slots + i * f.structSize;
    f.nextFreeStruct = 0;
}

void MEAPI fixedInit(MePool* pool, int poolSize, int structSize, int alignment)
{
    pool->t = MePoolFIXED;
    MePoolFixed& f = pool->u.fixed;
    // Rounding the slot size keeps every slot aligned, not only the first.
    if (alignment > 0)
        structSize = (structSize + alignment - 1) / alignment * alignment;
    f.structSize = structSize;
    f.poolSize = poolSize;
    f.createdAligned = alignment > 0;
    f.structArray = allocate(size_t(poolSize) * structSize, alignment);
    f.freeStructStack = static_cast<void**>(MeMemoryAPI.create(sizeof(void*) * poolSize));
    fixedDeset(pool);
}

void MEAPI fixedDestroy(MePool* pool)
{
    MePoolFixed& f = pool->u.fixed;
    release(f.structArray, f.createdAligned);
    MeMemoryAPI.destroy(f.freeStructStack);
    pool->t = MePoolNULL;
}

void* MEAPI fixedGetStruct(MePool* pool)
{
    MePoolFixed& f = pool->u.fixed;
    return f.nextFreeStruct < f.poolSize ? f.freeStructStack[f.nextFreeStruct++] : nullptr;
}

void MEAPI fixedPutStruct(MePool* pool, void* s)
{
    MePoolFixed& f = pool->u.fixed;
    f.freeStructStack[--f.nextFreeStruct] = s;
}

int MEAPI fixedGetUsed(MePool* pool)
{
    return pool->u.fixed.nextFreeStruct;
}

int MEAPI fixedGetUnused(MePool* pool)
{
    return pool->u.fixed.poolSize - pool->u.fixed.nextFreeStruct;
}

void MEAPI mallocInit(MePool* pool, int poolSize, int structSize, int alignment)
{
    pool->t = MePoolMALLOC;
    pool->u.malloc = {0, structSize, poolSize, alignment};
}

void MEAPI mallocDestroy(MePool* pool)
{
    pool->t = MePoolNULL;
}

// Structs are freed one by one through putStruct, so there is nothing to reset.
void MEAPI mallocDeset(MePool*)
{
}

void* MEAPI mallocGetStruct(MePool* pool)
{
    MePoolMalloc& m = pool->u.malloc;
    if (m.usedStructs >= m.poolSize)
        return nullptr;
    ++m.usedStructs;
    return allocate(m.structSize, m.alignment);
}

void MEAPI mallocPutStruct(MePool* pool, void* s)
{
    MePoolMalloc& m = pool->u.malloc;
    --m.usedStructs;
    release(s, m.alignment > 0);
}

int MEAPI mallocGetUsed(MePool* pool)
{
    return pool->u.malloc.usedStructs;
}

int MEAPI mallocGetUnused(MePool* pool)
{
    return pool->u.malloc.poolSize - pool->u.malloc.usedStructs;
}

} // namespace

extern "C" {

struct MePoolAPI MePoolFixedAPI = {fixedInit,      fixedDestroy, fixedDeset,    fixedGetStruct,
                                   fixedPutStruct, fixedGetUsed, fixedGetUnused};
struct MePoolAPI MePoolMallocAPI = {mallocInit,      mallocDestroy, mallocDeset,    mallocGetStruct,
                                    mallocPutStruct, mallocGetUsed, mallocGetUnused};
struct MePoolAPI MePoolAPI = MePoolFixedAPI;

} // extern "C"
