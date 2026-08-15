#include <algorithm>

#include <MeChunk.h>
#include <MeMemory.h>
#include <MeMessage.h>

namespace {

void release(MeChunk* chunk)
{
    if (chunk->memBase)
        MeMemoryAPI.destroyAligned(chunk->memBase);
    chunk->memBase = nullptr;
    chunk->memSize = 0;
}

} // namespace

extern "C" {

void MEAPI MeChunkInit(MeChunk* chunk, int alignment)
{
    chunk->memBase = nullptr;
    chunk->memSize = 0;
    chunk->maxUsed = 0;
    chunk->alignment = alignment;
    chunk->mode = kMeChunkModeKeepOnPut;
    chunk->isInUse = MEFALSE;
}

void MEAPI MeChunkTerm(MeChunk* chunk)
{
    if (chunk->isInUse)
        MeWarning(0, "MeChunkTerm: chunk is still in use.");
    release(chunk);
    chunk->isInUse = MEFALSE;
}

void MEAPI MeChunkSetMode(MeChunk* chunk, kMeChunkMode mode)
{
    chunk->mode = mode;
}

void* MEAPI MeChunkGetMem(MeChunk* chunk, int size)
{
    if (chunk->isInUse)
    {
        MeWarning(0, "MeChunkGetMem: chunk is already in use.");
        return nullptr;
    }
    chunk->maxUsed = std::max(chunk->maxUsed, size);
    if (size > chunk->memSize)
    {
        // Doubling keeps reallocations rare while the working set grows over the first frames.
        int grown = std::max(size, 2 * chunk->memSize);
        release(chunk);
        chunk->memBase = MeMemoryAPI.createAligned(grown, chunk->alignment);
        if (!chunk->memBase)
            return nullptr;
        chunk->memSize = grown;
    }
    chunk->isInUse = 1;
    return chunk->memBase;
}

void MEAPI MeChunkPutMem(MeChunk* chunk, void* mem)
{
    if (!chunk->isInUse || mem != chunk->memBase)
        MeWarning(0, "MeChunkPutMem: memory was not taken from this chunk.");
    chunk->isInUse = MEFALSE;
    if (chunk->mode == kMeChunkModeFreeOnPut)
        release(chunk);
}

int MEAPI MeChunkGetMaxSize(MeChunk* chunk)
{
    return chunk->maxUsed;
}

int MEAPI MeChunkGetCurrentSize(MeChunk* chunk)
{
    return chunk->memSize;
}

MeBool MEAPI MeChunkIsInUse(MeChunk* chunk)
{
    return chunk->isInUse;
}

int MEAPI MeChunkGetAlignment(MeChunk* chunk)
{
    return chunk->alignment;
}

} // extern "C"
