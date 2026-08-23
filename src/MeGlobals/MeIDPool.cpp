#include <cstdlib>
#include <cstring>

#include <MeIDPool.h>

namespace {

// Words of 32 IDs added each time the pool runs out.
const unsigned int kWordsPerBlock = 4;

void resize(MeIDPool* pool, unsigned int blocks)
{
    unsigned int words = blocks * pool->block;
    pool->IDbitfield = static_cast<MeU32*>(std::realloc(pool->IDbitfield, words * sizeof(MeU32)));
    unsigned int oldWords = pool->maxBlocks * pool->block;
    std::memset(pool->IDbitfield + oldWords, 0, (words - oldWords) * sizeof(MeU32));
    pool->maxBlocks = blocks;
    pool->maxIDs = words * 32;
}

} // namespace

extern "C" {

MeIDPool* MEAPI MeIDPoolCreate()
{
    MeIDPool* pool = static_cast<MeIDPool*>(std::calloc(1, sizeof(MeIDPool)));
    pool->block = kWordsPerBlock;
    resize(pool, 1);
    return pool;
}

void MEAPI MeIDPoolCopy(MeIDPool* to, MeIDPool* from)
{
    if (to == from)
        return;
    std::free(to->IDbitfield);
    *to = *from;
    size_t bytes = from->maxBlocks * from->block * sizeof(MeU32);
    to->IDbitfield = static_cast<MeU32*>(std::malloc(bytes));
    std::memcpy(to->IDbitfield, from->IDbitfield, bytes);
}

MeBool MEAPI MeIDPoolIsEmpty(MeIDPool* pool)
{
    return pool->assignedIDs == 0;
}

int MEAPI MeIDPoolRequestID(MeIDPool* pool)
{
    if (pool->assignedIDs == pool->maxIDs)
        resize(pool, pool->maxBlocks + 1);
    for (unsigned int w = 0;; ++w)
    {
        if (pool->IDbitfield[w] == 0xffffffffu)
            continue;
        unsigned int bit = 0;
        while (pool->IDbitfield[w] & (1u << bit))
            ++bit;
        pool->IDbitfield[w] |= 1u << bit;
        ++pool->assignedIDs;
        return static_cast<int>(w * 32 + bit);
    }
}

void MEAPI MeIDPoolReturnID(MeIDPool* pool, int id)
{
    if (id < 0 || static_cast<unsigned int>(id) >= pool->maxIDs)
        return;
    MeU32 mask = 1u << (id % 32);
    if (pool->IDbitfield[id / 32] & mask)
    {
        pool->IDbitfield[id / 32] &= ~mask;
        --pool->assignedIDs;
    }
}

void MEAPI MeIDPoolReset(MeIDPool* pool)
{
    std::memset(pool->IDbitfield, 0, pool->maxBlocks * pool->block * sizeof(MeU32));
    pool->assignedIDs = 0;
}

void MEAPI MeIDPoolDestroy(MeIDPool* pool)
{
    if (!pool)
        return;
    std::free(pool->IDbitfield);
    std::free(pool);
}

} // extern "C"
