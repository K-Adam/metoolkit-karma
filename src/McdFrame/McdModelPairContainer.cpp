#include <cstring>

#include "McdImpl.h"

// Pairs are stored contiguously as [goodbye][hello][staying], so goodbyeFirst and helloFirst are fixed
// ends of their neighbours and inserting into an inner range shifts the ranges after it.

namespace {

MeBool insertAt(McdModelPairContainer* c, int at, McdModelPairID p)
{
    if (c->stayingEnd >= c->size)
        return 0;
    std::memmove(c->array + at + 1, c->array + at, (c->stayingEnd - at) * sizeof(McdModelPair*));
    c->array[at] = p;
    ++c->stayingEnd;
    return 1;
}

} // namespace

extern "C" {

void MEAPI McdModelPairContainerInit(McdModelPairContainer* c, McdModelPair** array, int size)
{
    c->array = array;
    c->size = size;
    McdModelPairContainerReset(c);
}

McdModelPairContainer* MEAPI McdModelPairContainerCreate(int size)
{
    auto* c = new McdModelPairContainer{};
    McdModelPairContainerInit(c, new McdModelPair*[size], size);
    return c;
}

void MEAPI McdModelPairContainerDestroy(McdModelPairContainer* c)
{
    if (!c)
        return;
    delete[] c->array;
    delete c;
}

void MEAPI McdModelPairContainerReset(McdModelPairContainer* c)
{
    c->goodbyeFirst = c->goodbyeEnd = 0;
    c->helloFirst = c->helloEndStayingFirst = c->stayingEnd = 0;
}

int MEAPI McdModelPairContainerGetGoodbyeCount(McdModelPairContainer* c)
{
    return c->goodbyeEnd - c->goodbyeFirst;
}

int MEAPI McdModelPairContainerGetHelloCount(McdModelPairContainer* c)
{
    return c->helloEndStayingFirst - c->helloFirst;
}

int MEAPI McdModelPairContainerGetStayingCount(McdModelPairContainer* c)
{
    return c->stayingEnd - c->helloEndStayingFirst;
}

McdModelPair** MEAPI McdModelPairContainerGetGoodbyeArray(McdModelPairContainer* c, int* count)
{
    *count = McdModelPairContainerGetGoodbyeCount(c);
    return c->array + c->goodbyeFirst;
}

McdModelPair** MEAPI McdModelPairContainerGetHelloArray(McdModelPairContainer* c, int* count)
{
    *count = McdModelPairContainerGetHelloCount(c);
    return c->array + c->helloFirst;
}

McdModelPair** MEAPI McdModelPairContainerGetStayingArray(McdModelPairContainer* c, int* count)
{
    *count = McdModelPairContainerGetStayingCount(c);
    return c->array + c->helloEndStayingFirst;
}

MeBool MEAPI McdModelPairContainerInsertGoodbyePair(McdModelPairContainer* c, McdModelPairID p)
{
    if (!insertAt(c, c->goodbyeEnd, p))
        return 0;
    ++c->goodbyeEnd;
    ++c->helloFirst;
    ++c->helloEndStayingFirst;
    return 1;
}

MeBool MEAPI McdModelPairContainerInsertHelloPair(McdModelPairContainer* c, McdModelPairID p)
{
    if (!insertAt(c, c->helloEndStayingFirst, p))
        return 0;
    ++c->helloEndStayingFirst;
    return 1;
}

MeBool MEAPI McdModelPairContainerInsertStayingPair(McdModelPairContainer* c, McdModelPairID p)
{
    return insertAt(c, c->stayingEnd, p);
}

void MEAPI McdModelPairContainerIteratorInit(McdModelPairContainerIterator* it, McdModelPairContainer* c)
{
    it->container = c;
    it->count = 0;
}

} // extern "C"
