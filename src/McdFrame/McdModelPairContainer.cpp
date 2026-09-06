#include <cstring>
#include <initializer_list>

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

// p points into the container's array. Every range boundary after the removed slot moves down by one.
void MEAPI McdModelPairContainerRemovePair(McdModelPairContainer* c, McdModelPairID* p)
{
    int at = (int)(p - c->array);
    if (at < 0 || at >= c->stayingEnd)
        return;
    std::memmove(c->array + at, c->array + at + 1, (c->stayingEnd - at - 1) * sizeof(McdModelPair*));
    for (int* boundary : {&c->goodbyeEnd, &c->helloFirst, &c->helloEndStayingFirst, &c->stayingEnd})
        if (*boundary > at)
            --*boundary;
}

void MEAPI McdModelPairContainerPrintStats(McdModelPairContainer* c)
{
    MeInfo(0, "McdModelPairContainer: size %d goodbye %d hello %d staying %d", c->size,
           McdModelPairContainerGetGoodbyeCount(c), McdModelPairContainerGetHelloCount(c),
           McdModelPairContainerGetStayingCount(c));
}

void MEAPI McdPairContainerPrintStats(McdModelPairContainer* c)
{
    McdModelPairContainerPrintStats(c);
}

void MEAPI McdModelPairContainerIteratorInit(McdModelPairContainerIterator* it, McdModelPairContainer* c)
{
    it->container = c;
    it->count = 0;
}

} // extern "C"

int mcdFillContainer(McdModelPairContainer* c, McdSpacePairIterator* it, const std::vector<McdModelPair*>& goodbye,
                     const std::vector<McdModelPair*>& hello, const std::vector<McdModelPair*>* staying)
{
    McdModelPairContainerReset(c);
    const std::vector<McdModelPair*>* lists[] = {&goodbye, &hello, staying};
    MeBool (MEAPI * insert[])(McdModelPairContainer*, McdModelPairID) = {
        McdModelPairContainerInsertGoodbyePair, McdModelPairContainerInsertHelloPair,
        McdModelPairContainerInsertStayingPair};
    int index = 0;
    for (int l = 0; l < 3; ++l)
    {
        if (!lists[l])
            continue;
        for (McdModelPair* p : *lists[l])
        {
            if (index++ < it->count)
                continue;
            if (!insert[l](c, p))
                return 1;
            ++it->count;
        }
    }
    return 0;
}
