#include <cstdint>

#include <MeDict.h>
#include <MePool.h>

#include "check.h"

namespace {

int compareInts(const void* a, const void* b)
{
    intptr_t x = (intptr_t)a, y = (intptr_t)b;
    return x < y ? -1 : x > y;
}

const void* key(intptr_t k)
{
    return (const void*)k;
}

} // namespace

int main()
{
    MeDict d;
    MeDictInit(&d, DICTCOUNT_T_MAX, compareInts);
    MeDictNode nodes[64];
    // Inserted out of order, read back sorted.
    for (int i = 0; i < 64; ++i)
    {
        int k = (i * 37) % 64;
        MeDictNodeInit(&nodes[k], &nodes[k]);
        MeDictInsert(&d, &nodes[k], key(k));
    }
    CHECK(MeDictCount(&d) == 64 && MeDictVerify(&d));
    int expected = 0;
    for (MeDictNode* n = MeDictFirst(&d); n; n = MeDictNext(&d, n))
    {
        CHECK(MeDictNodeGetKey(n) == key(expected));
        CHECK(MeDictNodeGet(n) == &nodes[expected]);
        ++expected;
    }
    CHECK(expected == 64);

    for (int k = 0; k < 64; k += 2)
        MeDictDelete(&d, &nodes[k]);
    CHECK(MeDictCount(&d) == 32 && MeDictVerify(&d));
    CHECK(MeDictLookup(&d, key(10)) == nullptr && MeDictLookup(&d, key(11)) == &nodes[11]);
    CHECK(MeDictLowerBound(&d, key(10)) == &nodes[11] && MeDictUpperBound(&d, key(10)) == &nodes[9]);
    CHECK(MeDictFirst(&d) == &nodes[1] && MeDictLast(&d) == &nodes[63]);
    CHECK(MeDictPrev(&d, &nodes[11]) == &nodes[9]);
    CHECK(MeDictContains(&d, &nodes[11]) && !MeDictNodeIsInADict(&nodes[10]));

    // The allocating interface frees its own nodes.
    MeDict* owned = MeDictCreate(DICTCOUNT_T_MAX, compareInts);
    for (int k = 0; k < 10; ++k)
        CHECK(MeDictAllocInsert(owned, key(k), nullptr));
    MeDictDeleteFree(owned, MeDictLookup(owned, key(3)));
    CHECK(MeDictCount(owned) == 9 && MeDictVerify(owned));
    MeDictFreeNodes(owned);
    CHECK(MeDictIsEmpty(owned));
    MeDictDestroy(owned);

    // A fixed pool hands out at most its size, a malloc pool counts what is out.
    MePool fixed;
    MePoolFixedAPI.init(&fixed, 2, 24, 16);
    void* a = MePoolFixedAPI.getStruct(&fixed);
    void* b = MePoolFixedAPI.getStruct(&fixed);
    CHECK(a && b && a != b && MePoolFixedAPI.getStruct(&fixed) == nullptr);
    CHECK((uintptr_t)a % 16 == 0 && (uintptr_t)b % 16 == 0);
    MePoolFixedAPI.putStruct(&fixed, a);
    CHECK(MePoolFixedAPI.getUsed(&fixed) == 1 && MePoolFixedAPI.getUnused(&fixed) == 1);
    CHECK(MePoolFixedAPI.getStruct(&fixed) == a);
    MePoolFixedAPI.destroy(&fixed);

    MePool heap;
    MePoolMallocAPI.init(&heap, 3, 40, 0);
    void* c = MePoolMallocAPI.getStruct(&heap);
    CHECK(c && MePoolMallocAPI.getUsed(&heap) == 1);
    MePoolMallocAPI.putStruct(&heap, c);
    CHECK(MePoolMallocAPI.getUsed(&heap) == 0);
    MePoolMallocAPI.destroy(&heap);
    return 0;
}
