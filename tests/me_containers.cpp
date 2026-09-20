#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <MeHash.h>
#include <MeIDPool.h>
#include <MeStream.h>

#include "check.h"

static void* key(std::intptr_t i)
{
    return reinterpret_cast<void*>(i);
}

static int keysFreed = 0;

static void MEAPI freeKey(void* const k)
{
    ++keysFreed;
    std::free(k);
}

int main()
{
    MeHash* h = MeHashCreate(7);
    char a[] = "alpha", b[] = "beta";
    int x = 1, y = 2;
    CHECK(MeHashInsert(a, &x, h) == NULL);
    CHECK(MeHashInsert(b, &y, h) == NULL);
    CHECK(MeHashLookup("alpha", h) == &x);
    CHECK(MeHashPopulation(h) == 2);
    CHECK(MeHashInsert("beta", &x, h) == &y);
    CHECK(MeHashPopulation(h) == 2);
    MeHashIterator it;
    MeHashInitIterator(&it, h);
    int seen = 0;
    while (MeHashGetDatum(&it))
        ++seen;
    CHECK(seen == 2);
    CHECK(MeHashDelete("alpha", h) == &x);
    CHECK(MeHashLookup("alpha", h) == NULL);
    MeHashDestroy(h);

    MeHash* owned = MeHashCreate(5);
    MeHashSetKeyFreeFunc(owned, freeKey);
    char* first = static_cast<char*>(std::malloc(4));
    char* second = static_cast<char*>(std::malloc(4));
    std::strcpy(first, "key");
    std::strcpy(second, "key");
    MeHashInsert(first, &x, owned);
    CHECK(MeHashInsert(second, &y, owned) == &x);
    CHECK(keysFreed == 1 && MeHashPopulation(owned) == 1);
    MeHashInitIterator(&it, owned);
    CHECK(MeHashGetDatum(&it) == &y && it.bucket->key == second);
    MeHashDestroy(owned);
    CHECK(keysFreed == 2);

    MeHash* ints = MeHashCreate(3);
    MeHashSetHashFunc(ints, MeHashInt);
    MeHashSetKeyCompareFunc(ints, MeHashIntCompare);
    for (std::intptr_t i = 0; i < 20; ++i)
        MeHashInsert(key(i), key(i * 10), ints);
    CHECK(MeHashLookup(key(13), ints) == key(130));
    CHECK(MeHashPopulation(ints) == 20);
    MeHashReset(ints);
    CHECK(MeHashPopulation(ints) == 0);
    MeHashDestroy(ints);

    MeIDPool* pool = MeIDPoolCreate();
    CHECK(MeIDPoolIsEmpty(pool));
    for (int i = 0; i < 300; ++i)
        CHECK(MeIDPoolRequestID(pool) == i);
    MeIDPoolReturnID(pool, 42);
    CHECK(MeIDPoolRequestID(pool) == 42);
    MeIDPool* copy = MeIDPoolCreate();
    MeIDPoolCopy(copy, pool);
    CHECK(MeIDPoolRequestID(copy) == 300);
    MeIDPoolReset(pool);
    CHECK(MeIDPoolIsEmpty(pool));
    CHECK(MeIDPoolRequestID(pool) == 0);
    MeIDPoolDestroy(copy);
    MeIDPoolDestroy(pool);

    MeStream s = MeStreamOpenAsMemBuffer(4);
    char text[] = "first line\nsecond\n";
    CHECK(MeStreamWrite(text, 1, std::strlen(text), s) == std::strlen(text));
    CHECK(s->bufLength == std::strlen(text));
    MeStreamRewind(s);
    char line[64];
    CHECK(MeStreamReadLine(line, sizeof(line), s) && std::strcmp(line, "first line\n") == 0);
    char word[6] = {0};
    CHECK(MeStreamRead(word, 1, 6, s) == 6 && std::memcmp(word, "second", 6) == 0);
    MeStreamMemBufferFreeSlackSpace(s);
    CHECK(s->bufSize == s->bufLength);
    MeStreamClose(s);

    MeStream m = MeStreamCreateFromMemBuffer(text, 5, 5);
    CHECK(MeStreamRead(word, 1, 6, m) == 5);
    MeStreamClose(m);
    CHECK(MeStreamOpen("this/file/does/not/exist", kMeOpenModeRDONLY) == NULL);
    return 0;
}
