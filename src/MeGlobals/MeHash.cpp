#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <MeHash.h>

namespace {

MeHashBucket** find(const void* key, const MeHash* table)
{
    MeHashBucket** link = &table->table[table->hash(key, table->size)];
    while (*link && table->compare((*link)->key, key) != 0)
        link = &(*link)->next;
    return link;
}

void release(MeHash* table, MeHashBucket* bucket)
{
    if (table->freeKey)
        table->freeKey(bucket->key);
    if (table->freeDatum)
        table->freeDatum(bucket->datum);
    std::free(bucket);
}

} // namespace

extern "C" {

int MEAPI MeHashString(const void* i, int size)
{
    unsigned int h = 5381;
    for (const unsigned char* s = static_cast<const unsigned char*>(i); *s; ++s)
        h = h * 33 + *s;
    return static_cast<int>(h % static_cast<unsigned int>(size));
}

// Integer keys are stored in the key pointer itself, so no memory is needed per key.
int MEAPI MeHashInt(const void* i, int size)
{
    return static_cast<int>(static_cast<std::uintptr_t>(reinterpret_cast<std::intptr_t>(i)) % static_cast<unsigned int>(size));
}

int MEAPI MeHashStringCompare(const void* k1, const void* k2)
{
    return std::strcmp(static_cast<const char*>(k1), static_cast<const char*>(k2));
}

int MEAPI MeHashIntCompare(const void* k1, const void* k2)
{
    return k1 != k2;
}

MeHash* MEAPI MeHashCreate(int size)
{
    MeHash* table = static_cast<MeHash*>(std::calloc(1, sizeof(MeHash)));
    table->size = size > 0 ? size : 1;
    table->hash = MeHashString;
    table->compare = MeHashStringCompare;
    table->table = static_cast<MeHashBucket**>(std::calloc(table->size, sizeof(MeHashBucket*)));
    return table;
}

void MEAPI MeHashReset(MeHash* table)
{
    for (int i = 0; i < table->size; ++i)
    {
        while (MeHashBucket* bucket = table->table[i])
        {
            table->table[i] = bucket->next;
            release(table, bucket);
        }
    }
    table->population = 0;
    table->collisions = 0;
}

void MEAPI MeHashDestroy(MeHash* table)
{
    if (!table)
        return;
    MeHashReset(table);
    std::free(table->table);
    std::free(table);
}

void* MEAPI MeHashInsert(const void* key, const void* datum, MeHash* table)
{
    MeHashBucket** link = find(key, table);
    if (*link)
    {
        void* old = (*link)->datum;
        (*link)->datum = const_cast<void*>(datum);
        return old;
    }
    MeHashBucket*& head = table->table[table->hash(key, table->size)];
    if (head)
        ++table->collisions;
    MeHashBucket* bucket = static_cast<MeHashBucket*>(std::malloc(sizeof(MeHashBucket)));
    bucket->key = const_cast<void*>(key);
    bucket->datum = const_cast<void*>(datum);
    bucket->next = head;
    head = bucket;
    ++table->population;
    return nullptr;
}

void* MEAPI MeHashDelete(const void* key, MeHash* table)
{
    MeHashBucket** link = find(key, table);
    MeHashBucket* bucket = *link;
    if (!bucket)
        return nullptr;
    *link = bucket->next;
    void* datum = bucket->datum;
    if (table->freeKey)
        table->freeKey(bucket->key);
    std::free(bucket);
    --table->population;
    return datum;
}

void* MEAPI MeHashLookup(const void* key, const MeHash* table)
{
    MeHashBucket* bucket = *find(key, table);
    return bucket ? bucket->datum : nullptr;
}

void MEAPI MeHashSetHashFunc(MeHash* table, MeHashFunc hash)
{
    table->hash = hash;
}

void MEAPI MeHashSetKeyCompareFunc(MeHash* table, MeHashCompareFunc compare)
{
    table->compare = compare;
}

void MEAPI MeHashSetKeyFreeFunc(MeHash* table, MeHashFreeFunc keyFree)
{
    table->freeKey = keyFree;
}

void MEAPI MeHashSetDatumFreeFunc(MeHash* table, MeHashFreeFunc datumFree)
{
    table->freeDatum = datumFree;
}

int MEAPI MeHashPopulation(const MeHash* table)
{
    return table->population;
}

MeHashIterator* MEAPI MeHashInitIterator(MeHashIterator* i, const MeHash* table)
{
    i->table = const_cast<MeHash*>(table);
    i->bucket = nullptr;
    i->index = -1;
    return i;
}

void* MEAPI MeHashGetDatum(MeHashIterator* i)
{
    if (i->bucket)
        i->bucket = i->bucket->next;
    while (!i->bucket && ++i->index < i->table->size)
        i->bucket = i->table->table[i->index];
    return i->bucket ? i->bucket->datum : nullptr;
}

} // extern "C"
