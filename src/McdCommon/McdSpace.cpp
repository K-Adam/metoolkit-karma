#include <algorithm>
#include <set>

#include "McdImpl.h"

namespace {

typedef std::pair<McdModelID, McdModelID> PairKey;

McdModelID modelOf(btBroadphaseProxy* proxy)
{
    return static_cast<McdModelID>(static_cast<btCollisionObject*>(proxy->m_clientObject)->getUserPointer());
}

PairKey orderedKey(McdModelID a, McdModelID b)
{
    return a->mSpaceID < b->mSpaceID ? PairKey(a, b) : PairKey(b, a);
}

void syncBroadphase(McdSpaceID s, McdModelID m)
{
    btBroadphaseProxy* proxy = impl(m)->object.getBroadphaseHandle();
    s->broadphase.setAabb(proxy, toBt(m->mInstance.min), toBt(m->mInstance.max), &s->frame->dispatcher);
}

} // namespace

extern "C" {

int McdXAxis = 1;
int McdYAxis = 2;
int McdZAxis = 4;
int McdAllAxes = 7;

McdSpaceID MEAPI McdSpaceAxisSortCreate(McdFrameworkID fwk, int axes, int objectCount, int pairCount)
{
    (void)axes;
    (void)objectCount;
    (void)pairCount;
    McdSpaceID s = new _McdSpace(impl(fwk));
    // A new space starts inside a change block so it can be populated right away.
    McdSpaceBeginChanges(s);
    return s;
}

void MEAPI McdSpaceDestroy(McdSpaceID s)
{
    for (auto& kv : s->pairs)
    {
        McdGoodbye(kv.second);
        McdModelPairDestroy(kv.second);
    }
    for (McdModelPair* p : s->removed)
        McdModelPairDestroy(p);
    for (McdModelPair* p : s->goodbye)
        McdModelPairDestroy(p);
    btCollisionObjectArray& objects = s->world.getCollisionObjectArray();
    while (objects.size() > 0)
    {
        auto* m = static_cast<McdModelID>(objects[0]->getUserPointer());
        s->world.removeCollisionObject(objects[0]);
        m->mSpace = nullptr;
        m->mSpaceID = -1;
        impl(m)->frozen = false;
    }
    delete s;
}

// The dynamic AABB tree is maintained incrementally, so there is nothing to build up front.
void MEAPI McdSpaceBuild(McdSpaceID)
{
}

MeBool MEAPI McdSpaceInsertModel(McdSpaceID s, McdModelID cm)
{
    if (!cm || cm->mSpace)
        return 0;
    cm->mSpace = s;
    cm->mSpaceID = s->nextModelId++;
    mcdModelSync(cm);
    s->world.addCollisionObject(&impl(cm)->object);
    syncBroadphase(s, cm);
    return 1;
}

MeBool MEAPI McdSpaceRemoveModel(McdModelID cm)
{
    McdSpaceID s = cm ? cm->mSpace : nullptr;
    if (!s)
        return 0;
    for (auto it = s->pairs.begin(); it != s->pairs.end();)
    {
        if (it->first.first == cm || it->first.second == cm)
        {
            // Release the narrowphase state now, the pair is still reported as goodbye on the next query.
            McdGoodbye(it->second);
            for (std::vector<McdModelPair*>* list : {&s->hello, &s->staying})
                list->erase(std::remove(list->begin(), list->end(), it->second), list->end());
            s->removed.push_back(it->second);
            it = s->pairs.erase(it);
        }
        else
            ++it;
    }
    s->world.removeCollisionObject(&impl(cm)->object);
    cm->mSpace = nullptr;
    cm->mSpaceID = -1;
    impl(cm)->frozen = false;
    return 1;
}

void MEAPI McdSpaceUpdateModel(McdModelID cm)
{
    McdModelUpdate(cm);
    if (cm->mSpace)
        syncBroadphase(cm->mSpace, cm);
}

MeBool MEAPI McdSpaceFreezeModel(McdModelID cm)
{
    if (!cm->mSpace)
        return 0;
    impl(cm)->frozen = true;
    return 1;
}

MeBool MEAPI McdSpaceUnfreezeModel(McdModelID cm)
{
    if (!cm->mSpace)
        return 0;
    impl(cm)->frozen = false;
    return 1;
}

MeBool MEAPI McdSpaceModelIsFrozen(McdModelID cm)
{
    return impl(cm)->frozen;
}

void MEAPI McdSpaceUpdateAll(McdSpaceID s)
{
    for (McdModelPair* p : s->goodbye)
        McdModelPairDestroy(p);
    s->goodbye.swap(s->removed);
    s->removed.clear();
    s->hello.clear();
    s->staying.clear();

    btCollisionObjectArray& objects = s->world.getCollisionObjectArray();
    for (int i = 0; i < objects.size(); ++i)
    {
        auto* m = static_cast<McdModelID>(objects[i]->getUserPointer());
        if (!impl(m)->frozen)
            McdSpaceUpdateModel(m);
    }
    s->world.computeOverlappingPairs();

    std::set<PairKey> current;
    btBroadphasePairArray& overlaps = s->broadphase.getOverlappingPairCache()->getOverlappingPairArray();
    for (int i = 0; i < overlaps.size(); ++i)
    {
        btBroadphaseProxy *p0 = overlaps[i].m_pProxy0, *p1 = overlaps[i].m_pProxy1;
        McdModelID a = modelOf(p0), b = modelOf(p1);
        if (impl(a)->frozen && impl(b)->frozen)
            continue;
        // The DBVT pads its leaves and prunes stale pairs lazily, so its pair cache over-reports.
        if (!TestAabbAgainstAabb2(p0->m_aabbMin, p0->m_aabbMax, p1->m_aabbMin, p1->m_aabbMax))
            continue;
        PairKey key = orderedKey(a, b);
        current.insert(key);
        auto found = s->pairs.find(key);
        if (found == s->pairs.end())
        {
            McdModelPair* p = McdModelPairCreate(key.first, key.second);
            s->pairs.emplace(key, p);
            s->hello.push_back(p);
        }
        else
            s->staying.push_back(found->second);
    }
    for (auto it = s->pairs.begin(); it != s->pairs.end();)
    {
        if (current.count(it->first))
            ++it;
        else
        {
            s->goodbye.push_back(it->second);
            it = s->pairs.erase(it);
        }
    }
}

void MEAPI McdSpaceBeginChanges(McdSpaceID space)
{
    ++space->changing;
}

void MEAPI McdSpaceEndChanges(McdSpaceID space)
{
    if (space->changing > 0)
        --space->changing;
}

int MEAPI McdSpaceIsChanging(McdSpaceID space)
{
    return space->changing > 0;
}

void MEAPI McdSpaceSetUserData(McdSpaceID s, void* data)
{
    s->userData = data;
}

void* MEAPI McdSpaceGetUserData(McdSpaceID s)
{
    return s->userData;
}

int MEAPI McdSpaceGetModelCount(McdSpaceID space)
{
    return space->world.getNumCollisionObjects();
}

void MEAPI McdSpacePairIteratorBegin(McdSpaceID s, McdSpacePairIterator* it)
{
    it->count = 0;
    it->ptr = s;
}

int MEAPI McdSpaceGetPairs(McdSpaceID s, McdSpacePairIterator* it, McdModelPairContainer* c)
{
    McdModelPairContainerReset(c);
    const std::vector<McdModelPair*>* lists[] = {&s->goodbye, &s->hello, &s->staying};
    MeBool (MEAPI * insert[])(McdModelPairContainer*, McdModelPairID) = {
        McdModelPairContainerInsertGoodbyePair, McdModelPairContainerInsertHelloPair,
        McdModelPairContainerInsertStayingPair};
    int index = 0;
    for (int l = 0; l < 3; ++l)
    {
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

} // extern "C"
