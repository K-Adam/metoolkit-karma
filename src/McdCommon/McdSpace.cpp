#include <algorithm>
#include <functional>
#include <set>

#include "McdImpl.h"

namespace {

McdModelID modelOf(btCollisionObject* o)
{
    return static_cast<McdModelID>(o->getUserPointer());
}

McdModelID modelOf(btBroadphaseProxy* proxy)
{
    return modelOf(static_cast<btCollisionObject*>(proxy->m_clientObject));
}

McdModelKey orderedKey(McdModelID a, McdModelID b)
{
    return a->mSpaceID < b->mSpaceID ? McdModelKey(a, b) : McdModelKey(b, a);
}

void syncBroadphase(McdSpaceID s, McdModelID m)
{
    btBroadphaseProxy* proxy = impl(m)->object.getBroadphaseHandle();
    s->broadphase.setAabb(proxy, toBt(m->mInstance.min), toBt(m->mInstance.max), &s->frame->dispatcher);
}

void detach(McdSpaceID s, McdModelID m)
{
    McdModelImpl* mi = impl(m);
    s->world.removeCollisionObject(&mi->object);
    m->mSpace = nullptr;
    m->mSpaceID = -1;
    mi->frozen = false;
    mi->cullTable = nullptr;
    mi->cullIndex = mi->cullId = 0;
}

bool culled(McdModelID a, McdModelID b)
{
    McdModelImpl *ia = impl(a), *ib = impl(b);
    return ia->cullTable && ia->cullTable == ib->cullTable && ia->cullId == ib->cullId &&
           McdCullingTableGet(ia->cullTable, (MeU32)ia->cullIndex, (MeU32)ib->cullIndex);
}

bool pairEnabled(McdModelID a, McdModelID b)
{
    return !impl(a->frame)->disabledPairs.count(mcdUnorderedKey(a, b)) && !culled(a, b);
}

void updatePairs(McdSpaceID s, const std::function<void(McdModelID)>& updateModel)
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
        McdModelID m = modelOf(objects[i]);
        if (!impl(m)->frozen)
        {
            updateModel(m);
            syncBroadphase(s, m);
        }
    }
    // Not computeOverlappingPairs: that would replace the model AABBs with Bullet's own.
    s->broadphase.calculateOverlappingPairs(&s->frame->dispatcher);

    std::set<McdModelKey> current;
    btBroadphasePairArray& overlaps = s->broadphase.getOverlappingPairCache()->getOverlappingPairArray();
    for (int i = 0; i < overlaps.size(); ++i)
    {
        btBroadphaseProxy *p0 = overlaps[i].m_pProxy0, *p1 = overlaps[i].m_pProxy1;
        McdModelID a = modelOf(p0), b = modelOf(p1);
        if ((impl(a)->frozen && impl(b)->frozen) || !pairEnabled(a, b))
            continue;
        // The DBVT pads its leaves and prunes stale pairs lazily, so its pair cache over-reports.
        if (!TestAabbAgainstAabb2(p0->m_aabbMin, p0->m_aabbMax, p1->m_aabbMin, p1->m_aabbMax))
            continue;
        McdModelKey key = orderedKey(a, b);
        auto found = s->pairs.find(key);
        if (found == s->pairs.end())
        {
            if (s->pairs.size() >= s->pairCapacity)
            {
                if (s->poolFullHandler)
                    s->poolFullHandler(key.first, key.second);
                continue;
            }
            McdModelPair* p = McdModelPairCreate(key.first, key.second);
            s->pairs.emplace(key, p);
            s->hello.push_back(p);
        }
        else
            s->staying.push_back(found->second);
        current.insert(key);
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

typedef std::pair<MeReal, McdLineSegIntersectResult> Hit;

// Every model the segment enters, nearest first.
std::vector<Hit> lineSegHits(McdSpaceID s, MeReal* inOrig, MeReal* inDest, McdLineSegIntersectEnableCallback filter,
                             void* filterData)
{
    std::vector<Hit> hits;
    btVector3 from = toBt(inOrig), to = toBt(inDest);
    btCollisionObjectArray& objects = s->world.getCollisionObjectArray();
    for (int i = 0; i < objects.size(); ++i)
    {
        McdModelID m = modelOf(objects[i]);
        btScalar lambda = 1;
        btVector3 normal;
        if (!btRayAabb(from, to, toBt(m->mInstance.min), toBt(m->mInstance.max), lambda, normal) &&
            !TestPointAgainstAabb2(toBt(m->mInstance.min), toBt(m->mInstance.max), from))
            continue;
        if (filter && !filter(m, filterData))
            continue;
        McdLineSegIntersectResult r;
        if (McdLineSegIntersect(m, inOrig, inDest, &r))
            hits.emplace_back(r.distance, r);
    }
    std::stable_sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.first < b.first; });
    return hits;
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
    McdSpaceID s = new _McdSpace(impl(fwk));
    s->frame->spaces.insert(s);
    s->frame->spaceDestroyHook = McdSpaceDestroy;
    s->pairCapacity = (size_t)std::max(pairCount, 0);
    // A new space starts inside a change block so it can be populated right away.
    McdSpaceBeginChanges(s);
    return s;
}

void MEAPI McdSpaceDestroy(McdSpaceID s)
{
    if (!s)
        return;
    s->frame->spaces.erase(s);
    for (auto& kv : s->pairs)
        McdModelPairDestroy(kv.second);
    for (McdModelPair* p : s->removed)
        McdModelPairDestroy(p);
    for (McdModelPair* p : s->goodbye)
        McdModelPairDestroy(p);
    btCollisionObjectArray& objects = s->world.getCollisionObjectArray();
    while (objects.size() > 0)
    {
        detach(s, modelOf(objects[0]));
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

MeBool MEAPI McdSpaceInsertModelWithCulling(McdSpaceID s, McdModelID cm, McdCullingTable* table, int cullingIndex,
                                            int cullingID)
{
    if (!McdSpaceInsertModel(s, cm))
        return 0;
    McdSpaceSetModelCullingParameters(s, cm, table, cullingIndex, cullingID);
    return 1;
}

void MEAPI McdSpaceSetModelCullingParameters(McdSpaceID, McdModelID cm, McdCullingTable* table, int cullingIndex,
                                             int cullingID)
{
    McdModelImpl* mi = impl(cm);
    mi->cullTable = table;
    mi->cullIndex = cullingIndex;
    mi->cullId = cullingID;
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
    // Pending goodbye pairs outlive the model until the next update, so they must not need it any more.
    for (McdModelPair* p : s->goodbye)
        if (p->model1 == cm || p->model2 == cm)
            McdGoodbye(p);
    detach(s, cm);
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
    updatePairs(s, McdModelUpdate);
}

void MEAPI McdSpacePathUpdateAll(McdSpaceID s, MeReal duration)
{
    updatePairs(s, [s, duration](McdModelID m) {
        if (s->aabbFn)
            s->aabbFn(m, duration);
        else
            McdModelUpdatePath(m, duration);
    });
}

void MEAPI McdSpaceSetAABBFn(McdSpaceID s, McdSpaceUpdateAABBFnPtr updateAABBFn)
{
    s->aabbFn = updateAABBFn;
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

int MEAPI McdSpaceGetLineSegIntersections(McdSpaceID space, MeReal* inOrig, MeReal* inDest,
                                          McdLineSegIntersectResult* outList, int inMaxListSize)
{
    std::vector<Hit> hits = lineSegHits(space, inOrig, inDest, nullptr, nullptr);
    int n = std::clamp(inMaxListSize, 0, (int)hits.size());
    for (int i = 0; i < n; ++i)
        outList[i] = hits[i].second;
    return n;
}

int MEAPI McdSpaceGetLineSegFirstIntersection(McdSpaceID space, MeReal* inOrig, MeReal* inDest,
                                              McdLineSegIntersectResult* outResult)
{
    return McdSpaceGetLineSegFirstEnabledIntersection(space, inOrig, inDest, nullptr, nullptr, outResult);
}

int MEAPI McdSpaceGetLineSegFirstEnabledIntersection(McdSpaceID space, MeReal* inOrig, MeReal* inDest,
                                                     McdLineSegIntersectEnableCallback filterCB, void* filterData,
                                                     McdLineSegIntersectResult* outResult)
{
    std::vector<Hit> hits = lineSegHits(space, inOrig, inDest, filterCB, filterData);
    if (hits.empty())
        return 0;
    *outResult = hits[0].second;
    return 1;
}

void MEAPI McdSpaceSetPoolFullHandler(McdSpaceID space, McdSpacePoolErrorFnPtr handler)
{
    space->poolFullHandler = handler;
}

McdSpacePoolErrorFnPtr MEAPI McdSpaceGetPoolFullHandler(McdSpaceID space)
{
    return space->poolFullHandler;
}

MeBool MEAPI McdSpaceEnablePair(McdModelID cm1, McdModelID cm2)
{
    impl(cm1->frame)->disabledPairs.erase(mcdUnorderedKey(cm1, cm2));
    return 1;
}

MeBool MEAPI McdSpaceDisablePair(McdModelID cm1, McdModelID cm2)
{
    impl(cm1->frame)->disabledPairs.insert(mcdUnorderedKey(cm1, cm2));
    return 1;
}

MeBool MEAPI McdSpacePairIsEnabled(McdModelID cm1, McdModelID cm2)
{
    return pairEnabled(cm1, cm2);
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
    return mcdFillContainer(c, it, s->goodbye, s->hello, &s->staying);
}

int MEAPI McdSpaceGetTransitions(McdSpaceID s, McdSpacePairIterator* iter, McdModelPairContainer* a)
{
    return mcdFillContainer(a, iter, s->goodbye, s->hello, nullptr);
}

// Walks Bullet's object array backwards. Removing the current model moves the last one into its slot, which has
// already been visited, so a loop that removes each model it gets still sees them all.
void MEAPI McdSpaceModelIteratorBegin(McdSpaceID s, McdSpaceModelIterator* it)
{
    it->it = s->world.getNumCollisionObjects();
    it->count = 0;
}

MeBool MEAPI McdSpaceGetModel(McdSpaceID s, McdSpaceModelIterator* it, McdModelID* model)
{
    it->it = std::min(it->it, s->world.getNumCollisionObjects());
    if (it->it <= 0)
        return 0;
    *model = modelOf(s->world.getCollisionObjectArray()[--it->it]);
    ++it->count;
    return 1;
}

} // extern "C"
