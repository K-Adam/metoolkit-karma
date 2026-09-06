#include <algorithm>

#include <McdModelPairManager.h>

#include "McdImpl.h"

namespace {

struct ManagerImpl
{
    McdModelPairManager pub{};
    int size = 0;
    std::map<McdModelKey, McdModelPair*> active;
    std::vector<McdModelPair*> hello, goodbye;

    std::vector<McdModelPair*> staying() const
    {
        std::vector<McdModelPair*> s;
        for (auto& kv : active)
            if (std::find(hello.begin(), hello.end(), kv.second) == hello.end())
                s.push_back(kv.second);
        return s;
    }
};

ManagerImpl* impl(McdModelPairManagerID m) { return reinterpret_cast<ManagerImpl*>(m); }

} // namespace

extern "C" {

McdModelPairManagerID MEAPI McdModelPairManagerCreate(int size)
{
    auto* m = new ManagerImpl();
    m->size = size;
    return &m->pub;
}

void MEAPI McdModelPairManagerDestroy(McdModelPairManager* manager)
{
    ManagerImpl* m = impl(manager);
    for (auto& kv : m->active)
    {
        McdGoodbye(kv.second);
        McdModelPairDestroy(kv.second);
    }
    for (McdModelPair* p : m->goodbye)
        McdModelPairDestroy(p);
    delete m;
}

MeBool MEAPI McdModelPairManagerActivate(McdModelPairManagerID manager, McdModelID model1, McdModelID model2)
{
    ManagerImpl* m = impl(manager);
    McdModelKey key = mcdUnorderedKey(model1, model2);
    if (m->active.count(key))
        return 0;
    if ((int)m->active.size() >= m->size)
    {
        if (manager->poolFullHandler)
            manager->poolFullHandler(model1, model2);
        return 0;
    }
    McdModelPair* p = McdModelPairCreate(model1, model2);
    m->active.emplace(key, p);
    m->hello.push_back(p);
    return 1;
}

MeBool MEAPI McdModelPairManagerDeactivate(McdModelPairManagerID manager, McdModelID model1, McdModelID model2)
{
    ManagerImpl* m = impl(manager);
    auto found = m->active.find(mcdUnorderedKey(model1, model2));
    if (found == m->active.end())
        return 0;
    McdModelPair* p = found->second;
    m->active.erase(found);
    m->hello.erase(std::remove(m->hello.begin(), m->hello.end(), p), m->hello.end());
    // A pair nobody said hello to needs no goodbye.
    if (p->phase == kMcdFFStateInactive)
        McdModelPairDestroy(p);
    else
        m->goodbye.push_back(p);
    return 1;
}

McdModelPairID MEAPI McdModelPairManagerGetPair(McdModelPairManagerID manager, McdModelID m1, McdModelID m2)
{
    ManagerImpl* m = impl(manager);
    auto found = m->active.find(mcdUnorderedKey(m1, m2));
    return found == m->active.end() ? nullptr : found->second;
}

void MEAPI McdModelPairManagerFlush(McdModelPairManagerID manager)
{
    ManagerImpl* m = impl(manager);
    for (McdModelPair* p : m->goodbye)
        McdModelPairDestroy(p);
    m->goodbye.clear();
    m->hello.clear();
}

void MEAPI McdModelPairManagerResetIterator(McdModelPairManagerID manager, McdSpacePairIterator* iterator)
{
    iterator->count = 0;
    iterator->ptr = manager;
}

MeBool MEAPI McdModelPairManagerGetTransitions(McdModelPairManagerID manager, McdSpacePairIterator* iterator,
                                               McdModelPairContainer* container)
{
    ManagerImpl* m = impl(manager);
    return (MeBool)mcdFillContainer(container, iterator, m->goodbye, m->hello, nullptr);
}

MeBool MEAPI McdModelPairManagerGetPairs(McdModelPairManagerID manager, McdSpacePairIterator* iterator,
                                         McdModelPairContainer* container)
{
    ManagerImpl* m = impl(manager);
    std::vector<McdModelPair*> staying = m->staying();
    return (MeBool)mcdFillContainer(container, iterator, m->goodbye, m->hello, &staying);
}

int MEAPI McdModelPairManagerGetSize(McdModelPairManagerID manager)
{
    return impl(manager)->size;
}

void MEAPI McdModelPairManagerSetPoolFullHandler(McdModelPairManagerID manager, McdSpacePoolErrorFnPtr handler)
{
    manager->poolFullHandler = handler;
}

McdSpacePoolErrorFnPtr MEAPI McdModelPairManagerGetPoolFullHandler(McdModelPairManagerID manager)
{
    return manager->poolFullHandler;
}

} // extern "C"
