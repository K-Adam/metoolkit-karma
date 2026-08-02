#include "McdImpl.h"

namespace {

McdInteractions* userInteractions(McdModelPair* p)
{
    McdInteractions* i = McdFrameworkGetInteractions(p->model1->frame, McdModelGetGeometryType(p->model1),
                                                     McdModelGetGeometryType(p->model2));
    return i && i->intersectFn ? i : nullptr;
}

btCollisionObjectWrapper wrap(McdModelID m)
{
    btCollisionObject& o = impl(m)->object;
    return btCollisionObjectWrapper(nullptr, o.getCollisionShape(), &o, o.getWorldTransform(), -1, -1);
}

void bulletIntersect(McdModelPair* p, McdIntersectResult* r)
{
    auto* algorithm = static_cast<btCollisionAlgorithm*>(p->m_cachedData);
    if (!algorithm)
        return;
    btCollisionObjectWrapper w1 = wrap(p->model1), w2 = wrap(p->model2);
    btManifoldResult result(&w1, &w2);
    btDispatcherInfo info;
    algorithm->processCollision(&w1, &w2, info, &result);

    btManifoldArray manifolds;
    algorithm->getAllContactManifolds(manifolds);
    const btCollisionObject* object2 = &impl(p->model2)->object;
    int limit = mcdContactLimit(p, r);
    btVector3 sum(0, 0, 0);
    for (int i = 0; i < manifolds.size(); ++i)
    {
        btPersistentManifold* m = manifolds[i];
        // Bullet's normal points from body1 to body0. The original library's points from model2 to model1,
        // although the McdContact header comment says the opposite.
        btScalar sign = m->getBody1() == object2 ? 1 : -1;
        for (int j = 0; j < m->getNumContacts() && r->contactCount < limit; ++j)
        {
            const btManifoldPoint& pt = m->getContactPoint(j);
            btVector3 n = pt.m_normalWorldOnB * sign;
            mcdAddContact(r, (pt.getPositionWorldOnA() + pt.getPositionWorldOnB()) * 0.5f, n, pt.getDistance());
            sum += n;
        }
    }
    if (r->contactCount > 0 && sum.length2() > 0)
        fromBt(sum.normalized(), r->normal);
}

} // namespace

extern "C" {

MeBool MEAPI McdNearby(McdModelPair* p)
{
    const McdGeometryInstance& a = p->model1->mInstance;
    const McdGeometryInstance& b = p->model2->mInstance;
    for (int k = 0; k < 3; ++k)
        if (a.max[k] < b.min[k] || b.max[k] < a.min[k])
            return 0;
    return 1;
}

void MEAPI McdHello(McdModelPair* p)
{
    p->phase = kMcdFFStateHello;
    McdFrameworkID f = p->model1->frame;
    if (f->mHelloCallbackFnPtr)
        f->mHelloCallbackFnPtr(p);
    if (McdInteractions* i = userInteractions(p))
    {
        if (i->helloFn)
            i->helloFn(p);
        return;
    }
    btCollisionObjectWrapper w1 = wrap(p->model1), w2 = wrap(p->model2);
    p->m_cachedData = impl(f)->dispatcher.findAlgorithm(&w1, &w2, nullptr, BT_CONTACT_POINT_ALGORITHMS);
}

void MEAPI McdGoodbye(McdModelPair* p)
{
    // A pair already said goodbye when its model left the space, and that model may be gone by now.
    if (p->phase == kMcdFFStateGoodbye || p->phase == kMcdFFStateInactive)
        return;
    p->phase = kMcdFFStateGoodbye;
    if (McdInteractions* i = userInteractions(p))
    {
        if (i->goodbyeFn)
            i->goodbyeFn(p);
        return;
    }
    if (auto* algorithm = static_cast<btCollisionAlgorithm*>(p->m_cachedData))
    {
        algorithm->~btCollisionAlgorithm();
        impl(p->model1->frame)->dispatcher.freeCollisionAlgorithm(algorithm);
        p->m_cachedData = nullptr;
    }
}

MeBool MEAPI McdIntersect(McdModelPair* p, McdIntersectResult* r)
{
    r->pair = p;
    r->contactCount = 0;
    r->touch = 0;
    r->normal[0] = r->normal[1] = r->normal[2] = 0;
    if (p->phase == kMcdFFStateHello)
        p->phase = kMcdFFStateStaying;

    if (McdInteractions* i = userInteractions(p))
        r->touch = i->intersectFn(p, r);
    else
    {
        bulletIntersect(p, r);
        r->touch = r->contactCount > 0;
    }

    if (r->touch)
    {
        if (p->model1->mIntersectFn)
            p->model1->mIntersectFn(p->model1, r);
        if (p->model2->mIntersectFn)
            p->model2->mIntersectFn(p->model2, r);
    }
    return r->touch != 0;
}

void MEAPI McdHelloEach(McdModelPairContainer* c)
{
    for (int i = c->helloFirst; i < c->helloEndStayingFirst; ++i)
        McdHello(c->array[i]);
}

void MEAPI McdGoodbyeEach(McdModelPairContainer* c)
{
    for (int i = c->goodbyeFirst; i < c->goodbyeEnd; ++i)
        McdGoodbye(c->array[i]);
}

MeBool MEAPI McdIntersectEach(McdModelPairContainer* pairs, McdModelPairContainerIterator* it,
                              McdIntersectResult* resultArray, int resultArraySize, int* resultCount,
                              McdContact* contactArray, int contactArraySize, int* contactCount)
{
    *resultCount = 0;
    *contactCount = 0;
    int i = pairs->helloFirst + it->count;
    for (; i < pairs->stayingEnd; ++i)
    {
        if (*resultCount >= resultArraySize || *contactCount >= contactArraySize)
            break;
        McdIntersectResult& r = resultArray[*resultCount];
        r.contacts = contactArray + *contactCount;
        r.contactMaxCount = contactArraySize - *contactCount;
        if (McdIntersect(pairs->array[i], &r))
        {
            ++*resultCount;
            *contactCount += r.contactCount;
        }
    }
    it->count = i - pairs->helloFirst;
    return i < pairs->stayingEnd;
}

} // extern "C"
