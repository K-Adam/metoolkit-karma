#include <BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpa2.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <LinearMath/btTransformUtil.h>

#include "McdImpl.h"

namespace {

McdInteractions* userInteractions(McdModelPair* p)
{
    McdInteractions* i = McdFrameworkGetInteractions(p->model1->frame, McdModelGetGeometryType(p->model1),
                                                     McdModelGetGeometryType(p->model2));
    return i && i->intersectFn ? i : nullptr;
}

} // namespace

int mcdBulletIntersect(McdModelPair* p, McdIntersectResult* r)
{
    auto* algorithm = static_cast<btCollisionAlgorithm*>(p->m_cachedData);
    if (!algorithm)
        return 0;
    btCollisionObjectWrapper w1 = mcdWrap(p->model1), w2 = mcdWrap(p->model2);
    btManifoldResult result(&w1, &w2);
    btDispatcherInfo info;
    algorithm->processCollision(&w1, &w2, info, &result);
    mcdSetMeanNormal(r, mcdAddManifoldContacts(algorithm, &impl(p->model1)->object, r, mcdContactLimit(p, r)));
    return r->contactCount > 0;
}

namespace {

bool pointInside(const btCollisionShape* shape, const btTransform& t, const btVector3& point)
{
    if (shape->isCompound())
    {
        auto* compound = static_cast<const btCompoundShape*>(shape);
        for (int i = 0; i < compound->getNumChildShapes(); ++i)
            if (pointInside(compound->getChildShape(i), t * compound->getChildTransform(i), point))
                return true;
        return false;
    }
    if (!shape->isConvex())
        return false;
    btGjkEpaSolver2::sResults res;
    return btGjkEpaSolver2::SignedDistance(point, 0, static_cast<const btConvexShape*>(shape), t, res) < 0;
}

btTransform predicted(McdModelID m, MeReal time)
{
    btTransform t = toBtTransform(m->mInstance.mTM), out;
    btVector3 lin = m->linearVelocity ? toBt(m->linearVelocity) : btVector3(0, 0, 0);
    btVector3 ang = m->angularVelocity ? toBt(m->angularVelocity) : btVector3(0, 0, 0);
    btTransformUtil::integrateTransform(t, lin, ang, time, out);
    return out;
}

} // namespace

extern "C" {

MeBool MEAPI McdNearby(McdModelPair* p)
{
    return McdGeometryInstanceOverlap(&p->model1->mInstance, &p->model2->mInstance);
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
    mcdBulletHello(p);
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
    mcdBulletGoodbye(p);
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
        r->touch = mcdBulletIntersect(p, r);

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

MeBool MEAPI McdIntersectAt(McdModelPair* p, McdIntersectResult* result, MeReal time)
{
    McdModelID models[2] = {p->model1, p->model2};
    MeMatrix4 moved[2];
    MeMatrix4Ptr saved[2];
    for (int i = 0; i < 2; ++i)
    {
        btTransform t = predicted(models[i], time);
        fromBtTransform(t, moved[i]);
        saved[i] = models[i]->mInstance.mTM;
        models[i]->mInstance.mTM = moved[i];
        impl(models[i])->object.setWorldTransform(t);
        McdGeometryInstanceUpdateAABB(&models[i]->mInstance, moved[i], 1);
    }
    MeBool touch = McdIntersect(p, result);
    for (int i = 0; i < 2; ++i)
    {
        models[i]->mInstance.mTM = saved[i];
        mcdModelSync(models[i]);
    }
    return touch;
}

MeBool MEAPI McdSafeTime(McdModelPair* p, MeReal maxTime, McdSafeTimeResult* result)
{
    result->pair = p;
    result->time = maxTime;
    McdInteractions* i = McdFrameworkGetInteractions(p->model1->frame, McdModelGetGeometryType(p->model1),
                                                     McdModelGetGeometryType(p->model2));
    if (i && i->safetimeFn)
        return i->safetimeFn(p, maxTime, result) != 0;
    const btCollisionShape* s1 = impl(p->model1)->object.getCollisionShape();
    const btCollisionShape* s2 = impl(p->model2)->object.getCollisionShape();
    if (!s1->isConvex() || !s2->isConvex())
        return 0;
    btVoronoiSimplexSolver simplex;
    btGjkEpaPenetrationDepthSolver depth;
    btContinuousConvexCollision cast(static_cast<const btConvexShape*>(s1), static_cast<const btConvexShape*>(s2),
                                     &simplex, &depth);
    btConvexCast::CastResult hit;
    if (cast.calcTimeOfImpact(toBtTransform(p->model1->mInstance.mTM), predicted(p->model1, maxTime),
                              toBtTransform(p->model2->mInstance.mTM), predicted(p->model2, maxTime), hit))
        result->time = hit.m_fraction * maxTime;
    return 1;
}

unsigned int MEAPI McdLineSegIntersect(const McdModelID cm, MeReal* const inOrig, MeReal* const inDest,
                                       McdLineSegIntersectResult* outOverlap)
{
    if (McdLineSegIntersectFnPtr fn = McdGeometryGetLineSegIntersectFnPtr(McdModelGetGeometry(cm)))
        return fn(cm, inOrig, inDest, outOverlap) != 0;
    btCollisionObject& o = impl(cm)->object;
    btVector3 from = toBt(inOrig), to = toBt(inDest);
    outOverlap->model = cm;
    if (pointInside(o.getCollisionShape(), o.getWorldTransform(), from))
    {
        fromBt(from, outOverlap->position);
        outOverlap->normal[0] = outOverlap->normal[1] = outOverlap->normal[2] = 0;
        outOverlap->distance = 0;
        return 1;
    }
    btCollisionWorld::ClosestRayResultCallback cb(from, to);
    btCollisionWorld::rayTestSingle(btTransform(btQuaternion::getIdentity(), from),
                                    btTransform(btQuaternion::getIdentity(), to), &o, o.getCollisionShape(),
                                    o.getWorldTransform(), cb);
    if (!cb.hasHit())
        return 0;
    fromBt(cb.m_hitPointWorld, outOverlap->position);
    fromBt(cb.m_hitNormalWorld.normalized(), outOverlap->normal);
    outOverlap->distance = cb.m_closestHitFraction * (to - from).length();
    return 1;
}

} // extern "C"

btCollisionObjectWrapper mcdWrap(McdModelID m)
{
    btCollisionObject& o = impl(m)->object;
    return btCollisionObjectWrapper(nullptr, o.getCollisionShape(), &o, o.getWorldTransform(), -1, -1);
}

btVector3 mcdAddManifoldContacts(btCollisionAlgorithm* algorithm, const btCollisionObject* object1,
                                 McdIntersectResult* r, int limit)
{
    btManifoldArray manifolds;
    algorithm->getAllContactManifolds(manifolds);
    btVector3 sum(0, 0, 0);
    for (int i = 0; i < manifolds.size(); ++i)
    {
        btPersistentManifold* m = manifolds[i];
        // Bullet's normal points from body1 to body0. The original library's points from model2 to model1,
        // although the McdContact header comment says the opposite.
        btScalar sign = m->getBody0() == object1 ? 1 : -1;
        for (int j = 0; j < m->getNumContacts() && r->contactCount < limit; ++j)
        {
            const btManifoldPoint& pt = m->getContactPoint(j);
            btVector3 n = pt.m_normalWorldOnB * sign;
            mcdAddContact(r, (pt.getPositionWorldOnA() + pt.getPositionWorldOnB()) * 0.5f, n, pt.getDistance());
            sum += n;
        }
    }
    return sum;
}

int mcdIntersectOnce(McdModelPair* p, McdIntersectResult* r)
{
    r->pair = p;
    r->contactCount = 0;
    r->normal[0] = r->normal[1] = r->normal[2] = 0;
    if (McdInteractions* i = userInteractions(p))
    {
        if (i->helloFn)
            i->helloFn(p);
        r->touch = i->intersectFn(p, r);
        if (i->goodbyeFn)
            i->goodbyeFn(p);
        return r->touch;
    }
    mcdBulletHello(p);
    r->touch = mcdBulletIntersect(p, r);
    mcdBulletGoodbye(p);
    return r->touch;
}

void mcdBulletHello(McdModelPair* p)
{
    btCollisionObjectWrapper w1 = mcdWrap(p->model1), w2 = mcdWrap(p->model2);
    p->m_cachedData =
        impl(p->model1->frame)->dispatcher.findAlgorithm(&w1, &w2, nullptr, BT_CONTACT_POINT_ALGORITHMS);
}

void mcdBulletGoodbye(McdModelPair* p)
{
    if (auto* algorithm = static_cast<btCollisionAlgorithm*>(p->m_cachedData))
    {
        algorithm->~btCollisionAlgorithm();
        impl(p->model1->frame)->dispatcher.freeCollisionAlgorithm(algorithm);
        p->m_cachedData = nullptr;
    }
}

