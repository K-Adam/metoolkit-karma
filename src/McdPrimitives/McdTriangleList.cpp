#include <algorithm>
#include <cmath>
#include <vector>

#include <BulletCollision/CollisionShapes/btTriangleShape.h>

#include "Primitive.h"

// A triangle list has no Bullet shape. Its triangles come from the user callback on every intersect call, in
// the triangle list model's frame, and each is collided against the other model's convex shape.

namespace {

struct TriListImpl
{
    McdTriangleList pub;
    std::vector<McdUserTriangle> buffer;
};

TriListImpl* get(McdGeometryID g) { return reinterpret_cast<TriListImpl*>(g); }

void setBox(McdTriangleList& t, const MeVector3 min, const MeVector3 max)
{
    for (int k = 0; k < 3; ++k)
    {
        t.center[k] = (min[k] + max[k]) / 2;
        t.radius[k] = (max[k] - min[k]) / 2;
    }
}

// Rewrites the normals of the contacts from first on according to the triangle's flags.
void applyFlags(const McdUserTriangle& t, const btTransform& listTM, bool listFirst, McdIntersectResult* r, int first)
{
    if (!t.normal)
        return;
    btVector3 face = listTM.getBasis() * toBt(*t.normal);
    // Contact normals point towards model1, so away from the triangle only when the other model is model1.
    btScalar toAway = listFirst ? -1 : 1;
    for (int i = first; i < r->contactCount; ++i)
    {
        btVector3 away = toBt(r->contacts[i].normal) * toAway;
        if (!(t.flags & kMcdTriangleUseSmallestPenetration))
            away = away.dot(face) < 0 ? -face : face;
        if (!(t.flags & kMcdTriangleTwoSided) && away.dot(face) < 0)
            away = -away;
        fromBt(away * toAway, r->contacts[i].normal);
    }
}

} // namespace

int MEAPI mcdTriangleListIntersect(McdModelPair* p, McdIntersectResult* r)
{
    bool listFirst = McdModelGetGeometryType(p->model1) == kMcdGeometryTypeTriangleList;
    McdModelID list = listFirst ? p->model1 : p->model2;
    McdModelID other = listFirst ? p->model2 : p->model1;
    TriListImpl* tl = get(McdModelGetGeometry(list));
    btCollisionObject& otherObject = impl(other)->object;
    if (!tl->pub.triangleListGenerator || !otherObject.getCollisionShape()->isConvex())
        return 0;

    btTransform listTM = toBtTransform(McdModelGetTransformPtr(list));
    MeVector3 center, localCenter;
    MeReal radius;
    McdModelGetBSphere(other, center, &radius);
    fromBt(listTM.inverse() * toBt(center), localCenter);
    tl->pub.list = tl->buffer.data();
    int count = tl->pub.triangleListGenerator(p, tl->pub.list, localCenter,
                                              radius + McdModelGetContactTolerance(other), tl->pub.triangleMaxCount);
    // The callback may point list at its own array, otherwise only the buffer's slots exist.
    if (tl->pub.list == tl->buffer.data())
        count = std::min(count, tl->pub.triangleMaxCount);

    btCollisionDispatcher& dispatcher = impl(list->frame)->dispatcher;
    btCollisionObjectWrapper otherWrap = mcdWrap(other);
    btDispatcherInfo info;
    int limit = mcdContactLimit(p, r);
    btVector3 sum(0, 0, 0);
    for (int i = 0; i < count && r->contactCount < limit; ++i)
    {
        const McdUserTriangle& t = tl->pub.list[i];
        btTriangleShape shape(listTM * toBt(*t.vertices[0]), listTM * toBt(*t.vertices[1]),
                              listTM * toBt(*t.vertices[2]));
        // Bullet inflates convex shapes by their margin, which would make the triangle collide early.
        shape.setMargin(0);
        btCollisionObject triObject;
        triObject.setCollisionShape(&shape);
        triObject.setUserPointer(list);
        btCollisionObjectWrapper triWrap(nullptr, &shape, &triObject, triObject.getWorldTransform(), -1, -1);
        btCollisionAlgorithm* algorithm =
            dispatcher.findAlgorithm(&otherWrap, &triWrap, nullptr, BT_CONTACT_POINT_ALGORITHMS);
        btManifoldResult result(&otherWrap, &triWrap);
        result.m_closestPointDistanceThreshold = mcdPairTolerance(p);
        algorithm->processCollision(&otherWrap, &triWrap, info, &result);
        int first = r->contactCount;
        mcdAddManifoldContacts(algorithm, listFirst ? &triObject : &otherObject, result.m_closestPointDistanceThreshold,
                               r, limit);
        algorithm->~btCollisionAlgorithm();
        dispatcher.freeCollisionAlgorithm(algorithm);

        applyFlags(t, listTM, listFirst, r, first);
        for (int c = first; c < r->contactCount; ++c)
        {
            if (listFirst)
                r->contacts[c].element1.ptr = t.triangleData.ptr;
            else
                r->contacts[c].element2.ptr = t.triangleData.ptr;
            sum += toBt(r->contacts[c].normal);
        }
    }
    mcdSetMeanNormal(r, sum);
    return r->contactCount > 0;
}

void mcdRegisterTriangleListInteraction(McdFramework* frame, int type)
{
    mcdSetInteractionsBoth(frame, type, kMcdGeometryTypeTriangleList,
                           mcdIntersectInteraction(mcdTriangleListIntersect));
}

extern "C" {

MeI16 MEAPI McdTriangleListGetTypeId()
{
    return kMcdGeometryTypeTriangleList;
}

void MEAPI McdTriangleListRegisterType(McdFramework* frame)
{
    McdFrameworkRegisterGeometryType(frame, kMcdGeometryTypeTriangleList, const_cast<char*>("TriangleList"),
                                     McdTriangleListDestroy, McdTriangleListUpdateAABB, McdTriangleListGetBSphere,
                                     McdTriangleListMaximumPoint, McdTriangleListGetMassProperties, nullptr);
}

McdTriangleListID MEAPI McdTriangleListCreate(McdFramework* frame, MeVector3 min, MeVector3 max,
                                              int triangleMaxCount, McdTriangleListFnPtr f)
{
    auto* t = new TriListImpl{};
    McdGeometryInit(&t->pub.m_g, frame, kMcdGeometryTypeTriangleList);
    setBox(t->pub, min, max);
    t->pub.triangleListGenerator = f;
    McdTriangleListSetMaxTriangles(&t->pub.m_g, triangleMaxCount);
    return &t->pub.m_g;
}

void MEAPI McdTriangleListDestroy(McdGeometryID g)
{
    McdGeometryDeinit(g);
    delete get(g);
}

void MEAPI McdTriangleListSetMaxTriangles(McdTriangleListID g, int max)
{
    TriListImpl* t = get(g);
    t->buffer.assign((size_t)std::max(max, 0), McdUserTriangle{});
    t->pub.triangleMaxCount = max;
    t->pub.list = t->buffer.data();
}

int MEAPI McdTriangleListGetMaxTriangles(McdTriangleListID g)
{
    return get(g)->pub.triangleMaxCount;
}

void MEAPI McdTriangleListSetBoundingBox(McdTriangleListID g, MeVector3 min, MeVector3 max)
{
    setBox(get(g)->pub, min, max);
}

void MEAPI McdTriangleListGetBoundingBox(McdTriangleListID g, MeVector3 min, MeVector3 max)
{
    const McdTriangleList& t = get(g)->pub;
    for (int k = 0; k < 3; ++k)
    {
        min[k] = t.center[k] - t.radius[k];
        max[k] = t.center[k] + t.radius[k];
    }
}

void MEAPI McdTriangleListSetGenerator(McdTriangleListID g, McdTriangleListFnPtr f)
{
    get(g)->pub.triangleListGenerator = f;
}

McdTriangleListFnPtr MEAPI McdTriangleListGetGenerator(McdTriangleListID g)
{
    return get(g)->pub.triangleListGenerator;
}

void* MEAPI McdTriangleListGetUserData(McdTriangleListID g)
{
    return get(g)->pub.userData;
}

void MEAPI McdTriangleListSetUserData(McdTriangleListID g, void* data)
{
    get(g)->pub.userData = data;
}

void MEAPI McdTriangleListUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool)
{
    const McdTriangleList& t = get(ins->mGeometry)->pub;
    btTransform tm = toBtTransform(&finalTM[0][0]);
    btVector3 c = tm * toBt(t.center);
    btMatrix3x3 abs = tm.getBasis().absolute();
    btVector3 e = abs * toBt(t.radius);
    fromBt(c - e, ins->min);
    fromBt(c + e, ins->max);
}

void MEAPI McdTriangleListGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    const McdTriangleList& t = get(g)->pub;
    for (int k = 0; k < 3; ++k)
        center[k] = t.center[k];
    *radius = toBt(t.radius).length();
}

void MEAPI McdTriangleListMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    const McdTriangleList& t = get(ins->mGeometry)->pub;
    worldMaximumPoint(ins, inDir, outPoint, [&t](MeReal* d, MeReal* out) {
        for (int k = 0; k < 3; ++k)
            out[k] = t.center[k] + (d[k] < 0 ? -t.radius[k] : t.radius[k]);
    });
}

MeI16 MEAPI McdTriangleListGetMassProperties(McdGeometryID, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    mcdSetDiagonalInertia(relTM, inertia, 0, 0, 0);
    *volume = 0;
    return 0;
}

char* MEAPI McdTriangleListGetTypeName(McdGeometryID)
{
    return const_cast<char*>("TriangleList");
}

void MEAPI McdTriangleListDebugDraw(const McdGeometryID, const MeMatrix4, const MeReal[3])
{
}

} // extern "C"
