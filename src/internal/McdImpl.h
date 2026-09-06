#pragma once

#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>

#include <map>
#include <set>
#include <vector>

#include <Mcd.h>
#include <McdModelPair.h>

#include "BulletConvert.h"

// Built-in geometry types collide through Bullet. A type without a shape function relies on the
// McdInteractions table instead, which is how user geometry plug-ins work in the original.
typedef btCollisionShape* (*McdShapeFn)(McdGeometryID);

typedef std::pair<McdModelID, McdModelID> McdModelKey;

inline McdModelKey mcdUnorderedKey(McdModelID a, McdModelID b) { return a < b ? McdModelKey(a, b) : McdModelKey(b, a); }

struct McdFrameworkImpl
{
    McdFramework pub{};
    btDefaultCollisionConfiguration config;
    btCollisionDispatcher dispatcher{&config};
    // Stands in for geometry without a Bullet shape, so its models can still live in a space.
    btEmptyShape emptyShape;
    std::vector<McdGeometryVTable> vtables;
    std::vector<McdShapeFn> shapeFns;
    std::vector<McdInteractions> interactions;
    std::set<McdModelKey> disabledPairs;
    McdRequest request{};
};

inline McdFrameworkImpl* impl(McdFrameworkID f) { return reinterpret_cast<McdFrameworkImpl*>(f); }

void mcdRegisterShapeFn(McdFrameworkID frame, int typeId, McdShapeFn fn);
btCollisionShape* mcdGeometryShape(McdGeometryID g);

// Child instances of aggregate models, allocated by the aggregate and freed with the model.
struct McdChildInstance
{
    McdGeometryInstance ins{};
    MeMatrix4 tm;
};

void mcdFreeChildInstances(McdGeometryInstance* ins);

struct McdModelImpl
{
    McdModel pub{};
    btCollisionObject object;
    MeMatrix4 ownTM;
    MeMatrix4 ownRelTM;
    MeMatrix4 compoundTM;
    bool frozen = false;
    McdCullingTable* cullTable = nullptr;
    int cullIndex = 0;
    int cullId = 0;
};

inline McdModelImpl* impl(McdModelID m) { return reinterpret_cast<McdModelImpl*>(m); }

void mcdModelSync(McdModelID m);

inline int mcdContactLimit(McdModelPair* p, McdIntersectResult* r)
{
    int limit = r->contactMaxCount;
    if (p->request && p->request->contactMaxCount > 0 && p->request->contactMaxCount < limit)
        limit = p->request->contactMaxCount;
    return limit;
}

inline void mcdAddContact(McdIntersectResult* r, const btVector3& position, const btVector3& normal, MeReal separation)
{
    McdContact& c = r->contacts[r->contactCount++];
    fromBt(position, c.position);
    fromBt(normal, c.normal);
    c.separation = separation;
    c.dims = 0;
    c.element1.ptr = c.element2.ptr = nullptr;
}

btCollisionObjectWrapper mcdWrap(McdModelID m);

// Copies an algorithm's manifold points into r with normals pointing towards object1, up to limit, and returns
// the sum of the normals added.
btVector3 mcdAddManifoldContacts(btCollisionAlgorithm* algorithm, const btCollisionObject* object1,
                                 McdIntersectResult* r, int limit);

struct _McdSpace
{
    McdFrameworkImpl* frame;
    btDbvtBroadphase broadphase;
    btCollisionWorld world;
    std::map<McdModelKey, McdModelPair*> pairs;
    std::vector<McdModelPair*> goodbye, hello, staying;
    std::vector<McdModelPair*> removed;
    int nextModelId = 0;
    int changing = 0;
    size_t pairCapacity = 0;
    void* userData = nullptr;
    McdSpaceUpdateAABBFnPtr aabbFn = nullptr;
    McdSpacePoolErrorFnPtr poolFullHandler = nullptr;

    explicit _McdSpace(McdFrameworkImpl* f) : frame(f), world(&f->dispatcher, &broadphase, &f->config) {}
};

// Fills c with goodbye, hello and (when given) staying pairs, resuming after the it->count pairs already
// returned. Returns 1 while pairs remain that did not fit.
int mcdFillContainer(McdModelPairContainer* c, McdSpacePairIterator* it, const std::vector<McdModelPair*>& goodbye,
                     const std::vector<McdModelPair*>& hello, const std::vector<McdModelPair*>* staying);

// Triangle list against any model with a convex Bullet shape, defined in McdPrimitives. Geometry libraries
// register it for their own convex types.
int MEAPI mcdTriangleListIntersect(McdModelPair* p, McdIntersectResult* r);
void mcdRegisterTriangleListInteraction(McdFramework* frame, int type);

// One narrowphase test with its own hello and goodbye and without model callbacks, for pairs that exist only
// for the duration of a call.
int mcdIntersectOnce(McdModelPair* p, McdIntersectResult* r);

// Bullet's narrowphase for a pair, with its algorithm cached in m_cachedData between hello and goodbye.
void mcdBulletHello(McdModelPair* p);
void mcdBulletGoodbye(McdModelPair* p);
int mcdBulletIntersect(McdModelPair* p, McdIntersectResult* r);

// Mass properties are reported for unit density: inertia is per unit mass and volume is returned separately.
inline void mcdSetDiagonalInertia(MeMatrix4 relTM, MeMatrix3 inertia, MeReal ix, MeReal iy, MeReal iz)
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            relTM[r][c] = r == c ? (MeReal)1 : (MeReal)0;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            inertia[r][c] = 0;
    inertia[0][0] = ix;
    inertia[1][1] = iy;
    inertia[2][2] = iz;
}

inline void mcdSetInteractionsBoth(McdFramework* frame, int a, int b, McdInteractions i)
{
    McdFrameworkSetInteractions(frame, a, b, &i);
    McdFrameworkSetInteractions(frame, b, a, &i);
}

inline void mcdSetMeanNormal(McdIntersectResult* r, const btVector3& sum)
{
    if (r->contactCount > 0 && sum.length2() > 0)
        fromBt(sum.normalized(), r->normal);
}

inline const McdGeometryVTable* mcdVTable(McdGeometryID g)
{
    int type = McdGeometryGetTypeId(g);
    return McdFrameworkTypeIsRegistered(g->frame, type) ? &g->frame->geometryVTableTable[type] : nullptr;
}

// A shape-versus-plane pair in either model order.
struct McdPlanePair
{
    btTransform shapeTM;
    btVector3 n, origin;
    // Points from model2 to model1.
    btVector3 contactNormal;
    MeReal tolerance;
    int limit;
    McdGeometryID shape;

    btScalar height(const btVector3& p) const { return n.dot(p - origin); }
};

inline McdPlanePair mcdPlanePair(McdModelPair* p, McdIntersectResult* r)
{
    bool planeFirst = McdModelGetGeometryType(p->model1) == kMcdGeometryTypePlane;
    McdModelID shape = planeFirst ? p->model2 : p->model1;
    McdModelID plane = planeFirst ? p->model1 : p->model2;
    btTransform pt = toBtTransform(McdModelGetTransformPtr(plane));
    McdPlanePair pp;
    pp.shapeTM = toBtTransform(McdModelGetTransformPtr(shape));
    pp.n = pt.getBasis().getColumn(2);
    pp.origin = pt.getOrigin();
    pp.contactNormal = planeFirst ? -pp.n : pp.n;
    pp.tolerance = McdModelGetContactTolerance(shape) + McdModelGetContactTolerance(plane);
    pp.limit = mcdContactLimit(p, r);
    pp.shape = McdModelGetGeometry(shape);
    return pp;
}

inline int mcdFinishPlanePair(const McdPlanePair& pp, McdIntersectResult* r)
{
    if (r->contactCount > 0)
        fromBt(pp.contactNormal, r->normal);
    return r->contactCount > 0;
}

inline McdInteractions mcdIntersectInteraction(McdIntersectFn fn)
{
    McdInteractions i{};
    i.intersectFn = fn;
    return i;
}
