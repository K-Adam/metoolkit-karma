#include <McdConvexMesh.h>
#include <McdGjk.h>
#include <McdPlaneIntersect.h>
#include <McdPrimitives.h>

#include "McdImpl.h"

// Karma's GJK and plane slicing are replaced by Bullet's convex narrowphase. McdCacheHello, McdCacheGoodbye and
// McdGjkCgIntersect stay as its public entry points.

namespace {

void addPlaneContact(const McdPlanePair& pp, McdIntersectResult* r, const btVector3& point, MeReal inflate)
{
    btScalar d = pp.height(point) - inflate;
    if (d < 0 && r->contactCount < pp.limit)
        mcdAddContact(r, point - pp.n * inflate, pp.contactNormal, d);
}

// Every penetrating hull vertex is a contact, like the corners of a box.
int MEAPI intersectConvexMeshPlane(McdModelPair* p, McdIntersectResult* r)
{
    McdPlanePair pp = mcdPlanePair(p, r);
    const McdConvexHull* hull = McdConvexMeshGetPolyhedron(pp.shape);
    for (int i = 0; i < hull->numVertex; ++i)
        addPlaneContact(pp, r, pp.shapeTM * toBt(hull->vertex[i].position), McdConvexMeshGetFatness(pp.shape));
    return mcdFinishPlanePair(pp, r);
}

} // namespace

extern "C" {

MeBool MEAPI McdCacheHello(McdModelPair* p)
{
    mcdBulletHello(p);
    return p->m_cachedData != nullptr;
}

void MEAPI McdCacheGoodbye(McdModelPair* p)
{
    mcdBulletGoodbye(p);
}

int MEAPI McdGjkCgIntersect(McdModelPair* p, McdIntersectResult* result)
{
    return mcdBulletIntersect(p, result);
}

// Any geometry against a plane: its deepest point, when it penetrates.
int MEAPI McdPlaneIntersect(McdModelPair* p, McdIntersectResult* result)
{
    McdPlanePair pp = mcdPlanePair(p, result);
    bool planeFirst = McdModelGetGeometryType(p->model1) == kMcdGeometryTypePlane;
    MeVector3 down, deepest;
    fromBt(-pp.n, down);
    McdGeometryInstanceMaximumPoint(McdModelGetGeometryInstance(planeFirst ? p->model2 : p->model1), down, deepest);
    addPlaneContact(pp, result, toBt(deepest), 0);
    result->touch = mcdFinishPlanePair(pp, result);
    return result->touch;
}

MeBool MEAPI McdConvexMeshRegisterBoxAndSphereFns(McdFramework* frame)
{
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, kMcdGeometryTypeBox, mcdBulletInteraction());
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, kMcdGeometryTypeSphere, mcdBulletInteraction());
    return 1;
}

MeBool MEAPI McdConvexMeshPrimitivesRegisterInteractions(McdFramework* frame)
{
    McdConvexMeshRegisterBoxAndSphereFns(frame);
    for (int type : {kMcdGeometryTypeCylinder, kMcdGeometryTypeSphyl})
        mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, type, mcdBulletInteraction());
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, kMcdGeometryTypePlane,
                           mcdIntersectInteraction(intersectConvexMeshPlane));
    mcdRegisterTriangleListInteraction(frame, kMcdGeometryTypeConvexMesh);
    return 1;
}

MeBool MEAPI McdConvexMeshConvexMeshRegisterInteraction(McdFramework* frame)
{
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, kMcdGeometryTypeConvexMesh, mcdBulletInteraction());
    return 1;
}

} // extern "C"
