#include <McdConvexMesh.h>
#include <McdGjk.h>
#include <McdPlaneIntersect.h>
#include <McdPrimitives.h>

#include "McdImpl.h"

// Karma's GJK and plane slicing are replaced by Bullet's convex narrowphase. The entry points keep their
// registration roles: McdCacheHello, McdCacheGoodbye and McdGjkCgIntersect form a convex pair interaction.

namespace {

McdInteractions gjkInteraction()
{
    McdInteractions i{};
    i.helloFn = McdCacheHello;
    i.goodbyeFn = McdCacheGoodbye;
    i.intersectFn = McdGjkCgIntersect;
    return i;
}

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
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, kMcdGeometryTypeBox, gjkInteraction());
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, kMcdGeometryTypeSphere, gjkInteraction());
    return 1;
}

MeBool MEAPI McdConvexMeshPrimitivesRegisterInteractions(McdFramework* frame)
{
    McdConvexMeshRegisterBoxAndSphereFns(frame);
    for (int type : {kMcdGeometryTypeConvexMesh, kMcdGeometryTypeCylinder, kMcdGeometryTypeSphyl})
        mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, type, gjkInteraction());
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeConvexMesh, kMcdGeometryTypePlane,
                           mcdIntersectInteraction(intersectConvexMeshPlane));
    mcdRegisterTriangleListInteraction(frame, kMcdGeometryTypeConvexMesh);
    return 1;
}

} // extern "C"
