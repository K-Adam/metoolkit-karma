#include "McdImpl.h"

// Exact box-plane contacts: every box corner within the contact tolerance of the plane, as Karma reports
// a resting face as its four corners. Bullet's generic convex-plane test only samples points.

namespace {

int MEAPI intersectBoxPlane(McdModelPair* p, McdIntersectResult* r)
{
    bool boxFirst = McdModelGetGeometryType(p->model1) == kMcdGeometryTypeBox;
    McdModelID box = boxFirst ? p->model1 : p->model2;
    McdModelID plane = boxFirst ? p->model2 : p->model1;
    btTransform bt = toBtTransform(McdModelGetTransformPtr(box));
    btTransform pt = toBtTransform(McdModelGetTransformPtr(plane));
    btVector3 n = pt.getBasis().getColumn(2);
    const MeReal* radii = McdBoxGetRadii(McdModelGetGeometry(box));
    MeReal tolerance = McdModelGetContactTolerance(box) + McdModelGetContactTolerance(plane);
    int limit = mcdContactLimit(p, r);
    // The normal points from model2 to model1.
    btVector3 normal = boxFirst ? n : -n;

    for (int corner = 0; corner < 8 && r->contactCount < limit; ++corner)
    {
        btVector3 local((corner & 1 ? 1 : -1) * radii[0], (corner & 2 ? 1 : -1) * radii[1],
                        (corner & 4 ? 1 : -1) * radii[2]);
        btVector3 c = bt * local;
        btScalar d = n.dot(c - pt.getOrigin());
        if (d > tolerance)
            continue;
        mcdAddContact(r, c - n * (d / 2), normal, d);
    }
    if (r->contactCount > 0)
        fromBt(normal, r->normal);
    return r->contactCount > 0;
}

} // namespace

void mcdRegisterBoxPlane(McdFramework* frame)
{
    McdInteractions i{};
    i.intersectFn = intersectBoxPlane;
    McdFrameworkSetInteractions(frame, kMcdGeometryTypeBox, kMcdGeometryTypePlane, &i);
    McdFrameworkSetInteractions(frame, kMcdGeometryTypePlane, kMcdGeometryTypeBox, &i);
}
