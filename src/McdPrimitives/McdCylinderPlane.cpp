#include "Primitive.h"

// Karma tests four rim points per cap, starting below the cap centre and turning about the cylinder axis, and
// reports those that penetrate. A face resting on the plane gives four contacts, a cylinder on its side two.

namespace {

int MEAPI intersectCylinderPlane(McdModelPair* p, McdIntersectResult* r)
{
    McdPlanePair pp = mcdPlanePair(p, r);
    MeReal radius = McdCylinderGetRadius(pp.shape), half = McdCylinderGetHalfHeight(pp.shape);
    btVector3 down = pp.shapeTM.getBasis().transpose() * -pp.n;
    btVector3 u(down.x(), down.y(), 0);
    u = u.length2() > 1e-12f ? u.normalized() : btVector3(1, 0, 0);
    btVector3 rim[4] = {u, btVector3(-u.y(), u.x(), 0), -u, btVector3(u.y(), -u.x(), 0)};
    for (MeReal z : {-half, half})
        for (int i = 0; i < 4 && r->contactCount < pp.limit; ++i)
        {
            btVector3 c = pp.shapeTM * (rim[i] * radius + btVector3(0, 0, z));
            btScalar d = pp.height(c);
            if (d < 0)
                mcdAddContact(r, c, pp.contactNormal, d);
        }
    return mcdFinishPlanePair(pp, r);
}

} // namespace

void mcdRegisterCylinderPlane(McdFramework* frame)
{
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeCylinder, kMcdGeometryTypePlane,
                           mcdIntersectInteraction(intersectCylinderPlane));
}
