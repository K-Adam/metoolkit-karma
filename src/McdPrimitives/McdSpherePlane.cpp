#include "Primitive.h"

// Karma places the contact at the lowest point of the sphere and accepts it within the contact tolerance.

namespace {

int MEAPI intersectSpherePlane(McdModelPair* p, McdIntersectResult* r)
{
    McdPlanePair pp = mcdPlanePair(p, r);
    MeReal radius = McdSphereGetRadius(pp.shape);
    btVector3 c = pp.shapeTM.getOrigin();
    btScalar d = pp.height(c) - radius;
    if (d <= pp.tolerance && pp.limit > 0)
        mcdAddContact(r, c - pp.n * radius, pp.contactNormal, d);
    return mcdFinishPlanePair(pp, r);
}

} // namespace

void mcdRegisterSpherePlane(McdFramework* frame)
{
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeSphere, kMcdGeometryTypePlane,
                           mcdIntersectInteraction(intersectSpherePlane));
}
