#include "Primitive.h"

// Every penetrating box corner is a contact, as Karma reports a resting face as its four corners. Bullet's
// generic convex-plane test only samples points.

namespace {

int MEAPI intersectBoxPlane(McdModelPair* p, McdIntersectResult* r)
{
    McdPlanePair pp = mcdPlanePair(p, r);
    const MeReal* radii = McdBoxGetRadii(pp.shape);
    for (int corner = 0; corner < 8 && r->contactCount < pp.limit; ++corner)
    {
        btVector3 local((corner & 1 ? 1 : -1) * radii[0], (corner & 2 ? 1 : -1) * radii[1],
                        (corner & 4 ? 1 : -1) * radii[2]);
        btVector3 c = pp.shapeTM * local;
        btScalar d = pp.height(c);
        if (d < 0)
            mcdAddContact(r, c, pp.contactNormal, d);
    }
    return mcdFinishPlanePair(pp, r);
}

} // namespace

void mcdRegisterBoxPlane(McdFramework* frame)
{
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeBox, kMcdGeometryTypePlane,
                           mcdIntersectInteraction(intersectBoxPlane));
}
