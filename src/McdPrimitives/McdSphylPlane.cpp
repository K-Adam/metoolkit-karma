#include <cmath>

#include "Primitive.h"

// Matches Karma: the deeper end cap gives one contact. A second one sits at the other end when it is also in
// range, or where the axis rises to one radius above the plane when that point lies far enough along the
// axis. Both contacts lie on the plane and report the deeper separation.

namespace {

int MEAPI intersectSphylPlane(McdModelPair* p, McdIntersectResult* r)
{
    McdPlanePair pp = mcdPlanePair(p, r);
    MeReal radius = McdSphylGetRadius(pp.shape), half = McdSphylGetHeight(pp.shape) / 2;
    btVector3 e1 = pp.shapeTM * btVector3(0, 0, -half), e2 = pp.shapeTM * btVector3(0, 0, half);
    if (pp.height(e2) < pp.height(e1))
        std::swap(e1, e2);
    btScalar h1 = pp.height(e1), h2 = pp.height(e2), d1 = h1 - radius;
    if (d1 > pp.tolerance || pp.limit < 1)
        return 0;
    mcdAddContact(r, e1 - pp.n * h1, pp.contactNormal, d1);

    btVector3 axis = e2 - e1;
    btScalar length = axis.length();
    if (length > 0 && pp.limit > 1)
    {
        bool second = h2 - radius <= pp.tolerance;
        btVector3 q = e2;
        if (!second && d1 < 0)
        {
            btScalar t = -d1 / (h2 - h1);
            q = e1 + axis * t;
            second = t * length > radius * std::fabs(axis.dot(pp.n)) / length;
        }
        if (second)
            mcdAddContact(r, q - pp.n * pp.height(q), pp.contactNormal, d1);
    }
    return mcdFinishPlanePair(pp, r);
}

} // namespace

void mcdRegisterSphylPlane(McdFramework* frame)
{
    mcdSetInteractionsBoth(frame, kMcdGeometryTypeSphyl, kMcdGeometryTypePlane,
                           mcdIntersectInteraction(intersectSphylPlane));
}
