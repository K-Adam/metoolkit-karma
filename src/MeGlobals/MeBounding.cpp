#include <MeBounding.h>
#include <MeMath.h>

// Ritter's approximation, up to about 5% above the minimal sphere. Use Welzl if a tight fit matters.
extern "C" void MEAPI MeBoundingSphereCalc(MeVector3* points, int numPoints, MeVector3 centre, MeReal* radius)
{
    MeVector3Set(centre, 0, 0, 0);
    *radius = 0;
    if (numPoints <= 0)
        return;

    auto farthestFrom = [&](const MeVector3 p) {
        int best = 0;
        for (int i = 1; i < numPoints; ++i)
            if (MeVector3DistanceSqr(points[i], p) > MeVector3DistanceSqr(points[best], p))
                best = i;
        return best;
    };
    int a = farthestFrom(points[0]);
    int b = farthestFrom(points[a]);
    MeVector3Add(centre, points[a], points[b]);
    MeVector3Scale(centre, MeReal(0.5));
    MeReal r = MeVector3Distance(points[a], points[b]) / 2;

    for (int i = 0; i < numPoints; ++i)
    {
        MeReal d = MeVector3Distance(points[i], centre);
        if (d > r)
        {
            // Grow just enough to touch the outlier, keeping the far side of the old sphere.
            MeReal grown = (r + d) / 2;
            MeVector3 toPoint;
            MeVector3Subtract(toPoint, points[i], centre);
            MeVector3MultiplyAdd(centre, (grown - r) / d, toPoint);
            r = grown;
        }
    }
    *radius = r;
}
