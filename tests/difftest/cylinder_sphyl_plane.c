/* Cylinders and sphyls against a plane: single intersect calls in several poses, then a fall through MstUniverse. */
#include <math.h>
#include <string.h>

#include <Mst.h>

#include "trace.h"

/* Rotation by angle about x, then the shape centre at height z. */
static void pose(MeMatrix4 tm, MeReal angle, MeReal z)
{
    memset(tm, 0, sizeof(MeMatrix4));
    tm[0][0] = tm[3][3] = 1;
    tm[1][1] = tm[2][2] = (MeReal)cos(angle);
    tm[1][2] = (MeReal)sin(angle);
    tm[2][1] = -(MeReal)sin(angle);
    tm[3][2] = z;
}

static void intersect(const char* name, McdGeometryID g, McdModelID plane, MeReal angle, MeReal lowest)
{
    MeMatrix4 tm;
    McdModelID m = McdModelCreate(g);
    McdModelPairID pair = McdModelPairCreate(m, plane);
    McdContact contacts[16];
    McdIntersectResult r;
    MeReal deepest = 0, c[3] = {0, 0, 0};
    int i;

    /* lowest is the distance from the centre to the lowest point, so every pose sinks 0.05 into the plane. */
    pose(tm, angle, lowest - 0.05f);
    McdModelSetTransformPtr(m, tm);
    McdModelUpdate(m);
    McdHello(pair);
    r.contacts = contacts;
    r.contactMaxCount = 16;
    McdIntersect(pair, &r);
    EXACT("%s touch %d contacts %d", name, r.touch, r.contactCount);
    for (i = 0; i < r.contactCount; ++i)
    {
        if (contacts[i].separation < deepest)
            deepest = contacts[i].separation;
        c[0] += contacts[i].position[0] / r.contactCount;
        c[1] += contacts[i].position[1] / r.contactCount;
        c[2] += contacts[i].position[2] / r.contactCount;
    }
    APPROX(1e-3, "%s deepest %.4f normal %.3f %.3f %.3f", name, deepest, r.normal[0], r.normal[1], r.normal[2]);
    APPROX(2e-2, "%s centroid %.3f %.3f %.3f", name, c[0], c[1], c[2]);
    McdGoodbye(pair);
    McdModelPairDestroy(pair);
    McdModelDestroy(m);
}

static MdtBodyID drop(MstUniverseID u, const char* name, McdGeometryID g, MeReal x)
{
    McdModelID m = MstModelAndBodyCreate(u, g, 1);
    MdtBodyID b = McdModelGetBody(m);
    MeMatrix3 inertia;
    MdtBodyGetInertiaTensor(b, inertia);
    APPROX(1e-4, "%s mass %.5f inertia %.5f %.5f %.5f", name, MdtBodyGetMass(b), inertia[0][0], inertia[1][1],
           inertia[2][2]);
    MdtBodySetPosition(b, x, 0, 2);
    return b;
}

int main(void)
{
    MstUniverseSizes sizes = MstUniverseDefaultSizes;
    MstUniverseID universe = MstUniverseCreate(&sizes);
    McdFrameworkID frame = MstUniverseGetFramework(universe);
    MeMatrix4 groundTM = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    McdGeometryID cylinder = McdCylinderCreate(frame, 0.5f, 1);
    McdGeometryID sphyl = McdSphylCreate(frame, 0.3f, 1);
    McdModelID plane = McdModelCreate(McdPlaneCreate(frame));
    MdtBodyID cylBody, sphylBody;
    MeReal tilt = 0.3f;
    MeVector3 p;
    int i, cylTouch = -1, sphylTouch = -1;

    McdModelSetTransformPtr(plane, groundTM);
    McdModelUpdate(plane);
    EXACT("cylinder radius %.3f height %.3f", McdCylinderGetRadius(cylinder), McdCylinderGetHeight(cylinder));
    EXACT("sphyl radius %.3f height %.3f", McdSphylGetRadius(sphyl), McdSphylGetHeight(sphyl));

    intersect("cylinder upright", cylinder, plane, 0, 0.5f);
    intersect("cylinder side", cylinder, plane, (MeReal)(ME_PI / 2), 0.5f);
    intersect("cylinder tilted", cylinder, plane, tilt, 0.5f * (MeReal)cos(tilt) + 0.5f * (MeReal)sin(tilt));
    intersect("sphyl upright", sphyl, plane, 0, 0.8f);
    intersect("sphyl side", sphyl, plane, (MeReal)(ME_PI / 2), 0.3f);
    intersect("sphyl tilted", sphyl, plane, tilt, 0.5f * (MeReal)cos(tilt) + 0.3f);
    McdModelDestroy(plane);

    MdtWorldSetGravity(MstUniverseGetWorld(universe), 0, 0, -10);
    MstFixedModelCreate(universe, McdPlaneCreate(frame), groundTM);
    cylBody = drop(universe, "cylinder", cylinder, -2);
    sphylBody = drop(universe, "sphyl", sphyl, 2);
    /* The sphyl falls lying along y. */
    MdtBodySetQuaternion(sphylBody, (MeReal)cos(ME_PI / 4), (MeReal)sin(ME_PI / 4), 0, 0);

    for (i = 1; i <= 300; ++i)
    {
        MstUniverseStep(universe, 0.01f);
        MdtBodyGetPosition(cylBody, p);
        if (cylTouch < 0 && p[2] < 0.51f)
            cylTouch = i;
        MdtBodyGetPosition(sphylBody, p);
        if (sphylTouch < 0 && p[2] < 0.31f)
            sphylTouch = i;
    }
    APPROX(1, "cylinder first touch step %d", cylTouch);
    APPROX(1, "sphyl first touch step %d", sphylTouch);
    MdtBodyGetPosition(cylBody, p);
    APPROX(0.005, "cylinder rest %.4f %.4f %.4f", p[0], p[1], p[2]);
    MdtBodyGetPosition(sphylBody, p);
    APPROX(0.005, "sphyl rest %.4f %.4f %.4f", p[0], p[1], p[2]);

    MstUniverseDestroy(universe);
    return 0;
}
