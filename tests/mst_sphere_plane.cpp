#include <Mst.h>

#include "check.h"

int main()
{
    MstUniverseSizes sizes = MstUniverseDefaultSizes;
    MstUniverseID universe = MstUniverseCreate(&sizes);
    McdFrameworkID frame = MstUniverseGetFramework(universe);
    MdtWorldSetGravity(MstUniverseGetWorld(universe), 0, 0, -10);

    MeMatrix4 groundTM = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    McdModelID ground = MstFixedModelCreate(universe, McdPlaneCreate(frame), groundTM);
    CHECK(ground != nullptr);

    const MeReal radius = (MeReal)0.5;
    McdModelID ball = MstModelAndBodyCreate(universe, McdSphereCreate(frame, radius), 1);
    MdtBodyID body = McdModelGetBody(ball);
    CHECK(body && MdtBodyIsEnabled(body));
    CHECK_NEAR(MdtBodyGetMass(body), 4.0 / 3.0 * ME_PI * radius * radius * radius, 1e-4);
    MdtBodySetPosition(body, 0, 0, 2);

    for (int i = 0; i < 300; ++i)
        MstUniverseStep(universe, (MeReal)(1.0 / 100));

    MeVector3 p, v;
    MdtBodyGetPosition(body, p);
    MdtBodyGetLinearVelocity(body, v);
    CHECK_NEAR(p[2], radius, 0.01 * radius);
    CHECK_NEAR(v[2], 0, 0.05);
    CHECK_NEAR(p[0], 0, 1e-4);

    MstModelAndBodyDestroy(ball);
    MstFixedModelDestroy(ground);
    MstUniverseDestroy(universe);
    return 0;
}
