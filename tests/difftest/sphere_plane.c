#include <Mst.h>

#include "trace.h"

int main(void)
{
    MstUniverseSizes sizes = MstUniverseDefaultSizes;
    MstUniverseID universe = MstUniverseCreate(&sizes);
    McdFrameworkID frame = MstUniverseGetFramework(universe);
    MeMatrix4 groundTM = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    McdModelID ball;
    MdtBodyID body;
    MeVector3 p;
    int i, firstTouch = -1;

    MdtWorldSetGravity(MstUniverseGetWorld(universe), 0, 0, -10);
    MstFixedModelCreate(universe, McdPlaneCreate(frame), groundTM);
    ball = MstModelAndBodyCreate(universe, McdSphereCreate(frame, 0.5f), 1);
    body = McdModelGetBody(ball);
    MdtBodySetPosition(body, 0, 0, 2);
    APPROX(1e-4, "mass %.5f", MdtBodyGetMass(body));

    for (i = 1; i <= 300; ++i)
    {
        MstUniverseStep(universe, 0.01f);
        MdtBodyGetPosition(body, p);
        if (firstTouch < 0 && p[2] < 0.51f)
            firstTouch = i;
    }
    /* Kea and Bullet resolve the impact differently, so only its timing and the resting state are compared. */
    APPROX(1, "first touch step %d", firstTouch);
    APPROX(0.005, "rest z %.4f", p[2]);
    EXACT("rest x %.4f y %.4f", p[0], p[1]);

    MstUniverseDestroy(universe);
    return 0;
}
