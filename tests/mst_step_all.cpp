#include <Mst.h>

#include "check.h"

int main()
{
    MstUniverseSizes sizes = MstUniverseDefaultSizes;
    MstUniverseID universe = MstUniverseCreate(&sizes);
    McdFrameworkID frame = MstUniverseGetFramework(universe);
    MdtWorldID world = MstUniverseGetWorld(universe);
    McdSpaceID space = MstUniverseGetSpace(universe);
    MstBridgeID bridge = MstUniverseGetBridge(universe);
    MdtWorldSetGravity(world, 0, 0, -10);

    MeMatrix4 groundTM = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    McdModelID ground = MstFixedModelCreate(universe, McdPlaneCreate(frame), groundTM);
    McdModelID ball = MstModelAndBodyCreate(universe, McdSphereCreate(frame, (MeReal)0.5), 1);
    MdtBodyID body = McdModelGetBody(ball);
    MdtBodySetPosition(body, 0, 0, 2);

    // Before the bridge has connected anything it cannot step.
    MstBridgeStepAll(bridge, (MeReal)0.01);
    MeVector3 p;
    MdtBodyGetPosition(body, p);
    CHECK_NEAR(p[2], 2, 0);

    MstBridgeUpdateTransitions(bridge, space, world);
    for (int i = 0; i < 300; ++i)
        MstBridgeStepAll(bridge, (MeReal)0.01);
    MdtBodyGetPosition(body, p);
    CHECK_NEAR(p[2], 0.5, 0.01);

    MstModelAndBodyDestroy(ball);
    MstFixedModelDestroy(ground);
    MstUniverseDestroy(universe);
    return 0;
}
