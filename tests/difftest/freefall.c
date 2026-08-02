#include <Mdt.h>

#include "trace.h"

int main(void)
{
    MdtWorldID world = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID body = MdtBodyCreate(world);
    MeVector3 p, v;
    int i;

    MdtWorldSetGravity(world, 0, -9.81f, 0);
    MdtBodySetPosition(body, 0, 10, 0);
    MdtBodySetLinearVelocity(body, 1, 0, 0);
    MdtBodyEnable(body);
    EXACT("enabled %d mass %.4f", MdtBodyIsEnabled(body), MdtBodyGetMass(body));

    for (i = 1; i <= 60; ++i)
    {
        MdtWorldStep(world, 1.0f / 60);
        if (i % 10 == 0)
        {
            MdtBodyGetPosition(body, p);
            MdtBodyGetLinearVelocity(body, v);
            APPROX(1e-4, "step %d pos %.6f %.6f %.6f vel %.6f %.6f %.6f", i, p[0], p[1], p[2], v[0], v[1], v[2]);
        }
    }

    MdtBodyDisable(body);
    MdtWorldStep(world, 1.0f / 60);
    MdtBodyGetPosition(body, p);
    APPROX(1e-4, "disabled pos %.6f %.6f %.6f", p[0], p[1], p[2]);

    MdtWorldDestroy(world);
    return 0;
}
