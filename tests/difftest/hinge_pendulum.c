#include <Mdt.h>

#include "trace.h"

/* A body hinged to the world one unit from its centre swings under gravity, first freely and then against a
   hard limit, and finally a second pendulum is driven by a motor. */
int main(void)
{
    MdtWorldID world = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID bob = MdtBodyCreate(world), wheel = MdtBodyCreate(world);
    MdtHingeID hinge = MdtHingeCreate(world), motor = MdtHingeCreate(world);
    MdtLimitID limit = MdtHingeGetLimit(hinge);
    MdtSingleLimitID stop;
    MeVector3 p, v, a;
    int i;

    MdtWorldSetGravity(world, 0, -9.81f, 0);
    MdtBodySetPosition(bob, 1, 0, 0);
    MdtBodyEnable(bob);
    MdtHingeSetBodies(hinge, bob, 0);
    MdtHingeSetPosition(hinge, 0, 0, 0);
    MdtHingeSetAxis(hinge, 0, 0, 1);
    MdtLimitCalculatePosition(limit, 1);
    MdtHingeEnable(hinge);

    MdtHingeGetAxis(hinge, a);
    MdtHingeGetPosition(hinge, p);
    EXACT("axis %.4f %.4f %.4f pos %.4f %.4f %.4f", a[0], a[1], a[2], p[0], p[1], p[2]);

    for (i = 1; i <= 120; ++i)
    {
        MdtWorldStep(world, 1.0f / 60);
        if (i % 10 == 0)
        {
            MdtBodyGetPosition(bob, p);
            MdtBodyGetAngularVelocity(bob, v);
            APPROX(0.003, "free %d pos %.4f %.4f %.4f w %.4f angle %.4f rate %.4f", i, p[0], p[1], p[2], v[2],
                   MdtLimitGetPosition(limit), MdtLimitGetVelocity(limit));
        }
    }

    stop = MdtLimitGetLowerLimit(limit);
    MdtSingleLimitSetStop(stop, -0.5f);
    MdtSingleLimitSetRestitution(stop, 0);
    MdtLimitSetLowerLimit(limit, stop);
    stop = MdtLimitGetUpperLimit(limit);
    MdtSingleLimitSetStop(stop, 0.5f);
    MdtSingleLimitSetRestitution(stop, 0);
    MdtLimitSetUpperLimit(limit, stop);
    MdtLimitActivateLimits(limit, 1);
    for (i = 1; i <= 120; ++i)
    {
        MdtWorldStep(world, 1.0f / 60);
        if (i % 20 == 0)
            APPROX(0.003, "limited %d angle %.4f over %.4f", i, MdtLimitGetPosition(limit), MdtLimitGetOvershoot(limit));
    }
    MdtHingeDisable(hinge);

    MdtBodySetPosition(wheel, 0, 0, 5);
    MdtBodyEnable(wheel);
    MdtHingeSetBodies(motor, wheel, 0);
    MdtHingeSetPosition(motor, 0, 0, 5);
    MdtHingeSetAxis(motor, 1, 0, 0);
    MdtLimitSetLimitedForceMotor(MdtHingeGetLimit(motor), 2, 1);
    MdtHingeEnable(motor);
    for (i = 1; i <= 60; ++i)
    {
        MdtWorldStep(world, 1.0f / 60);
        if (i % 15 == 0)
        {
            MdtBodyGetAngularVelocity(wheel, v);
            APPROX(0.001, "motor %d w %.4f %.4f %.4f", i, v[0], v[1], v[2]);
        }
    }

    MdtWorldDestroy(world);
    return 0;
}
