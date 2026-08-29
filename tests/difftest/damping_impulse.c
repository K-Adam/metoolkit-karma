/* Damping, impulses, a centre of mass offset and body getters, with auto-disable left at its default. */
#include <Mdt.h>

#include "trace.h"

static void vec(const char* name, int step, const MeReal* v)
{
    APPROX(1e-4, "%s %d %.6f %.6f %.6f", name, step, v[0], v[1], v[2]);
}

int main(void)
{
    MdtWorldID world = MdtWorldCreate(10, 10, 2, 3);
    MdtBodyID damped = MdtBodyCreate(world), kicked = MdtBodyCreate(world), offset = MdtBodyCreate(world);
    MeVector3 v;
    MeMatrix3 I;
    int i;

    MdtBodyGetInertiaTensor(damped, I);
    EXACT("defaults mass %g inertia %g damping %g %g", MdtBodyGetMass(damped), I[0][0],
          MdtBodyGetLinearVelocityDamping(damped), MdtBodyGetAngularVelocityDamping(damped));
    EXACT("auto-disable %d %g %g %g %g %g", MdtWorldGetAutoDisable(world), MdtWorldGetAutoDisableVelocityThreshold(world),
          MdtWorldGetAutoDisableAngularVelocityThreshold(world), MdtWorldGetAutoDisableAccelerationThreshold(world),
          MdtWorldGetAutoDisableAngularAccelerationThreshold(world), MdtWorldGetAutoDisableAliveTime(world));
    EXACT("epsilon %g gamma %g", MdtWorldGetEpsilon(world), MdtWorldGetGamma(world));

    MdtWorldSetGravity(world, 0, -10, 0);
    MdtBodySetMass(damped, 2);
    MdtBodySetLinearVelocity(damped, 3, 0, 0);
    MdtBodySetAngularVelocity(damped, 0, 0, 2);
    MdtBodySetLinearVelocityDamping(damped, 0.5f);
    MdtBodySetAngularVelocityDamping(damped, 0.3f);
    MdtBodyEnable(damped);

    MdtBodySetMass(kicked, 2);
    MdtBodySetSphericalInertiaTensor(kicked, 0.5f);
    MdtBodySetPosition(kicked, 5, 0, 0);
    MdtBodyEnable(kicked);

    MdtBodySetPosition(offset, 0, 5, 0);
    v[0] = 1;
    v[1] = 0;
    v[2] = 0;
    MdtBodySetCenterOfMassRelativePosition(offset, v);
    MdtBodySetAngularVelocity(offset, 0, 0, 1);
    MdtBodyEnable(offset);
    MdtBodyGetCenterOfMassPosition(offset, v);
    vec("com", 0, v);

    for (i = 1; i <= 20; ++i)
    {
        MdtBodyAddForce(damped, 1, 0, 0);
        if (i == 1)
            MdtBodyAddImpulse(kicked, 1, 0, 0);
        if (i == 5)
            MdtBodyAddImpulseAtPosition(kicked, 0, 1, 0, 6, 0, 0);
        MdtWorldStep(world, 0.02f);
        if (i % 5 && i != 1)
            continue;
        MdtBodyGetPosition(damped, v);
        vec("damped pos", i, v);
        MdtBodyGetLinearVelocity(damped, v);
        vec("damped vel", i, v);
        MdtBodyGetAngularVelocity(damped, v);
        vec("damped w", i, v);
        MdtBodyGetLinearAcceleration(damped, v);
        vec("damped acc", i, v);
        MdtBodyGetAngularAcceleration(damped, v);
        vec("damped accrot", i, v);
        MdtBodyGetLinearVelocity(kicked, v);
        vec("kicked vel", i, v);
        MdtBodyGetAngularVelocity(kicked, v);
        vec("kicked w", i, v);
        APPROX(1e-3, "energy %d %.5f %.5f", i, MdtBodyGetKineticEnergy(damped), MdtBodyGetKineticEnergy(kicked));
        APPROX(1e-6, "enabled time %d %.4f", i, MdtBodyGetEnabledTime(damped));
        /* Bullet integrates rotation with the exponential map, Kea to first order. */
        MdtBodyGetPosition(offset, v);
        APPROX(2e-3, "offset pos %d %.6f %.6f %.6f", i, v[0], v[1], v[2]);
        MdtBodyGetCenterOfMassPosition(offset, v);
        APPROX(2e-3, "offset com %d %.6f %.6f %.6f", i, v[0], v[1], v[2]);
    }

    MdtWorldDestroy(world);
    return 0;
}
