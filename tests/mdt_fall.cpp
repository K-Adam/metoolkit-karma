#include <Mdt.h>

#include "check.h"

int main()
{
    MdtWorldID world = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(world, 0, -10, 0);

    MdtBodyID falling = MdtBodyCreate(world);
    MdtBodyID resting = MdtBodyCreate(world);
    CHECK(!MdtBodyIsEnabled(falling));
    MdtBodyEnable(falling);
    MdtBodySetPosition(resting, 5, 0, 0);

    const MeReal dt = (MeReal)0.01;
    const int steps = 100;
    for (int i = 0; i < steps; ++i)
        MdtWorldStep(world, dt);

    MeVector3 p, v;
    MdtBodyGetPosition(falling, p);
    MdtBodyGetLinearVelocity(falling, v);
    // Semi-implicit Euler: after n steps y = -g dt^2 n (n + 1) / 2.
    CHECK_NEAR(p[1], -10 * dt * dt * steps * (steps + 1) / 2, 1e-3);
    CHECK_NEAR(v[1], -10.0, 1e-3);
    CHECK(MdtBodyGetTransformPtr(falling)[3][1] == p[1]);

    MdtBodyGetPosition(resting, p);
    CHECK(p[0] == 5 && p[1] == 0 && p[2] == 0);

    // Destroying a group-owned contact through the generic constraint path unlinks it from its group.
    MdtContactGroupID group = MdtContactGroupCreate(world);
    MdtContactID first = MdtContactGroupCreateContact(group);
    MdtContactID second = MdtContactGroupCreateContact(group);
    MdtConstraintEnable(MdtContactQuaConstraint(second));
    MdtContactDestroy(first);
    CHECK(MdtContactGroupGetCount(group) == 1 && MdtContactGroupGetFirstContact(group) == second);
    MdtContactGroupReset(group);
    CHECK(MdtContactGroupGetCount(group) == 0);
    MdtWorldStep(world, dt);

    MdtWorldDestroy(world);
    return 0;
}
