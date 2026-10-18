#include <Mdt.h>

#include "trace.h"

/* Springs with Karma's two soft limits: one body on a spring to the world, two free bodies sharing a spring,
   a damped spring, one attached off the centre of mass, which makes the body spin, a heavy body, a body
   moved after its spring was attached, and a spring too stiff for the world epsilon. The natural length adds
   to the separation at the first step. */
static MdtSpringID spring(MdtWorldID world, MdtBodyID a, MdtBodyID b, MeReal length, MeReal k, MeReal d)
{
    MdtSpringID s = MdtSpringCreate(world);
    MdtSpringSetBodies(s, a, b);
    MdtSpringSetNaturalLength(s, length);
    MdtSpringSetStiffness(s, k);
    MdtSpringSetDamping(s, d);
    return s;
}

static MdtBodyID body(MdtWorldID world, MeReal x, MeReal y, MeReal z)
{
    MdtBodyID b = MdtBodyCreate(world);
    MdtBodySetMass(b, 1);
    MdtBodySetPosition(b, x, y, z);
    MdtBodyEnable(b);
    return b;
}

static void trace(const char* name, int step, MdtBodyID b, MdtSpringID s, double tol)
{
    MeVector3 p, v, w;
    MdtBodyGetPosition(b, p);
    MdtBodyGetLinearVelocity(b, v);
    MdtBodyGetAngularVelocity(b, w);
    APPROX(tol, "%s %d pos %.4f %.4f %.4f vel %.4f %.4f %.4f w %.4f %.4f %.4f len %.4f", name, step, p[0], p[1],
           p[2], v[0], v[1], v[2], w[0], w[1], w[2], MdtLimitGetPosition(MdtSpringGetLimit(s)));
}

int main(void)
{
    MdtWorldID world = MdtWorldCreate(20, 20, 1, 1);
    MdtBodyID single, left, right, damped, spun, heavy, moved, stiff;
    MdtSpringID s1, s2, s3, s4, s5, s6, s7;
    int i;

    MdtWorldSetGravity(world, 0, 0, 0);
    MdtWorldSetAutoDisable(world, 0);

    single = body(world, 1.5f, 0, 0);
    s1 = spring(world, single, 0, 1, 20, 0);
    MdtSpringSetPosition(s1, 0, 1.5f, 0, 0);
    MdtSpringSetPosition(s1, 1, 0, 0, 0);
    MdtSpringEnable(s1);

    left = body(world, 0, 0, 10);
    right = body(world, 2, 0, 10);
    s2 = spring(world, left, right, 1, 20, 0);
    MdtSpringSetPosition(s2, 0, 0, 0, 10);
    MdtSpringSetPosition(s2, 1, 2, 0, 10);
    MdtSpringEnable(s2);

    damped = body(world, 0, 1.5f, 20);
    s3 = spring(world, damped, 0, 1, 20, 2);
    MdtSpringSetPosition(s3, 0, 0, 1.5f, 20);
    MdtSpringSetPosition(s3, 1, 0, 0, 20);
    MdtSpringEnable(s3);

    spun = body(world, 0, 0, 30);
    s4 = spring(world, spun, 0, 1, 20, 0);
    MdtSpringSetPosition(s4, 0, 0, 0.5f, 30);
    MdtSpringSetPosition(s4, 1, 1.5f, 0.5f, 30);
    MdtSpringEnable(s4);

    heavy = body(world, 0, 0, 40);
    MdtBodySetMass(heavy, 4);
    s5 = spring(world, heavy, 0, 0.5f, 20, 0);
    MdtSpringSetPosition(s5, 0, 0, 0, 40);
    MdtSpringSetPosition(s5, 1, 0, -1, 40);
    MdtSpringEnable(s5);

    moved = body(world, 0, 0, 50);
    s6 = spring(world, moved, 0, 0, 20, 0);
    MdtSpringSetPosition(s6, 0, 0, 0, 50);
    MdtSpringSetPosition(s6, 1, 0, 0, 49);
    MdtSpringEnable(s6);
    MdtBodySetPosition(moved, 0, 0, 50.5f);

    stiff = body(world, 0, 0, 60);
    MdtBodySetMass(stiff, 5);
    s7 = spring(world, stiff, 0, 0.2f, 1e5f, 0);
    MdtSpringSetPosition(s7, 0, 0, 0, 60);
    MdtSpringSetPosition(s7, 1, -1, 0, 60);
    MdtSpringEnable(s7);

    EXACT("stiffness %.4f damping %.4f", MdtLimitGetUpperLimit(MdtSpringGetLimit(s1))->stiffness,
          MdtLimitGetUpperLimit(MdtSpringGetLimit(s1))->damping);

    for (i = 1; i <= 120; ++i)
    {
        MdtWorldStep(world, 1.0f / 60);
        if (i % 10 != 0)
            continue;
        trace("single", i, single, s1, 5e-3);
        trace("left", i, left, s2, 5e-3);
        trace("right", i, right, s2, 5e-3);
        trace("damped", i, damped, s3, 5e-3);
        /* The spinning body drifts slowly from Kea's orientation integration. */
        trace("spun", i, spun, s4, 5e-2);
        trace("heavy", i, heavy, s5, 5e-3);
        trace("moved", i, moved, s6, 5e-3);
        trace("stiff", i, stiff, s7, 5e-3);
    }

    MdtWorldDestroy(world);
    return 0;
}
