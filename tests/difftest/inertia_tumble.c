#include <Mdt.h>

#include "trace.h"

/* Bodies with a full inertia tensor: one tumbling freely with the Coriolis force, one without it, one left
   spherical, and one with a centre of mass offset swinging on a hinge, whose frame must follow the body
   frame rather than the principal axes. */
static const MeMatrix3 tensor = {{0.5f, 0.1f, -0.05f}, {0.1f, 0.3f, 0.08f}, {-0.05f, 0.08f, 0.2f}};

static MdtBodyID body(MdtWorldID world, MeReal z, int nonSpherical, int coriolis)
{
    MdtBodyID b = MdtBodyCreate(world);
    MdtBodySetMass(b, 2);
    MdtBodySetInertiaTensor(b, tensor);
    if (nonSpherical)
        MdtBodyEnableNonSphericalInertia(b);
    if (coriolis)
        MdtBodyEnableCoriolisForce(b);
    MdtBodySetPosition(b, 0, 0, z);
    MdtBodySetAngularVelocityDamping(b, 0);
    MdtBodySetAngularVelocity(b, 1, 2, 3);
    MdtBodyEnable(b);
    return b;
}

static void trace(const char* name, int step, MdtBodyID b, double tol)
{
    MeVector3 p, w;
    MeVector4 q;
    MdtBodyGetPosition(b, p);
    MdtBodyGetAngularVelocity(b, w);
    MdtBodyGetQuaternion(b, q);
    APPROX(tol, "%s %d pos %.4f %.4f %.4f w %.4f %.4f %.4f q %.4f %.4f %.4f %.4f ke %.4f", name, step, p[0], p[1],
           p[2], w[0], w[1], w[2], q[0], q[1], q[2], q[3], MdtBodyGetKineticEnergy(b));
}

int main(void)
{
    MdtWorldID world = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID tumble, plain, sphere, swing;
    MdtHingeID hinge = MdtHingeCreate(world);
    MeMatrix3 I;
    MeVector3 com = {0.2f, -0.1f, 0.1f}, a;
    int i;

    MdtWorldSetGravity(world, 0, 0, 0);
    MdtWorldSetAutoDisable(world, 0);
    tumble = body(world, 0, 1, 1);
    plain = body(world, 10, 1, 0);
    sphere = body(world, 20, 0, 1);

    swing = body(world, 30, 1, 1);
    MdtBodySetAngularVelocity(swing, 0, 0, 0);
    MdtBodySetCenterOfMassRelativePosition(swing, com);
    MdtBodySetPosition(swing, 1, 0, 30);
    MdtHingeSetBodies(hinge, swing, 0);
    MdtHingeSetPosition(hinge, 0, 0, 30);
    MdtHingeSetAxis(hinge, 0.3f, 0.2f, 1);
    MdtHingeEnable(hinge);

    MdtBodyGetInertiaTensor(tumble, I);
    EXACT("inertia %.3f %.3f %.3f %.3f %.3f %.3f", I[0][0], I[0][1], I[0][2], I[1][1], I[1][2], I[2][2]);

    for (i = 1; i <= 120; ++i)
    {
        /* Gravity on the swinging body only, applied at its centre of mass. */
        MdtBodyAddForce(swing, 0, -2 * 9.81f, 0);
        MdtWorldStep(world, 1.0f / 60);
        if (i % 20 != 0)
            continue;
        trace("tumble", i, tumble, 1e-2);
        trace("plain", i, plain, 1e-2);
        trace("sphere", i, sphere, 1e-2);
        /* Kea softens the hinge rows by epsilon, which Bullet does not, so the swing drifts slowly. */
        if (i <= 60)
            trace("swing", i, swing, 1e-1);
        MdtHingeGetAxis(hinge, a);
        APPROX(1e-2, "hinge %d axis %.4f %.4f %.4f", i, a[0], a[1], a[2]);
    }

    MdtWorldDestroy(world);
    return 0;
}
