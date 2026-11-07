#include <Mdt.h>

#include "check.h"

namespace {

const MeReal dt = (MeReal)0.01;

void step(MdtWorldID w, int n)
{
    for (int i = 0; i < n; ++i)
        MdtWorldStep(w, dt);
}

MdtBodyID body(MdtWorldID w, MeReal x, MeReal y, MeReal z)
{
    MdtBodyID b = MdtBodyCreate(w);
    MdtBodySetPosition(b, x, y, z);
    MdtBodyEnable(b);
    return b;
}

// A joint on a disabled body leaves Bullet's world but stays enabled, and works again once the body wakes.
void disabledBody()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(w, 0, -10, 0);
    MdtBodyID b = body(w, 1, 0, 0);
    MdtBSJointID j = MdtBSJointCreate(w);
    MdtBSJointSetBodies(j, b, 0);
    MdtBSJointSetPosition(j, 0, 0, 0);
    MdtBSJointEnable(j);
    step(w, 5);
    MdtBodyDisable(b);
    step(w, 5);
    CHECK(MdtBSJointIsEnabled(j));
    MdtBodyEnable(b);
    step(w, 100);
    MeVector3 p;
    MdtBodyGetPosition(b, p);
    CHECK_NEAR(sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]), 1, 1e-2);
    MdtWorldDestroy(w);
}

// Destroying an enabled joint frees its body.
void destroyEnabled()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(w, 0, -10, 0);
    MdtBodyID a = body(w, 0, 0, 0), b = body(w, 0, -1, 0);
    MdtBSJointID j = MdtBSJointCreate(w);
    MdtBSJointSetBodies(j, a, 0);
    MdtBSJointEnable(j);
    MdtHingeID h = MdtHingeCreate(w);
    MdtHingeSetBodies(h, b, a);
    MdtHingeSetPosition(h, 0, 0, 0);
    MdtHingeEnable(h);
    step(w, 10);
    CHECK(MdtConstraintGetFirst(w) == MdtBSJointQuaConstraint(j));
    CHECK(MdtConstraintGetNext(MdtBSJointQuaConstraint(j)) == MdtHingeQuaConstraint(h));
    MdtBSJointDestroy(j);
    CHECK(MdtWorldGetTotalConstraints(w) == 1);
    step(w, 10);
    MeVector3 v;
    MdtBodyGetLinearVelocity(a, v);
    CHECK_NEAR(v[1], -1, 1e-2);
    // Destroying a body destroys the joints on it.
    MdtBodyDestroy(b);
    CHECK(MdtWorldGetTotalConstraints(w) == 0);
    step(w, 1);
    MdtWorldDestroy(w);
}

// A pendulum released outside hard limits settles between them, and the joint force holds up its weight.
void hingeLimits()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(w, 0, -10, 0);
    // A pendulum at rest on its stop would auto-disable and drop out of the solver before the force is read.
    MdtWorldSetAutoDisable(w, 0);
    MdtBodyID b = body(w, 1, 0, 0);
    MdtHingeID h = MdtHingeCreate(w);
    MdtHingeSetBodies(h, b, 0);
    MdtHingeSetPosition(h, 0, 0, 0);
    MdtHingeSetAxis(h, 0, 0, 1);
    MdtLimitID l = MdtHingeGetLimit(h);
    MdtSingleLimitID lo = MdtLimitGetLowerLimit(l), hi = MdtLimitGetUpperLimit(l);
    MdtSingleLimitSetStop(lo, -0.3f);
    MdtSingleLimitSetStop(hi, 0.3f);
    MdtSingleLimitSetRestitution(lo, 0);
    MdtLimitActivateLimits(l, 1);
    CHECK(MdtLimitPositionIsCalculated(l));
    MdtHingeEnable(h);
    step(w, 300);
    CHECK_NEAR(MdtLimitGetPosition(l), -0.3, 1e-2);
    MeVector3 f;
    MdtHingeGetForce(h, 0, f);
    CHECK_NEAR(f[1], 10, 0.5);
    MdtWorldDestroy(w);
}

// A prismatic joint keeps a falling body on its axis and fixes its orientation.
void prismatic()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(w, 0, -10, 0);
    MdtBodyID b = body(w, 0, 0, 0);
    MdtBodySetLinearVelocity(b, 1, 0, 0);
    MdtBodySetAngularVelocity(b, 1, 2, 3);
    MdtPrismaticID j = MdtPrismaticCreate(w);
    MdtPrismaticSetBodies(j, b, 0);
    MdtPrismaticSetAxis(j, 0, 1, 0);
    MdtPrismaticEnable(j);
    step(w, 100);
    MeVector3 p, a;
    MdtBodyGetPosition(b, p);
    MdtBodyGetAngularVelocity(b, a);
    CHECK_NEAR(p[0], 0, 1e-2);
    CHECK(p[1] < -4);
    CHECK_NEAR(a[0] * a[0] + a[1] * a[1] + a[2] * a[2], 0, 1e-4);
    CHECK_NEAR(MdtLimitGetPosition(MdtPrismaticGetLimit(j)), p[1], 0.1);
    MdtWorldDestroy(w);
}

// A soft spring to the world oscillates about its initial length with amplitude v / sqrt(k / m).
void spring()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID b = body(w, 2, 0, 0);
    MdtSpringID s = MdtSpringCreate(w);
    MdtSpringSetBodies(s, b, 0);
    MdtSpringSetPosition(s, 0, 2, 0, 0);
    MdtSpringSetPosition(s, 1, 0, 0, 0);
    MdtSpringSetStiffness(s, 50);
    MdtSpringEnable(s);
    MdtBodySetLinearVelocity(b, 1, 0, 0);
    MeReal longest = 0;
    for (int i = 0; i < 200; ++i)
    {
        MdtWorldStep(w, dt);
        MeVector3 pb;
        MdtBodyGetPosition(b, pb);
        longest = pb[0] > longest ? pb[0] : longest;
    }
    CHECK_NEAR(longest, 2 + 1 / sqrt(50.0), 0.02);
    MdtWorldDestroy(w);
}

// The generic frame setters store attachment frames relative to each body.
void frames()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID a = body(w, 1, 0, 0);
    MdtUniversalID u = MdtUniversalCreate(w);
    MdtConstraintID c = MdtUniversalQuaConstraint(u);
    MdtUniversalSetBodies(u, a, 0);
    MdtUniversalSetPosition(u, 2, 3, 0);
    MdtConstraintSetAxis(c, 1, 0, 0);
    MeVector3 p, o;
    MdtConstraintBodyGetPositionRel(c, 0, p);
    CHECK(p[0] == 1 && p[1] == 3 && p[2] == 0);
    MdtConstraintBodyGetPositionRel(c, 1, p);
    CHECK(p[0] == 2 && p[1] == 3 && p[2] == 0);
    MdtConstraintBodyGetAxes(c, 1, p, o);
    CHECK(p[0] == 1 && fabs(o[2] + 1) < 1e-6);
    CHECK(MdtConstraintDCastUniversal(c) == u && !MdtConstraintDCastHinge(c));
    MdtWorldDestroy(w);
}

// A cone with both limit angles at zero holds the swing instead of releasing it.
void lockedCone()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(w, 0, 0, 0);
    MdtBodyID b = body(w, 1, 0, 0);
    MdtSkeletalID j = MdtSkeletalCreate(w);
    MdtSkeletalSetBodies(j, b, 0);
    MdtSkeletalSetPosition(j, 0, 0, 0);
    MdtSkeletalSetAxes(j, 1, 0, 0, 0, 1, 0);
    MdtSkeletalSetConeOption(j, MdtSkeletalConeOptionCone);
    MdtSkeletalSetConePrimaryLimitAngle(j, 0);
    MdtSkeletalSetConeSecondaryLimitAngle(j, 0);
    MdtSkeletalEnable(j);
    MdtBodySetLinearVelocity(b, 0, 1, 0);
    step(w, 50);
    MeVector3 p;
    MdtBodyGetPosition(b, p);
    CHECK(fabs(p[1]) < 0.05);
    MdtConstraintDestroy(nullptr);
    MdtWorldDestroy(w);
}

} // namespace

int main()
{
    lockedCone();
    disabledBody();
    destroyEnabled();
    hingeLimits();
    prismatic();
    spring();
    frames();
    return 0;
}
