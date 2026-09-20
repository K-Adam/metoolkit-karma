#include <Mdt.h>

#include "check.h"

int main()
{
    MdtWorldID world = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID a = MdtBodyCreate(world), b = MdtBodyCreate(world);
    MeVector3 v;

    CHECK(MdtBodyGetFirst(world) == a && MdtBodyGetNext(a) == b && MdtBodyGetNext(b) == nullptr);

    // Rotating the body frame swings the centre of mass around the body origin.
    MdtBodySetPosition(a, 1, 2, 3);
    v[0] = 1, v[1] = 0, v[2] = 0;
    MdtBodySetCenterOfMassRelativePosition(a, v);
    MdtBodySetQuaternion(a, (MeReal)0.70710678, 0, 0, (MeReal)0.70710678);
    MdtBodyGetCenterOfMassPosition(a, v);
    CHECK_NEAR(v[0], 1, 1e-5);
    CHECK_NEAR(v[1], 3, 1e-5);
    MdtBodyGetPosition(a, v);
    CHECK(v[0] == 1 && v[1] == 2 && v[2] == 3);

    // A body spinning about its offset centre of mass moves its origin, not its centre of mass.
    MdtBodySetAngularVelocity(a, 0, 0, 1);
    MdtBodyEnable(a);
    MdtWorldSetAutoDisable(world, 0);
    MdtWorldStep(world, (MeReal)0.1);
    MdtBodyGetCenterOfMassPosition(a, v);
    CHECK_NEAR(v[0], 1, 1e-5);
    CHECK_NEAR(v[1], 3, 1e-5);
    MdtBodyGetPosition(a, v);
    CHECK(fabs(v[0] - 1) > 0.05);

    // Reset restores creation defaults and disables the body, keeping its sort key.
    MdtBodySetSortKey(a, 7);
    MdtBodySetLinearVelocityDamping(a, 1);
    MdtBodyReset(a);
    CHECK(!MdtBodyIsEnabled(a) && MdtBodyGetSortKey(a) == 7 && MdtBodyGetLinearVelocityDamping(a) == 0);
    CHECK(MdtBodyGetTransformPtr(a)[3][0] == 0 && MdtBodyGetCenterOfMassTransformPtr(a)[3][0] == 0);

    // A still body joined to a moving one by an enabled contact group stays enabled with it.
    MdtWorldSetAutoDisable(world, 1);
    MdtBodyEnable(a);
    MdtBodyEnable(b);
    MdtBodySetLinearVelocity(b, 1, 0, 0);
    MdtContactGroupID g = MdtContactGroupCreate(world);
    MdtContactGroupSetBodies(g, a, b);
    MdtContactGroupEnable(g);
    for (int i = 0; i < 30; ++i)
        MdtWorldStep(world, (MeReal)0.01);
    CHECK(MdtBodyIsEnabled(a) && MdtBodyGetPartition(a) == MdtBodyGetPartition(b));
    CHECK(MdtBodyGetFirstConstraint(a) == MdtContactGroupQuaConstraint(g));
    MdtContactGroupDisable(g);
    for (int i = 0; i < 30; ++i)
        MdtWorldStep(world, (MeReal)0.01);
    CHECK(!MdtBodyIsEnabled(a) && MdtBodyIsEnabled(b) && MdtBodyGetFirstConstraint(a) == nullptr);
    MdtContactGroupDestroy(g);

    // Box friction removes at most friction * h of momentum per step.
    MdtWorldSetAutoDisable(world, 0);
    MdtWorldSetGravity(world, 0, -10, 0);
    MdtBodyReset(a);
    MdtBodyEnable(a);
    MdtBodySetLinearVelocity(a, 5, 0, 0);
    g = MdtContactGroupCreate(world);
    MdtContactGroupSetBodies(g, a, nullptr);
    MdtContactGroupEnable(g);
    MdtContactID k = MdtContactGroupCreateContact(g);
    MdtContactParamsID p = MdtContactGetParams(k);
    MdtContactParamsSetType(p, MdtContactTypeFriction2D);
    MdtContactParamsSetFriction(p, 3);
    CHECK(p->friction1 == 3 && p->velThreshold > 0);
    MdtWorldStep(world, (MeReal)0.01);
    MdtBodyGetLinearVelocity(a, v);
    CHECK_NEAR(v[0], 4.97, 1e-4);
    CHECK_NEAR(MdtContactGroupGetNormalForce(g), 10, 0.2);
    MdtConstraintGetForce(MdtContactGroupQuaConstraint(g), 0, v);
    CHECK_NEAR(v[0], -3, 1e-3);
    CHECK_NEAR(v[1], 10, 0.2);
    CHECK(MdtConstraintGetRowCount(MdtContactQuaConstraint(k)) == 3);
    CHECK(MdtConstraintGetRowCount(MdtContactGroupQuaConstraint(g)) == 3);

    // Option bits follow the values, as in the original.
    MdtContactParamsSetSlip(p, (MeReal)0.1);
    MdtContactParamsSetPrimarySlide(p, (MeReal)0.2);
    CHECK((p->options & (MdtBclContactOptionSlip1 | MdtBclContactOptionSlip2 | MdtBclContactOptionSlide1)) ==
          (MdtBclContactOptionSlip1 | MdtBclContactOptionSlip2 | MdtBclContactOptionSlide1));
    MdtContactParamsSetPrimarySlide(p, 0);
    CHECK(!(p->options & MdtBclContactOptionSlide1));
    MdtContactReset(k);
    MdtContactGetDirection(k, v);
    CHECK(v[0] == 1 && p->options == 0 && MdtContactGetContactGroup(k) == g);

    // A contact appended to a group is destroyed with it and leaves nothing behind for the next step.
    int total = MdtWorldGetTotalConstraints(world);
    MdtContactID loose = MdtContactCreate(world);
    MdtContactGroupAppendContact(g, loose);
    CHECK(MdtWorldGetTotalConstraints(world) == total + 1);
    CHECK(MdtConstraintGetRowCount(MdtContactGroupQuaConstraint(g)) == 6);
    MdtWorldStep(world, (MeReal)0.01);
    MdtContactGroupDestroy(g);
    CHECK(MdtWorldGetTotalConstraints(world) == total - 2);
    MdtWorldStep(world, (MeReal)0.01);

    MeMatrix3 I;
    MdtMakeInertiaTensorBox(2, 1, 2, 3, I);
    CHECK_NEAR(I[0][0], 13.0 / 6, 1e-6);
    MdtBodySetPosition(b, 0, 1, 0);
    v[0] = 1, v[1] = 0, v[2] = 0;
    MdtConvertPositionVector(b, v, nullptr, v);
    CHECK_NEAR(v[1], 1, 1e-6);

    MdtWorldDestroy(world);
    return 0;
}
