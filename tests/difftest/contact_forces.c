/* Contact and contact group forces, row counts, normal force friction and group ownership of contacts. The
   normal force model shares the group's normal force equally and falls back to box friction without a group. */
#include <Mdt.h>

#include "trace.h"

static void force(const char* name, int step, MdtConstraintID c)
{
    MeVector3 f0, f1, t0;
    MdtConstraintGetForce(c, 0, f0);
    MdtConstraintGetForce(c, 1, f1);
    MdtConstraintGetTorque(c, 0, t0);
    APPROX(2e-2, "%s %d f0 %.4f %.4f %.4f f1 %.4f %.4f %.4f t0 %.4f %.4f %.4f", name, step, f0[0], f0[1], f0[2],
           f1[0], f1[1], f1[2], t0[0], t0[1], t0[2]);
}

static void friction(MdtContactID c)
{
    MdtContactParamsID p = MdtContactGetParams(c);
    MdtContactParamsSetType(p, MdtContactTypeFriction2D);
    MdtContactParamsSetPrimaryFrictionModel(p, MdtFrictionModelNormalForce);
    MdtContactParamsSetSecondaryFrictionModel(p, MdtFrictionModelNormalForce);
    MdtContactParamsSetPrimaryFrictionCoeffecient(p, 0.5f);
    MdtContactParamsSetSecondaryFrictionCoeffecient(p, 0.5f);
}

static MdtContactID contact(MdtContactID c, MdtBodyID b, MeReal x)
{
    MdtConstraintSetBodies(MdtContactQuaConstraint(c), b, 0);
    MdtContactSetPosition(c, x, 0, 0);
    MdtContactSetNormal(c, 0, 1, 0);
    friction(c);
    return c;
}

int main(void)
{
    MdtWorldID world = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID alone = MdtBodyCreate(world), grouped = MdtBodyCreate(world);
    MdtContactID c, k1, k2;
    MdtContactGroupID g;
    MeVector3 v;
    int i;

    MdtWorldSetGravity(world, 0, -10, 0);
    MdtWorldSetAutoDisable(world, 0);
    MdtBodySetMass(alone, 1);
    MdtBodySetPosition(alone, 0, 0.5f, 0);
    MdtBodyEnable(alone);
    MdtBodySetMass(grouped, 1);
    MdtBodySetPosition(grouped, 10, 0.5f, 0);
    MdtBodyEnable(grouped);

    c = contact(MdtContactCreate(world), alone, 0);
    MdtContactParamsSetFriction(MdtContactGetParams(c), 0.7f);
    EXACT("model %d", MdtContactParamsGetPrimaryFrictionModel(MdtContactGetParams(c)));
    MdtContactEnable(c);
    EXACT("contact rows %d", MdtConstraintGetRowCount(MdtContactQuaConstraint(c)));

    g = MdtContactGroupCreate(world);
    MdtConstraintSetBodies(MdtContactGroupQuaConstraint(g), grouped, 0);
    k1 = contact(MdtContactGroupCreateContact(g), grouped, 9.6f);
    k2 = contact(MdtContactGroupCreateContact(g), grouped, 10.8f);
    MdtContactParamsSetFriction(MdtContactGetParams(k1), 0.7f);
    MdtContactGroupEnable(g);
    EXACT("group rows %d total %d", MdtConstraintGetRowCount(MdtContactGroupQuaConstraint(g)),
          MdtWorldGetTotalConstraints(world));

    for (i = 1; i <= 10; ++i)
    {
        MdtBodyAddForce(alone, 3, 0, 0);
        MdtBodyAddForce(grouped, 4.5f, 0, 0);
        MdtWorldStep(world, 0.02f);
        if (i > 2 && i < 10)
            continue;
        MdtBodyGetLinearVelocity(alone, v);
        APPROX(2e-3, "alone vel %d %.4f %.4f %.4f", i, v[0], v[1], v[2]);
        MdtBodyGetLinearVelocity(grouped, v);
        APPROX(2e-3, "grouped vel %d %.4f %.4f %.4f", i, v[0], v[1], v[2]);
        force("contact", i, MdtContactQuaConstraint(c));
        force("group", i, MdtContactGroupQuaConstraint(g));
        force("k1", i, MdtContactQuaConstraint(k1));
        force("k2", i, MdtContactQuaConstraint(k2));
        APPROX(2e-2, "group normal %d %.4f", i, MdtContactGroupGetNormalForce(g));
    }

    /* A contact appended to a group is destroyed with it. */
    MdtContactDisable(c);
    MdtContactGroupAppendContact(g, c);
    EXACT("appended total %d count %d rows %d", MdtWorldGetTotalConstraints(world), MdtContactGroupGetCount(g),
          MdtConstraintGetRowCount(MdtContactGroupQuaConstraint(g)));
    MdtWorldStep(world, 0.02f);
    MdtContactGroupDestroy(g);
    EXACT("destroyed total %d", MdtWorldGetTotalConstraints(world));
    MdtWorldStep(world, 0.02f);

    MdtWorldDestroy(world);
    return 0;
}
