#include <Mdt.h>

#include "trace.h"

/* Unit boxes on the plane y = 0, each held by four corner contacts that follow it. Two slide under a push
   with first order slip, one with infinite and one with finite friction. Two have adhesive contacts: one is
   pulled up, first less and then more than the adhesion holds, and one starts above the plane and is pulled
   down onto it. */
typedef struct
{
    MdtBodyID body;
    MdtContactID corner[4];
} Box;

static void place(Box* b)
{
    MeVector3 p;
    int i;
    MdtBodyGetPosition(b->body, p);
    for (i = 0; i < 4; ++i)
    {
        MdtContactSetPosition(b->corner[i], p[0] + (i & 1 ? 0.5f : -0.5f), 0, p[2] + (i & 2 ? 0.5f : -0.5f));
        MdtContactSetPenetration(b->corner[i], 0.5f - p[1]);
    }
}

static Box box(MdtWorldID world, MeReal x, MeReal y, MeReal friction, MeReal slip, MeReal adhesion)
{
    Box b;
    int i;
    b.body = MdtBodyCreate(world);
    MdtBodySetPosition(b.body, x, y, 0);
    MdtBodyEnable(b.body);
    for (i = 0; i < 4; ++i)
    {
        MdtContactParamsID p;
        b.corner[i] = MdtContactCreate(world);
        MdtContactSetBodies(b.corner[i], b.body, 0);
        MdtContactSetNormal(b.corner[i], 0, 1, 0);
        p = MdtContactGetParams(b.corner[i]);
        MdtContactParamsSetType(p, MdtContactTypeFriction2D);
        MdtContactParamsSetFriction(p, friction);
        if (slip)
            MdtContactParamsSetSlip(p, slip);
        if (adhesion)
            MdtContactParamsSetMaxAdhesiveForce(p, adhesion);
        MdtContactEnable(b.corner[i]);
    }
    place(&b);
    return b;
}

static void trace(const char* name, int step, const Box* b, double tol)
{
    MeVector3 p, v, w, f, total = {0, 0, 0};
    int i, k;
    MdtBodyGetPosition(b->body, p);
    MdtBodyGetLinearVelocity(b->body, v);
    MdtBodyGetAngularVelocity(b->body, w);
    for (i = 0; i < 4; ++i)
    {
        MdtConstraintGetForce(MdtContactQuaConstraint(b->corner[i]), 0, f);
        for (k = 0; k < 3; ++k)
            total[k] += f[k];
    }
    APPROX(tol, "%s %d pos %.4f %.4f %.4f vel %.4f %.4f %.4f w %.4f %.4f %.4f f %.4f %.4f %.4f", name, step, p[0],
           p[1], p[2], v[0], v[1], v[2], w[0], w[1], w[2], total[0], total[1], total[2]);
}

int main(void)
{
    MdtWorldID world = MdtWorldCreate(10, 30, 1, 1);
    Box slip, boxed, held, gap;
    int i;

    MdtWorldSetGravity(world, 0, -10, 0);
    MdtWorldSetAutoDisable(world, 0);
    slip = box(world, 0, 0.5f, MEINFINITY, 0.05f, 0);
    boxed = box(world, 10, 0.5f, 1, 0.02f, 0);
    held = box(world, 20, 0.5f, 0, 0, 5);
    gap = box(world, 30, 0.52f, 0, 0, 5);
    EXACT("options %d %d", MdtContactParamsGetPrimarySlip(MdtContactGetParams(slip.corner[0])) > 0,
          MdtContactParamsGetMaxAdhesiveForce(MdtContactGetParams(held.corner[0])) > 0);

    for (i = 1; i <= 60; ++i)
    {
        MdtBodyAddForce(slip.body, 4, 0, 2);
        MdtBodyAddForce(boxed.body, 8, 0, -4);
        MdtBodyAddForce(held.body, 0, i <= 30 ? 25 : 40, 0);
        MdtBodyAddForce(gap.body, 0, 10, 0);
        MdtWorldStep(world, 0.02f);
        place(&slip);
        place(&boxed);
        place(&held);
        place(&gap);
        if (i % 10 != 0 && i > 3)
            continue;
        trace("slip", i, &slip, 3e-3);
        trace("boxed", i, &boxed, 2e-2);
        /* Bullet converges the adhesive rows less far in its ten iterations. */
        trace("held", i, &held, 6e-2);
        trace("gap", i, &gap, 6e-2);
    }

    MdtWorldDestroy(world);
    return 0;
}
