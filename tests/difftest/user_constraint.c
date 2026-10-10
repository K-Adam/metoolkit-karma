/* User constraints written as Kea rows. A ball and socket holds a body to the world and a second ball and
   socket hangs another body from it. A one sided row, added only while violated, keeps the first body out of
   x < -0.4 and also exercises xgamma. Anchors are found through the transformations passed in. */
#include <Mdt.h>

#include "trace.h"

typedef struct
{
    MeReal local[2][3];
    MeReal world[3];
} Socket;

static void anchor(const MdtKeaTransformation* t, const MeReal* local, MeReal* arm, MeReal* p)
{
    int k;
    for (k = 0; k < 3; ++k)
    {
        arm[k] = local[0] * t->R0[k] + local[1] * t->R1[k] + local[2] * t->R2[k];
        p[k] = t->pos[k] + arm[k];
    }
}

static void clearRow(MdtKeaJBlockPair* J, int row)
{
    int b, c;
    for (b = 0; b < 2; ++b)
        for (c = 0; c < 6; ++c)
            J[row / 4][b].col[c][row % 4] = 0;
}

/* The angular part is arm x e_k. */
static void pointRow(MdtKeaJBlock* J, int row, int k, const MeReal* arm, MeReal sign)
{
    J->col[k][row] = sign;
    J->col[3 + (k + 1) % 3][row] = sign * arm[(k + 2) % 3];
    J->col[3 + (k + 2) % 3][row] = -sign * arm[(k + 1) % 3];
}

static void setRow(MdtKeaConstraints* k, int r, MeReal xi, MeReal lo, MeReal xgamma)
{
    k->xi[r] = xi;
    k->c[r] = 0;
    k->lo[r] = lo;
    k->hi[r] = MEINFINITY;
    k->slipfactor[r] = 0;
    k->xgamma[r] = xgamma;
}

/* As Karma's own constraint functions do, the bodies are found through bodyindex, Jacobian rows are placed
   after the padded rows and the other vectors after the unpadded ones. */
static void MEAPI socket(MdtKeaConstraints* const k, void* const constraint, const MdtKeaTransformation* const tlist,
                         MdtKeaBody* const blist[], const void* const params)
{
    MdtUserConstraintID u = constraint;
    const Socket* s = MdtUserConstraintGetConstraintData(u);
    const int b0 = u->head.bodyindex[0], b1 = u->head.bodyindex[1];
    const int jrow = k->num_rows_inc_padding, row = k->num_rows_exc_padding;
    MeReal arm0[3], arm1[3], p0[3], p1[3];
    int i, n = 3;
    (void)blist;
    (void)params;
    k->Jbody[k->num_constraints][0] = b0;
    k->Jbody[k->num_constraints][1] = b1;
    anchor(&tlist[b0], s->local[0], arm0, p0);
    if (b1 != MdtBclNO_BODY)
        anchor(&tlist[b1], s->local[1], arm1, p1);
    for (i = 0; i < 3; ++i)
    {
        int r = jrow + i;
        clearRow(k->Jstore, r);
        pointRow(&k->Jstore[r / 4][0], r % 4, i, arm0, 1);
        if (b1 != MdtBclNO_BODY)
            pointRow(&k->Jstore[r / 4][1], r % 4, i, arm1, -1);
        setRow(k, row + i, p0[i] - (b1 != MdtBclNO_BODY ? p1[i] : s->world[i]), -MEINFINITY, 0);
    }
    if (b1 == MdtBclNO_BODY && tlist[b0].pos[0] < -0.4f)
    {
        clearRow(k->Jstore, jrow + 3);
        k->Jstore[(jrow + 3) / 4][0].col[0][(jrow + 3) % 4] = 1;
        setRow(k, row + 3, tlist[b0].pos[0] + 0.4f, 0, 0.3f);
        n = 4;
    }
    MdtBclEndConstraint(k, n);
}

static void force(const char* name, int step, MdtUserConstraintID c)
{
    MeVector3 f, t;
    MdtUserConstraintGetForce(c, 0, f);
    MdtUserConstraintGetTorque(c, 0, t);
    APPROX(0.05, "%s %d f %.4f %.4f %.4f t %.4f %.4f %.4f", name, step, f[0], f[1], f[2], t[0], t[1], t[2]);
}

int main(void)
{
    MdtWorldID world = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID bob = MdtBodyCreate(world), tail = MdtBodyCreate(world);
    MdtUserConstraintID top = MdtUserConstraintCreate(world), link = MdtUserConstraintCreate(world);
    MdtBSJointID bs = MdtBSJointCreate(world);
    Socket topSocket = {{{-1, 0, 0}, {0, 0, 0}}, {0, 0, 0}};
    Socket linkSocket = {{{0, -0.5f, 0}, {0, 0.5f, 0}}, {0, 0, 0}};
    MeMatrix4 tm;
    MeVector3 p, q;
    int i;

    EXACT("new fn %d data %d rows %d", MdtUserConstraintGetFunction(top) == 0,
          MdtUserConstraintGetConstraintData(top) == 0, MdtConstraintGetRowCount(MdtUserConstraintQuaConstraint(top)));
    EXACT("dcast %d %d", MdtConstraintDCastUserConstraint(MdtUserConstraintQuaConstraint(top)) == top,
          MdtConstraintDCastUserConstraint(MdtBSJointQuaConstraint(bs)) == 0);

    MdtWorldSetGravity(world, 0, -9.81f, 0);
    /* Enough iterations for Bullet to converge, so forces compare the row conventions and not the solvers. */
    MdtWorldSetMaxLCPIterations(world, 200);
    MdtBodySetPosition(bob, 1, 0, 0);
    MdtBodySetLinearVelocity(bob, 0, 0, 1);
    MdtBodyEnable(bob);

    MdtUserConstraintSetFunction(top, socket);
    MdtUserConstraintSetConstraintData(top, &topSocket);
    MdtUserConstraintSetBodies(top, bob, 0);
    MdtUserConstraintEnable(top);
    EXACT("set fn %d data %d rows %d", MdtUserConstraintGetFunction(top) == socket,
          MdtUserConstraintGetConstraintData(top) == &topSocket,
          MdtConstraintGetRowCount(MdtUserConstraintQuaConstraint(top)));

    for (i = 1; i <= 120; ++i)
    {
        MdtWorldStep(world, 1.0f / 60);
        if (i % 10 == 0)
        {
            MdtBodyGetPosition(bob, p);
            APPROX(0.003, "single %d pos %.4f %.4f %.4f", i, p[0], p[1], p[2]);
            force("single", i, top);
        }
    }

    MdtBodyGetTransform(bob, tm);
    MdtBodySetPosition(tail, p[0] - 0.5f * tm[1][0], p[1] - 0.5f * tm[1][1] - 0.5f, p[2] - 0.5f * tm[1][2]);
    MdtBodyEnable(tail);
    MdtUserConstraintSetFunction(link, socket);
    MdtUserConstraintSetConstraintData(link, &linkSocket);
    MdtUserConstraintSetBodies(link, bob, tail);
    MdtUserConstraintEnable(link);
    for (i = 1; i <= 120; ++i)
    {
        MdtWorldStep(world, 1.0f / 60);
        if (i % 10 == 0)
        {
            MdtBodyGetPosition(bob, p);
            MdtBodyGetPosition(tail, q);
            APPROX(0.003, "double %d pos %.4f %.4f %.4f tail %.4f %.4f %.4f", i, p[0], p[1], p[2], q[0], q[1], q[2]);
        }
    }

    MdtWorldDestroy(world);
    return 0;
}
