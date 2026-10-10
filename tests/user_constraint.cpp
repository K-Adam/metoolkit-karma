#include <cmath>

#include <Mdt.h>

#include "check.h"

namespace {

// A ball and socket to the world point (0, 0, 0) at the body's origin offset `local`, written as Karma's
// constraint functions write rows.
const MeReal local[3] = {-1, 0, 0};

void MEAPI socket(MdtKeaConstraints* const k, void* const constraint, const MdtKeaTransformation* const tlist,
                  MdtKeaBody* const[], const void* const)
{
    auto u = static_cast<MdtUserConstraintID>(constraint);
    const MdtKeaTransformation& t = tlist[u->head.bodyindex[0]];
    k->Jbody[k->num_constraints][0] = u->head.bodyindex[0];
    k->Jbody[k->num_constraints][1] = u->head.bodyindex[1];
    MeReal arm[3];
    for (int i = 0; i < 3; ++i)
        arm[i] = local[0] * t.R0[i] + local[1] * t.R1[i] + local[2] * t.R2[i];
    for (int i = 0; i < 3; ++i)
    {
        int j = k->num_rows_inc_padding + i, r = k->num_rows_exc_padding + i;
        MdtKeaJBlock& J = k->Jstore[j / 4][0];
        J.col[i][j % 4] = 1;
        J.col[3 + (i + 1) % 3][j % 4] = arm[(i + 2) % 3];
        J.col[3 + (i + 2) % 3][j % 4] = -arm[(i + 1) % 3];
        k->xi[r] = t.pos[i] + arm[i];
        k->lo[r] = -MEINFINITY;
        k->hi[r] = MEINFINITY;
    }
    MdtBclEndConstraint(k, 3);
}

MdtBodyID pendulum(MdtWorldID w)
{
    MdtWorldSetGravity(w, 0, -10, 0);
    MdtBodyID b = MdtBodyCreate(w);
    MdtBodySetPosition(b, 1, 0, 0);
    MdtBodySetLinearVelocity(b, 0, 0, 1);
    MdtBodyEnable(b);
    return b;
}

void tracksBSJoint()
{
    MdtWorldID wb = MdtWorldCreate(10, 10, 1, 1), wu = MdtWorldCreate(10, 10, 1, 1);
    MdtBodyID bb = pendulum(wb), bu = pendulum(wu);
    MdtBSJointID bs = MdtBSJointCreate(wb);
    MdtBSJointSetBodies(bs, bb, 0);
    MdtBSJointSetPosition(bs, 0, 0, 0);
    MdtBSJointEnable(bs);
    MdtUserConstraintID uc = MdtUserConstraintCreate(wu);
    MdtUserConstraintSetFunction(uc, socket);
    MdtUserConstraintSetBodies(uc, bu, 0);
    MdtUserConstraintEnable(uc);
    CHECK(MdtConstraintDCastUserConstraint(MdtUserConstraintQuaConstraint(uc)) == uc);
    CHECK(MdtConstraintDCastUserConstraint(MdtBSJointQuaConstraint(bs)) == nullptr);
    for (int i = 0; i < 200; ++i)
    {
        MdtWorldStep(wb, (MeReal)1 / 60);
        MdtWorldStep(wu, (MeReal)1 / 60);
        MeVector3 pb, pu, fb, fu;
        MdtBodyGetPosition(bb, pb);
        MdtBodyGetPosition(bu, pu);
        MdtBSJointGetForce(bs, 0, fb);
        MdtUserConstraintGetForce(uc, 0, fu);
        for (int k = 0; k < 3; ++k)
        {
            CHECK_NEAR(pb[k], pu[k], 1e-2);
            CHECK_NEAR(fb[k], fu[k], 0.5);
        }
    }
    MdtWorldDestroy(wb);
    MdtWorldDestroy(wu);
}

// Padding counts as the original library keeps them.
void rowBookkeeping()
{
    int exc[2], inc[2], count[2], size[3], ofs[3];
    MdtKeaConstraints k{};
    k.num_rows_exc_padding_partition = exc;
    k.num_rows_inc_padding_partition = inc;
    k.num_constraints_partition = count;
    k.Jsize = size;
    k.Jofs = ofs;
    MdtBclInitConstraintRowList(&k);
    MdtBclStartPartition(&k);
    MdtBclEndConstraint(&k, 3);
    CHECK(k.num_rows_exc_padding == 3 && k.num_rows_inc_padding == 7);
    MdtBclEndConstraint(&k, 5);
    CHECK(k.num_rows_exc_padding == 8 && k.num_rows_inc_padding == 12 && ofs[1] == 3 && size[1] == 5);
    MdtBclEndPartition(&k);
    MdtBclStartPartition(&k);
    MdtBclEndConstraint(&k, 3);
    CHECK(k.num_rows_inc_padding == 19 && inc[1] == 7 && exc[1] == 3 && ofs[2] == 0);
    MdtBclEndPartition(&k);
    CHECK(k.num_partitions == 2 && k.num_constraints == 3 && count[0] == 2 && count[1] == 1);
    CHECK(k.num_rows_exc_padding == 12 && k.num_rows_inc_padding == 16 && inc[1] == 4);
}

} // namespace

int main()
{
    tracksBSJoint();
    rowBookkeeping();
    return 0;
}
