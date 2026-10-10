#include <algorithm>

#include "MdtImpl.h"

namespace {

constexpr int maxRows = MdtJointRows::maxRows;
static_assert(maxRows % 4 == 0, "Jstore holds rows in blocks of 4");

// Kea input for one constraint in a partition of its own, so its rows start at 0 in every vector.
struct KeaInput
{
    MdtKeaJBlockPair J[maxRows / 4];
    MeReal xi[maxRows], c[maxRows], lo[maxRows], hi[maxRows], lambda[maxRows], slip[maxRows], xgamma[maxRows];
    MdtKeaForcePair force;
    int rowsExc, rowsInc, constraints, size, offset;
    MdtKeaBodyIndexPair bodies;
    MdtKeaConstraints k;
};

// The user function sees the body as Kea would during the solve: centre of mass velocities, applied forces
// including gravity and world frame inertia.
MdtKeaBody keaBody(const MdtBody& b, const btRigidBody& rb)
{
    MdtKeaBody k = b.keaBody;
    k.len = sizeof k;
    k.flags |= MdtKeaBodyFlagIsConstrained;
    k.invmass = rb.getInvMass();
    fromBt(rb.getTotalForce(), k.force);
    fromBt(rb.getTotalTorque(), k.torque);
    btMatrix3x3 invI = rb.getInvInertiaTensorWorld(), I = invI.inverse();
    MeReal* invRows[] = {k.invI0, k.invI1, k.invI2};
    MeReal* rows[] = {k.I0, k.I1, k.I2};
    for (int i = 0; i < 3; ++i)
    {
        fromBt(invI[i], invRows[i]);
        fromBt(I[i], rows[i]);
    }
    fromBt(rb.getLinearVelocity(), k.vel);
    fromBt(rb.getAngularVelocity(), k.velrot);
    fromBtQuat(rb.getOrientation(), k.qrot);
    return k;
}

// Kea solves J v = c - (gamma + xgamma) xi / h for a force in [lo, hi], with epsilon + slip / h added to the
// diagonal in impulse units. Bullet takes that as a CFM relative to the row's effective mass but only
// converges while it is below that mass. Epsilon is only numerical softening, so it is capped there and the
// row stays coupled with the others. A row whose slip alone is that large is solved alone through lock, as
// the implicit spring with the same regularisation, and clamped.
void addRow(MdtJointRows& r, MdtRow row, btScalar rhs, btScalar lo, btScalar hi, btScalar epsilon, btScalar slip)
{
    btScalar m = r.effectiveMass(row);
    if (slip <= 0 || slip < m / 2)
    {
        row.rhs = rhs;
        row.lo = lo;
        row.hi = hi;
        row.cfm = m > 0 ? btMin((epsilon + slip) / m, btScalar(0.5)) : 0;
        r.rows[r.n++] = row;
        return;
    }
    MdtRow* soft = r.lock(row, -rhs * r.h, 1 / (r.h * r.h * (epsilon + slip)));
    soft->lo = soft->hi = btClamped(soft->lo, lo, hi);
}

// Karma hands its constraint functions the partition's body arrays and finds each body through bodyindex.
// Here the arrays hold just this constraint's bodies.
void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtUserConstraintID u = MdtConstraintDCastUserConstraint(c);
    if (!u->head.bclFunction)
        return;
    KeaInput in{};
    MdtKeaConstraints& k = in.k;
    k.max_partitions = k.max_constraints = 1;
    k.max_rows_inc_padding = k.max_rows_exc_padding = maxRows;
    k.num_rows_exc_padding_partition = &in.rowsExc;
    k.num_rows_inc_padding_partition = &in.rowsInc;
    k.num_constraints_partition = &in.constraints;
    k.Jstore = in.J;
    k.xi = in.xi;
    k.c = in.c;
    k.lo = in.lo;
    k.hi = in.hi;
    k.lambda = in.lambda;
    k.force = &in.force;
    k.slipfactor = in.slip;
    k.xgamma = in.xgamma;
    k.Jsize = &in.size;
    k.Jofs = &in.offset;
    k.Jbody = &in.bodies;

    const btRigidBody* rb[] = {r.a, r.b};
    MdtKeaBody bodies[2];
    MdtKeaBody* blist[2] = {};
    MdtKeaTransformation tlist[2];
    for (int i = 0; i < 2; ++i)
    {
        MdtBodyID b = u->head.mdtbody[i];
        fromBtTransform(b ? rb[i]->getCenterOfMassTransform() : btTransform::getIdentity(), &tlist[i].R0);
        u->head.bodyindex[i] = b ? i : MdtBclNO_BODY;
        if (b)
        {
            bodies[i] = keaBody(*b, *rb[i]);
            blist[i] = &bodies[i];
        }
    }
    const MdtWorld& w = *u->head.world;
    MdtBclSolverParameters params{r.h, w.keaParams.epsilon, r.gamma};
    u->head.bclFunction(&k, u, tlist, blist, &params);

    int n = k.num_constraints ? std::min(in.size, maxRows) : 0;
    for (int i = 0; i < n; ++i)
    {
        const MdtKeaJBlockPair& J = in.J[i / 4];
        MdtRow row;
        for (int a = 0; a < 3; ++a)
        {
            row.linA[a] = J[0].col[a][i % 4];
            row.angA[a] = J[0].col[a + 3][i % 4];
            if (blist[1])
            {
                row.linB[a] = J[1].col[a][i % 4];
                row.angB[a] = J[1].col[a + 3][i % 4];
            }
        }
        addRow(r, row, in.c[i] - (r.gamma + in.xgamma[i]) * in.xi[i] / r.h, in.lo[i] * r.h, in.hi[i] * r.h,
               w.keaParams.epsilon, in.slip[i] / r.h);
    }
}

const MdtConstraintOps ops = mdtJointOps<MdtUserConstraint>(rows);

} // namespace

extern "C" {

MdtUserConstraintID MEAPI MdtUserConstraintCreate(const MdtWorldID w)
{
    MdtUserConstraintID j = mdtJointCreate<MdtUserConstraint>(w, MdtBclUSER, ops, MdtBclGETMAXROWS);
    mdtConstraintResetFrames(j->head);
    return j;
}

MdtConstraintID MEAPI MdtUserConstraintQuaConstraint(const MdtUserConstraintID j)
{
    return mdtQua(j);
}

MdtUserConstraintID MEAPI MdtConstraintDCastUserConstraint(const MdtConstraintID c)
{
    return mdtDCast<MdtUserConstraint>(c, MdtBclUSER);
}

MdtBclAddConstraintFn MEAPI MdtUserConstraintGetFunction(const MdtUserConstraintID j)
{
    return j->head.bclFunction;
}

void* MEAPI MdtUserConstraintGetConstraintData(const MdtUserConstraintID j)
{
    return j->userConstraint;
}

void MEAPI MdtUserConstraintSetFunction(const MdtUserConstraintID j, MdtBclAddConstraintFn f)
{
    j->head.bclFunction = f;
}

void MEAPI MdtUserConstraintSetConstraintData(const MdtUserConstraintID j, void* d)
{
    j->userConstraint = d;
}

} // extern "C"
