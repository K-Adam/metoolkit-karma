#include <algorithm>

#include <MdtMainLoop.h>
#include <MeChunk.h>

#include "MdtImpl.h"
#include "Unsupported.h"

namespace {

MdtWorldImpl& partitionWorld(const MdtPartitionOutput* po, unsigned int i)
{
    return *impl(po->bodies[po->bodiesStart[i]]->world);
}

int keaConstraintCount(const MdtPartitionOutput* po, unsigned int i)
{
    int n = 0;
    for (int k = 0; k < po->constraintsSize[i]; ++k)
        n += mdtKeaConstraintCount(po->constraints[po->constraintsStart[i] + k]);
    return n;
}

void setForce(MdtKeaForce& out, const btVector3& f, const btVector3& t)
{
    fromBt(f, out.force);
    fromBt(t, out.torque);
}

void setForces(MdtKeaForcePair& out, const MdtConstraintHeader& h)
{
    setForce(out.primary_body, toBt(h.resultForce[0]), toBt(h.resultTorque[0]));
    setForce(out.secondary_body, toBt(h.resultForce[1]), toBt(h.resultTorque[1]));
}

} // namespace

// Kea is replaced by Bullet's solver, which works on the partition's constraints directly. So the solve takes
// what MdtPackPartition prepared from the world instead of Kea rows, and leaves Kea's result: each body's
// acceleration from its applied and constraint forces, with its velocity untouched.
void mdtSolvePacked(MdtWorldImpl& w, MeReal h, MdtKeaForcePair* forces)
{
    std::vector<btCollisionObject*> bodies;
    // Bullet's solver adds the applied forces to the velocities itself.
    for (MdtBodyImpl* b : w.packedBodies)
        bodies.push_back(&b->rb);
    std::vector<btTypedConstraint*> constraints;
    for (MdtConstraintID c : w.packedConstraints)
        if (const MdtConstraintOps* ops = mdtConstraintOps(c->head.tag); ops && ops->joint)
            if (MdtJointBullet* j = ops->joint(c); j->inWorld)
                constraints.push_back(j->c.get());
    for (auto& c : w.stepConstraints)
        constraints.push_back(c.get());

    btContactSolverInfo info = w.world.getSolverInfo();
    info.m_timeStep = h;
    w.softDeltas.clear();
    w.solver.solveGroup(bodies.data(), (int)bodies.size(), w.stepManifolds.data(), (int)w.stepManifolds.size(),
                        constraints.data(), (int)constraints.size(), info, nullptr, &w.dispatcher);

    for (MdtBodyImpl* b : w.packedBodies)
    {
        MdtKeaBody& k = b->pub.keaBody;
        fromBt((b->rb.getLinearVelocity() - toBt(k.vel)) / h, k.accel);
        fromBt((b->rb.getAngularVelocity() - toBt(k.velrot)) / h, k.accelrot);
        b->rb.clearForces();
        MdtBodyResetImpulses(&b->pub);
    }

    mdtContactCollectForces(w, h);
    for (MdtConstraintID c : w.packedConstraints)
    {
        const MdtConstraintOps* ops = mdtConstraintOps(c->head.tag);
        if (MdtJointBullet* j = ops && ops->joint ? ops->joint(c) : nullptr)
        {
            const btJointFeedback& fb = j->feedback;
            setForce(forces->primary_body, fb.m_appliedForceBodyA, fb.m_appliedTorqueBodyA);
            setForce(forces->secondary_body, fb.m_appliedForceBodyB, fb.m_appliedTorqueBodyB);
            ++forces;
        }
        else if (MdtContactGroupID g = MdtConstraintDCastContactGroup(c))
            for (MdtContactID k = g->first; k; k = k->nextContact)
                setForces(*forces++, k->head);
        else
            setForces(*forces++, c->head);
    }
}

extern "C" {

void MEAPI MdtAutoDisableLastPartition(MdtPartitionOutput* po, void* cbdata)
{
    ME_UNSUPPORTED("MdtAutoDisableLastPartition");
}

unsigned int MEAPI MdtPackAllPartitions(const MdtPartitionOutput* po, const MeReal stepSize, MdtWorldParams* params,
                                        MdtKeaParameters* keaParams, MdtKeaTransformation* keaTMArray,
                                        MdtKeaConstraints* constraints)
{
    ME_UNSUPPORTED("MdtPackAllPartitions");
    return {};
}

// Pushes the partition's bodies into Bullet and prepares its constraints for MdtKeaAddConstraintForces. The
// constraints produce their rows inside Bullet's solver, so no Kea rows are written, only the counts.
unsigned int MEAPI MdtPackPartition(const MdtPartitionOutput* po, const unsigned int partitionindex,
                                    const MeReal stepSize, MdtWorldParams* params, MdtKeaParameters* keaParams,
                                    MdtKeaTransformation* keaTMArray, MdtKeaConstraints* constraints)
{
    MdtWorldImpl& w = partitionWorld(po, partitionindex);
    mdtStepCleanup(w);
    // Contacts and joints read the step from the world while they prepare and solve.
    w.pub.keaParams.stepsize = stepSize;
    w.packedStep = stepSize;

    MdtBody** bodies = po->bodies + po->bodiesStart[partitionindex];
    w.packedBodies.clear();
    for (int i = 0; i < po->bodiesSize[partitionindex]; ++i)
    {
        MdtBodyImpl& b = *impl(bodies[i]);
        mdtBodyPush(b, stepSize);
        b.rb.applyCentralForce(toBt(params->gravity) * b.pub.mass);
        std::memcpy(&keaTMArray[i], b.pub.comTM, sizeof(MeMatrix4));
        w.packedBodies.push_back(&b);
    }
    MdtBaseConstraint** first = po->constraints + po->constraintsStart[partitionindex];
    w.packedConstraints.assign(first, first + po->constraintsSize[partitionindex]);
    for (MdtConstraintID c : w.packedConstraints)
        mdtPrepare(w, c);

    int rows = po->info[partitionindex].rowCount;
    int count = keaConstraintCount(po, partitionindex);
    constraints->num_partitions = 1;
    constraints->num_rows_exc_padding_partition[0] = rows;
    constraints->num_rows_inc_padding_partition[0] = rows;
    constraints->num_constraints_partition[0] = count;
    constraints->num_rows_exc_padding = rows;
    constraints->num_rows_inc_padding = rows;
    constraints->num_constraints = count;
    return rows;
}

// The transforms are of the centres of mass, and the body frame keeps its offset from it.
void MEAPI MdtUnpackBodies(MdtKeaTransformation* keaTMArray, const unsigned int partitionindex, MdtPartitionOutput* po)
{
    MdtBody** bodies = po->bodies + po->bodiesStart[partitionindex];
    for (int i = 0; i < po->bodiesSize[partitionindex]; ++i)
    {
        MdtBody& b = *bodies[i];
        std::memcpy(b.comTM, &keaTMArray[i], sizeof(MeMatrix4));
        std::memcpy(b.bodyTM, b.comTM, sizeof(MeMatrix4));
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                b.bodyTM[3][c] -= b.com[r] * b.comTM[r][c];
        // The original only ages bodies while auto-disable is on.
        if (b.world->partitionParams.autoDisable)
            b.enabledTime += impl(b.world)->packedStep;
    }
}

// MdtKeaAddConstraintForces already wrote the results into the constraints.
unsigned int MEAPI MdtUnpackForces(const MdtKeaForcePair *force, const unsigned int partitionindex,
                                   MdtPartitionOutput* po)
{
    mdtStepCleanup(partitionWorld(po, partitionindex));
    return keaConstraintCount(po, partitionindex);
}

MeReal MEAPI MdtPartitionGetSafeTime(MdtPartitionOutput* po, int i)
{
    MeReal t = MEINFINITY;
    for (int k = 0; k < po->bodiesSize[i]; ++k)
        t = std::min(t, po->bodies[po->bodiesStart[i] + k]->safeTime);
    return t;
}

// Kea simplifies a partition whose matrix would be too large. Bullet's iterative solver has no such limit.
void MEAPI MdtLODLastPartition(MdtPartitionOutput* po, MdtPartitionParams* params)
{
}

MdtKeaConstraints* MEAPI MdtKeaConstraintsCreateFromChunk(MeChunk* chunk, int maxPartitions, int maxKeaConstraints,
                                                          int maxRows)
{
    return mdtCarve(chunk, [&](MdtChunkCarver& m) {
        MdtKeaConstraints* c = m.take<MdtKeaConstraints>(1);
        int* rowsExc = m.take<int>(maxPartitions);
        int* rowsInc = m.take<int>(maxPartitions);
        int* counts = m.take<int>(maxPartitions);
        MdtKeaForcePair* force = m.take<MdtKeaForcePair>(maxKeaConstraints);
        if (!c)
            return c;
        *c = MdtKeaConstraints{};
        c->max_partitions = maxPartitions;
        c->num_rows_exc_padding_partition = rowsExc;
        c->num_rows_inc_padding_partition = rowsInc;
        c->num_constraints_partition = counts;
        c->max_rows_exc_padding = maxRows;
        c->max_rows_inc_padding = maxRows;
        c->max_constraints = maxKeaConstraints;
        c->force = force;
        return c;
    });
}

} // extern "C"
