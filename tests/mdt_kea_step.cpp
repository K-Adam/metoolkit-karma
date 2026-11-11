#include <vector>

#include <Mdt.h>
#include <MdtKea.h>
#include <MdtMainLoop.h>
#include <MdtPartition.h>
#include <MeChunk.h>
#include <MeDict.h>
#include <MeMemory.h>
#include <MePool.h>

#include "check.h"

namespace {

// UT2004's pool takes every struct from the heap and ignores the pool size.
void* MEAPI unboundedGetStruct(MePool* pool)
{
    ++pool->u.malloc.usedStructs;
    return MeMemoryAPI.createAligned(pool->u.malloc.structSize, pool->u.malloc.alignment);
}

MdtBodyID other(MdtConstraintID c, MdtBodyID b)
{
    return c->head.mdtbody[0] == b ? c->head.mdtbody[1] : c->head.mdtbody[0];
}

// The partitioner and step UT2004 runs instead of MdtWorldStep, reduced to what it asks of the library: the
// public dictionaries and flags, then pack, Kea solve and integrate, unpack, all through the world's chunks.
void build(MdtWorldID w, MdtPartitionOutput* po)
{
    for (MeDictNode* n = MeDictFirst(&w->enabledBodyDict); n; n = MeDictNext(&w->enabledBodyDict, n))
    {
        auto* root = static_cast<MdtBodyID>(MeDictNodeGet(n));
        if (root->flags & MdtEntityAddedFlag)
            continue;
        int p = po->nPartitions++;
        po->bodiesStart[p] = po->totalBodies;
        po->constraintsStart[p] = po->totalConstraints;
        MdtPartInfoReset(&po->info[p]);
        root->flags |= MdtEntityAddedFlag;
        po->bodies[po->totalBodies++] = root;
        for (int i = po->bodiesStart[p]; i < po->totalBodies; ++i)
        {
            MdtBodyID b = po->bodies[i];
            for (MeDictNode* m = MeDictFirst(&b->constraintDict); m; m = MeDictNext(&b->constraintDict, m))
            {
                auto* c = static_cast<MdtConstraintID>(MeDictNodeGet(m));
                if (c->head.flags & MdtEntityAddedFlag)
                    continue;
                c->head.flags |= MdtEntityAddedFlag;
                po->constraints[po->totalConstraints++] = c;
                MdtPartInfoAddConstraint(&po->info[p], c);
                MdtBodyID o = other(c, b);
                if (o && !(o->flags & MdtEntityAddedFlag))
                {
                    MdtBodyEnable(o);
                    o->flags |= MdtEntityAddedFlag;
                    po->bodies[po->totalBodies++] = o;
                }
            }
        }
        po->bodiesSize[p] = po->totalBodies - po->bodiesStart[p];
        po->constraintsSize[p] = po->totalConstraints - po->constraintsStart[p];
    }
    for (int i = 0; i < po->totalBodies; ++i)
        po->bodies[i]->flags &= ~MdtEntityAddedFlag;
    for (int i = 0; i < po->totalConstraints; ++i)
        po->constraints[i]->head.flags &= ~MdtEntityAddedFlag;
}

void step(MdtWorldID w, MeReal h)
{
    MdtPartitionOutput* po = MdtPartOutCreateFromChunk(&w->partOutChunk, w->nBodies, w->nEnabledConstraints);
    CHECK(po);
    build(w, po);
    for (int p = 0; p < po->nPartitions; ++p)
    {
        int n = po->bodiesSize[p];
        auto* tm = static_cast<MdtKeaTransformation*>(MeChunkGetMem(&w->keaTMChunk, n * sizeof(MdtKeaTransformation)));
        MdtKeaConstraints* kc = MdtKeaConstraintsCreateFromChunk(
            &w->keaConstraintsChunk, 1, po->info[p].contactCount + po->info[p].jointCount, po->info[p].rowCount);
        MdtPackPartition(po, p, h, &w->params, &w->keaParams, tm, kc);
        auto** bodies = reinterpret_cast<MdtKeaBody**>(&po->bodies[po->bodiesStart[p]]);
        w->keaParams.stepsize = h;
        std::vector<MeReal> before;
        for (int i = 0; i < n; ++i)
            before.insert(before.end(), bodies[i]->vel, bodies[i]->vel + 3);
        MdtKeaAddConstraintForces(*kc, bodies, tm, n, w->keaParams);
        for (int i = 0; i < n; ++i)
            for (int k = 0; k < 3; ++k)
                CHECK(bodies[i]->vel[k] == before[3 * i + k]);
        MdtKeaIntegrateSystem(bodies, tm, n, w->keaParams);
        MdtUnpackBodies(tm, p, po);
        CHECK(MdtUnpackForces(kc->force, p, po) == (unsigned)kc->num_constraints);
        MeChunkPutMem(&w->keaConstraintsChunk, kc);
        MeChunkPutMem(&w->keaTMChunk, tm);
    }
    MeChunkPutMem(&w->partOutChunk, po);
    for (MeDictNode* n = MeDictFirst(&w->enabledBodyDict); n; n = MeDictNext(&w->enabledBodyDict, n))
        MdtBodyResetForces(static_cast<MdtBodyID>(MeDictNodeGet(n)));
}

int count(MeDict& d)
{
    int n = 0;
    for (MeDictNode* m = MeDictFirst(&d); m; m = MeDictNext(&d, m))
        ++n;
    return n;
}

} // namespace

int main()
{
    // UT2004 creates its world for one body and one constraint and lifts the caps by allocating from the heap.
    struct MePoolAPI fixed = MePoolAPI;
    MePoolAPI = MePoolMallocAPI;
    MePoolAPI.getStruct = unboundedGetStruct;
    MdtWorldID w = MdtWorldCreate(1, 1, 1, 1);
    MdtWorldSetAutoDisable(w, 0);
    MdtWorldSetGravity(w, 0, -10, 0);
    const MeReal h = (MeReal)0.01;

    // A free body falls exactly as Kea integrates it: velocity first, then position.
    MdtBodyID falling = MdtBodyCreate(w);
    MdtBodySetPosition(falling, 0, 100, 0);
    MdtBodyEnable(falling);
    const int steps = 50;
    for (int i = 0; i < steps; ++i)
        step(w, h);
    MeVector3 p, v;
    MdtBodyGetPosition(falling, p);
    MdtBodyGetLinearVelocity(falling, v);
    CHECK_NEAR(v[1], -10 * h * steps, 1e-3);
    CHECK_NEAR(p[1], 100 - 10 * h * h * steps * (steps + 1) / 2, 1e-3);

    // A body held up by a contact to the world stays put, and the contact carries its weight.
    MdtBodyID resting = MdtBodyCreate(w);
    MdtBodySetPosition(resting, 10, 0, 0);
    MdtBodyEnable(resting);
    MdtContactGroupID g = MdtContactGroupCreate(w);
    MdtConstraintSetBodies(MdtContactGroupQuaConstraint(g), resting, nullptr);
    MdtContactID c = MdtContactGroupCreateContact(g);
    MdtContactSetPosition(c, 10, 0, 0);
    MdtContactSetNormal(c, 0, 1, 0);
    MdtConstraintEnable(MdtContactGroupQuaConstraint(g));
    CHECK(count(resting->constraintDict) == 1 && w->nEnabledConstraints == 1);
    for (int i = 0; i < steps; ++i)
        step(w, h);
    MdtBodyGetPosition(resting, p);
    CHECK_NEAR(p[1], 0, 1e-3);
    MeVector3 f;
    MdtContactGetForce(c, 0, f);
    CHECK_NEAR(f[1], 10 * MdtBodyGetMass(resting), 1e-2);

    // A body hanging from a ball joint keeps its distance to the pivot while it swings.
    MdtBodyID bob = MdtBodyCreate(w);
    MdtBodySetPosition(bob, 20, 0, 0);
    MdtBodySetLinearVelocity(bob, 0, 0, 1);
    MdtBodyEnable(bob);
    MdtBSJointID joint = MdtBSJointCreate(w);
    MdtConstraintSetBodies(MdtBSJointQuaConstraint(joint), bob, nullptr);
    MdtConstraintSetPosition(MdtBSJointQuaConstraint(joint), 20, 2, 0);
    MdtConstraintEnable(MdtBSJointQuaConstraint(joint));
    for (int i = 0; i < steps; ++i)
        step(w, h);
    MdtBodyGetPosition(bob, p);
    MeReal dx = p[0] - 20, dy = p[1] - 2, dz = p[2];
    CHECK_NEAR(dx * dx + dy * dy + dz * dz, 4, 1e-2);
    CHECK(p[2] > 0.1);

    // Moving an enabled constraint to other bodies moves its links, and a reset keeps the body listed.
    MdtConstraintSetBodies(MdtBSJointQuaConstraint(joint), falling, nullptr);
    CHECK(count(bob->constraintDict) == 0 && count(falling->constraintDict) == 1);
    MdtConstraintSetBodies(MdtBSJointQuaConstraint(joint), bob, nullptr);
    MdtBodyReset(falling);
    MdtBodyEnable(falling);
    CHECK(count(w->bodyDict) == 3 && count(falling->constraintDict) == 0);

    // The public lists follow enabling, disabling and destruction.
    CHECK(w->nBodies == 3 && count(w->bodyDict) == 3 && count(w->enabledBodyDict) == 3);
    MdtBodyDisable(falling);
    CHECK(count(w->enabledBodyDict) == 2 && !(falling->flags & MdtEntityEnabledFlag));
    MdtConstraintDisable(MdtBSJointQuaConstraint(joint));
    CHECK(count(bob->constraintDict) == 0 && count(w->constraintDict) == 1 && w->nEnabledConstraints == 1);
    MdtBodyDestroy(resting);
    CHECK(w->nBodies == 2 && count(w->constraintDict) == 0 && w->nEnabledConstraints == 0);

    MdtWorldDestroy(w);

    // The default pool keeps Karma's capacity.
    MePoolAPI = fixed;
    w = MdtWorldCreate(1, 1, 1, 1);
    CHECK(MdtBodyCreate(w) != nullptr && MdtBodyCreate(w) == nullptr);
    MdtWorldDestroy(w);
    return 0;
}
