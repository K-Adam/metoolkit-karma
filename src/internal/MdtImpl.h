#pragma once

#include <btBulletDynamicsCommon.h>

#include <cstddef>
#include <cstring>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Mdt.h>
#include <MeChunk.h>
#include <MeDict.h>

#include "BulletConvert.h"

struct MdtWorldImpl;
struct MdtJointBullet;
struct MdtJointRows;

// Per constraint type behaviour, looked up by MdtConstraintHeader::tag. Each constraint type registers its
// table from its Create function, so adding a joint type does not touch the generic constraint code.
struct MdtConstraintOps
{
    void (*destroy)(MdtConstraintID c);
    // Called every step for enabled constraints, before Bullet solves.
    void (*prepare)(MdtWorldImpl& w, MdtConstraintID c);
    // Joints only. The Bullet constraint, and the rows the joint adds to it each step.
    MdtJointBullet* (*joint)(MdtConstraintID c);
    void (*rows)(MdtConstraintID c, MdtJointRows& r);
};

void mdtRegisterConstraintOps(int tag, const MdtConstraintOps* ops);
const MdtConstraintOps* mdtConstraintOps(int tag);
void mdtConstraintInit(MdtConstraintHeader& head, MdtWorldID w, int tag);

struct MdtBodyImpl
{
    MdtBody pub{};
    btRigidBody rb;
    // Orientation at the start of the step, which mdtBodyPull integrates itself.
    btQuaternion startRotation;

    explicit MdtBodyImpl(btCollisionShape* shape) : rb(1, nullptr, shape) {}
};

inline MdtBodyImpl* impl(MdtBodyID b) { return reinterpret_cast<MdtBodyImpl*>(b); }

// Velocity change already committed to a body by the soft rows fixed earlier in this step.
struct MdtSoftDelta
{
    btVector3 lin{0, 0, 0}, ang{0, 0, 0};
};
using MdtSoftDeltas = std::unordered_map<const btRigidBody*, MdtSoftDelta>;

// Bullet only runs dynamics here. Bodies never enter its broadphase and every contact comes from MdtContact
// constraints, written into the dispatcher as manifolds for one step.
struct MdtWorldImpl
{
    MdtWorld pub{};
    btDefaultCollisionConfiguration config;
    btCollisionDispatcher dispatcher{&config};
    btDbvtBroadphase broadphase;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world{&dispatcher, &broadphase, &solver, &config};
    btEmptyShape emptyShape;
    btRigidBody fixedBody{0, nullptr, &emptyShape};
    std::vector<MdtBodyImpl*> bodies;
    std::vector<MdtConstraintID> constraints;
    std::vector<btPersistentManifold*> stepManifolds;
    // Contact friction rows, added to the Bullet world for one step like stepManifolds.
    std::vector<std::unique_ptr<btTypedConstraint>> stepConstraints;
    MdtSoftDeltas softDeltas;
    int* matrixSizeLog = nullptr;
    int matrixSizeLogSize = 0;
    // Collision models live in Mcd, so Mst unbinds a body's model through this.
    void (*bodyDestroyHook)(MdtBodyID) = nullptr;
    // The partition MdtPackPartition prepared for the next MdtKeaAddConstraintForces.
    std::vector<MdtBodyImpl*> packedBodies;
    std::vector<MdtConstraintID> packedConstraints;
    MeReal packedStep = 0;
};

// Lays a struct and the arrays after it out in one MeChunk block. A first pass with a null base only sizes
// the block, a second one hands out the pointers.
struct MdtChunkCarver
{
    char* base = nullptr;
    size_t used = 0;

    template <class T>
    T* take(size_t count)
    {
        used = (used + alignof(std::max_align_t) - 1) / alignof(std::max_align_t) * alignof(std::max_align_t);
        T* p = base ? reinterpret_cast<T*>(base + used) : nullptr;
        used += count * sizeof(T);
        return p;
    }
};

// Runs layout once to size the block and once on the memory taken from chunk, returning what it lays out.
template <class Layout>
auto mdtCarve(MeChunk* chunk, Layout layout)
{
    MdtChunkCarver size;
    layout(size);
    MdtChunkCarver block{static_cast<char*>(MeChunkGetMem(chunk, (int)size.used))};
    return block.base ? layout(block) : nullptr;
}

inline MdtWorldImpl* impl(MdtWorldID w) { return reinterpret_cast<MdtWorldImpl*>(w); }

// impl() casts a public pointer to its impl struct, which needs the public struct at offset 0. These structs hold
// Bullet objects with virtual functions, so offsetof is only conditionally supported and GCC warns about it.
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif
static_assert(offsetof(MdtBodyImpl, pub) == 0);
static_assert(offsetof(MdtWorldImpl, pub) == 0);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

// The rigid body standing in for "the world" in a constraint with a null second body.
btRigidBody* mdtRigidBody(MdtWorldImpl& w, MdtBodyID b);

void mdtContactPrepare(MdtWorldImpl& w, MdtContactID c);
// After the solve, writes the forces of the contacts prepared this step and of their groups.
void mdtContactCollectForces(MdtWorldImpl& w, MeReal dt);

// Releases the contact manifolds and rows prepared for the last solve.
void mdtStepCleanup(MdtWorldImpl& w);
void mdtPrepare(MdtWorldImpl& w, MdtConstraintID c);
// Constraints Kea counts for c: one per joint or loose contact, one per contact of a group.
int mdtKeaConstraintCount(MdtConstraintID c);
// Solves the packed partition and leaves each body's constraint and applied forces as Kea accelerations.
void mdtSolvePacked(MdtWorldImpl& w, MeReal h, MdtKeaForcePair* forces);

// MeDict ordering for the world and body dictionaries, which are keyed by object address.
int mdtComparePointers(const void* a, const void* b);
void mdtDictAdd(MeDict& d, MeDictNode& n, void* object);
template <class T>
T mdtDictGet(MeDictNode* n)
{
    return n ? static_cast<T>(MeDictNodeGet(n)) : nullptr;
}
// Copies a dictionary's members into a vector, for walks whose callbacks may change it.
template <class T>
std::vector<T> mdtDictItems(MeDict& d)
{
    std::vector<T> out;
    for (MeDictNode* n = MeDictFirst(&d); n; n = MeDictNext(&d, n))
        out.push_back(static_cast<T>(MeDictNodeGet(n)));
    return out;
}

void mdtBodyUpdateCom(MdtBody& b);
// Copy an enabled body's state into its rigid body before a step, and back after it.
void mdtBodyPush(MdtBodyImpl& b, MeReal dt);
void mdtBodyPull(MdtBodyImpl& b, MeReal dt);

// One constraint equation J v + cfm lambda = rhs, solved for an impulse lambda in [lo, hi].
struct MdtRow
{
    btVector3 linA{0, 0, 0}, angA{0, 0, 0}, linB{0, 0, 0}, angB{0, 0, 0};
    btScalar rhs = 0, lo = -BT_LARGE_FLOAT, hi = BT_LARGE_FLOAT, cfm = 0;
};

// The rows a joint adds in one step. fa and fb are the constraint frame as attached to body A and body B, in
// world space. Every row is written so that J v is the rate of change of its error.
struct MdtJointRows
{
    static constexpr int maxRows = 8;
    const btRigidBody* a;
    const btRigidBody* b;
    btTransform fa, fb;
    btScalar h, gamma, epsilon;
    // Velocity of a world body B at the world origin, and its angular velocity.
    btVector3 worldLin{0, 0, 0}, worldAng{0, 0, 0};
    MdtSoftDeltas* softDeltas = nullptr;
    MdtRow rows[maxRows];
    int n = 0;

    btVector3 axisA(int i) const { return fa.getBasis().getColumn(i); }
    btVector3 axisB(int i) const { return fb.getBasis().getColumn(i); }
    // d/dt of (onA - onB).dir, with onA moving with A and onB with B.
    MdtRow linear(const btVector3& dir, const btVector3& onA, const btVector3& onB) const;
    // Rate of rotation of A relative to B about axis.
    MdtRow angular(const btVector3& axis) const;
    btScalar velocity(const MdtRow& r) const;
    btScalar effectiveMass(const MdtRow& r) const;
    // Adds j driving error to zero, hard when kp is infinite and as a damped spring otherwise. Adds nothing
    // when kp and kd are both zero.
    MdtRow* lock(const MdtRow& j, btScalar error, btScalar kp = MEINFINITY, btScalar kd = 0);
    // Records the impulses of this joint's soft rows for the soft rows of joints solved after it.
    void commitSoftRows();
    // Adds a motor row driving j at velocity v with force at most fmax.
    MdtRow& motor(const MdtRow& j, btScalar v, btScalar fmax);
    // Restricts r to pushing its error down (upper) or up.
    static void oneSided(MdtRow& r, bool upper);
    void lockPoint();
    // Aligns only the primary axes.
    void lockAxis(btScalar kp = MEINFINITY, btScalar kd = 0);
    void lockRotation(btScalar kp = MEINFINITY, btScalar kd = 0);
};

btVector3 mdtOrthogonal(const btVector3& p);

// World space rotation vector taking frame rb to frame ra.
btVector3 mdtRotationError(const btMatrix3x3& ra, const btMatrix3x3& rb);

// Updates the limit's position and velocity along dof and adds its limit, motor or lock row.
void mdtLimitRows(MdtJointRows& r, MdtBclLimit& l, const MdtRow& dof, btScalar position);

// The Bullet side of a joint: one persistent constraint, in the world while the joint is enabled and its
// bodies are.
struct MdtJointBullet
{
    std::unique_ptr<btTypedConstraint> c;
    btJointFeedback feedback{};
    bool inWorld = false;
};

template <class Pub>
struct MdtJointImpl
{
    Pub pub{};
    MdtJointBullet bt;
};

void mdtJointPrepare(MdtWorldImpl& w, MdtConstraintID c);
void mdtJointRemove(MdtWorldImpl& w, MdtJointBullet& j);
void mdtConstraintResetFrames(MdtConstraintHeader& head);

inline void mdtClearResults(MdtConstraintHeader& h)
{
    std::memset(h.resultForce, 0, sizeof h.resultForce);
    std::memset(h.resultTorque, 0, sizeof h.resultTorque);
}

template <class Pub>
MdtJointBullet* mdtJointBullet(MdtConstraintID c)
{
    return &reinterpret_cast<MdtJointImpl<Pub>*>(c)->bt;
}

template <class Pub>
void mdtJointDestroy(MdtConstraintID c)
{
    auto* j = reinterpret_cast<MdtJointImpl<Pub>*>(c);
    mdtJointRemove(*impl(c->head.world), j->bt);
    delete j;
}

template <class Pub>
constexpr MdtConstraintOps mdtJointOps(void (*rows)(MdtConstraintID, MdtJointRows&))
{
    return {mdtJointDestroy<Pub>, mdtJointPrepare, mdtJointBullet<Pub>, rows};
}

template <class Pub>
Pub* mdtJointCreate(MdtWorldID w, int tag, const MdtConstraintOps& ops, int maxRows)
{
    auto* j = new MdtJointImpl<Pub>();
    mdtRegisterConstraintOps(tag, &ops);
    mdtConstraintInit(j->pub.head, w, tag);
    j->pub.head.maxRows = maxRows;
    return &j->pub;
}

template <class Pub>
MdtConstraintID mdtQua(Pub* j)
{
    return reinterpret_cast<MdtConstraintID>(j);
}

template <class Pub>
Pub* mdtDCast(MdtConstraintID c, int tag)
{
    return c && c->head.tag == tag ? reinterpret_cast<Pub*>(c) : nullptr;
}

// Constraint frame of body i (0 or 1) relative to that body, and the body's own transform (identity for the
// world).
btTransform mdtRef(MdtConstraintID c, unsigned i);
void mdtSetRef(MdtConstraintID c, unsigned i, const btTransform& t);
btTransform mdtBodyTransform(MdtBodyID b);
// Constraint frame of body i in world space.
btTransform mdtFrameWorld(MdtConstraintID c, unsigned i);
