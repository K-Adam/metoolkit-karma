#pragma once

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <btBulletDynamicsCommon.h>

#include <memory>
#include <unordered_set>
#include <vector>

#include <Mdt.h>

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

    explicit MdtBodyImpl(btCollisionShape* shape) : rb(1, nullptr, shape) {}
};

inline MdtBodyImpl* impl(MdtBodyID b) { return reinterpret_cast<MdtBodyImpl*>(b); }

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
    std::unordered_set<MdtConstraintID> enabled;
    std::vector<btPersistentManifold*> stepManifolds;
    // Contact friction rows, added to the Bullet world for one step like stepManifolds.
    std::vector<std::unique_ptr<btTypedConstraint>> stepConstraints;
};

inline MdtWorldImpl* impl(MdtWorldID w) { return reinterpret_cast<MdtWorldImpl*>(w); }

// The rigid body standing in for "the world" in a constraint with a null second body.
btRigidBody* mdtRigidBody(MdtWorldImpl& w, MdtBodyID b);

void mdtContactPrepare(MdtWorldImpl& w, MdtContactID c);
extern const MdtConstraintOps mdtContactOps;

void mdtBodyUpdateCom(MdtBody& b);

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
    btScalar h, gamma;
    // Velocity of a world body B at the world origin, and its angular velocity.
    btVector3 worldLin{0, 0, 0}, worldAng{0, 0, 0};
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

// Continues an angle in [-pi, pi] from the previous continuous value, so positions can span several turns.
inline btScalar mdtUnwrap(btScalar angle, btScalar previous) { return previous + btNormalizeAngle(angle - previous); }

// The limit's raw position from the previous step, for unwrapping angles.
inline btScalar mdtLimitPrevious(const MdtBclLimit& l, btScalar fallback)
{
    return l.bPositionInitialised ? l.position - l.offset : fallback;
}
