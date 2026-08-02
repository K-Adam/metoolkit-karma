#pragma once

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <btBulletDynamicsCommon.h>

#include <unordered_set>
#include <vector>

#include <Mdt.h>

#include "BulletConvert.h"

struct MdtWorldImpl;

// Per constraint type behaviour, looked up by MdtConstraintHeader::tag. Each constraint type registers its
// table from its Create function, so adding a joint type does not touch the generic constraint code.
struct MdtConstraintOps
{
    void (*destroy)(MdtConstraintID c);
    // Called every step for enabled constraints, before Bullet solves.
    void (*prepare)(MdtWorldImpl& w, MdtConstraintID c);
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
};

inline MdtWorldImpl* impl(MdtWorldID w) { return reinterpret_cast<MdtWorldImpl*>(w); }

// The rigid body standing in for "the world" in a constraint with a null second body.
btRigidBody* mdtRigidBody(MdtWorldImpl& w, MdtBodyID b);

void mdtContactPrepare(MdtWorldImpl& w, MdtContactID c);
extern const MdtConstraintOps mdtContactOps;
