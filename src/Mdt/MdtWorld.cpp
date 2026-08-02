#include <cstring>

#include "MdtImpl.h"

namespace {

void pushBody(MdtBodyImpl& b)
{
    MdtBody& p = b.pub;
    MdtKeaBody& k = p.keaBody;
    b.rb.setMassProps(p.mass, btVector3(k.I0[0], k.I1[1], k.I2[2]));
    b.rb.updateInertiaTensor();
    btTransform t = toBtTransform(p.bodyTM);
    b.rb.setCenterOfMassTransform(t);
    b.rb.setLinearVelocity(toBt(k.vel));
    b.rb.setAngularVelocity(toBt(k.velrot));
    b.rb.applyCentralForce(toBt(k.force));
    b.rb.applyTorque(toBt(k.torque));
}

void pullBody(MdtBodyImpl& b, MeReal dt)
{
    MdtBody& p = b.pub;
    MdtKeaBody& k = p.keaBody;
    const btTransform& t = b.rb.getCenterOfMassTransform();
    fromBtTransform(t, p.bodyTM);
    std::memcpy(p.comTM, p.bodyTM, sizeof(MeMatrix4));
    fromBtQuat(t.getRotation(), k.qrot);
    btVector3 v = b.rb.getLinearVelocity(), w = b.rb.getAngularVelocity();
    fromBt((v - toBt(k.vel)) / dt, k.accel);
    fromBt((w - toBt(k.velrot)) / dt, k.accelrot);
    fromBt(v, k.vel);
    fromBt(w, k.velrot);
    k.force[0] = k.force[1] = k.force[2] = 0;
    k.torque[0] = k.torque[1] = k.torque[2] = 0;
    p.enabledTime += dt;
}

// An enabled body touching a disabled one wakes it, as Karma does before partitioning.
void wakeTouchedBodies(MdtWorldImpl& w)
{
    for (MdtConstraintID c : w.enabled)
    {
        MdtBodyID a = c->head.mdtbody[0], b = c->head.mdtbody[1];
        if (a && b && MdtBodyIsEnabled(a) != MdtBodyIsEnabled(b))
            MdtBodyEnable(MdtBodyIsEnabled(a) ? b : a);
    }
}

} // namespace

btRigidBody* mdtRigidBody(MdtWorldImpl& w, MdtBodyID b)
{
    return b ? &impl(b)->rb : &w.fixedBody;
}

extern "C" {

MdtWorldID MEAPI MdtWorldCreate(const unsigned int maxBodies, const unsigned int maxConstraints,
                                const MeReal lengthScale, const MeReal massScale)
{
    auto* w = new MdtWorldImpl();
    MdtWorld& p = w->pub;
    p.maxBodies = (int)maxBodies;
    p.maxConstraints = (int)maxConstraints;
    p.params.lengthScale = lengthScale;
    p.params.massScale = massScale;
    p.params.defaultDensity = massScale / (lengthScale * lengthScale * lengthScale);
    p.keaParams.epsilon = MDTWORLD_DEFAULT_EPSILON;
    p.keaParams.gamma = MDTWORLD_DEFAULT_GAMMA;
    p.keaParams.velocityZeroTol = MDTWORLD_DEFAULT_VELOCITY_ZERO_TOL;
    p.minSafeTime = MDTWORLD_DEFAULT_MIN_SAFETIME;
    p.toolkitVersionString = "Karma 1.2 (Bullet)";

    w->world.setGravity(btVector3(0, 0, 0));
    w->world.getSolverInfo().m_erp = p.keaParams.gamma;
    // Contact manifolds are not part of Bullet's island graph, so islands must not be split.
    w->world.getSimulationIslandManager()->setSplitIslands(false);
    w->world.addRigidBody(&w->fixedBody, 0, 0);
    return &w->pub;
}

void MEAPI MdtWorldDestroy(const MdtWorldID w)
{
    MdtWorldImpl* wi = impl(w);
    while (!wi->constraints.empty())
        MdtConstraintDestroy(wi->constraints.back());
    while (!wi->bodies.empty())
        MdtBodyDestroy(&wi->bodies.back()->pub);
    wi->world.removeRigidBody(&wi->fixedBody);
    delete wi;
}

void MEAPI MdtWorldStep(const MdtWorldID w, const MeReal stepSize)
{
    MdtWorldImpl& wi = *impl(w);
    wi.pub.keaParams.stepsize = stepSize;
    wakeTouchedBodies(wi);
    for (MdtBodyImpl* b : wi.bodies)
        if (MdtBodyIsEnabled(&b->pub))
            pushBody(*b);
    for (MdtConstraintID c : wi.constraints)
        if (wi.enabled.count(c))
            if (const MdtConstraintOps* ops = mdtConstraintOps(c->head.tag); ops && ops->prepare)
                ops->prepare(wi, c);

    wi.world.stepSimulation(stepSize, 0);

    for (btPersistentManifold* m : wi.stepManifolds)
        wi.dispatcher.releaseManifold(m);
    wi.stepManifolds.clear();
    for (MdtBodyImpl* b : wi.bodies)
        if (MdtBodyIsEnabled(&b->pub))
            pullBody(*b, stepSize);
}

int MEAPI MdtWorldGetTotalBodies(const MdtWorldID w)
{
    return (int)impl(w)->bodies.size();
}

int MEAPI MdtWorldGetEnabledBodies(const MdtWorldID w)
{
    return w->nEnabledBodies;
}

int MEAPI MdtWorldGetTotalConstraints(const MdtWorldID w)
{
    return (int)impl(w)->constraints.size();
}

int MEAPI MdtWorldGetEnabledConstraints(const MdtWorldID w)
{
    return (int)impl(w)->enabled.size();
}

int MEAPI MdtWorldGetMaxBodies(const MdtWorldID w)
{
    return w->maxBodies;
}

int MEAPI MdtWorldGetMaxConstraints(const MdtWorldID w)
{
    return w->maxConstraints;
}

MeReal MEAPI MdtWorldGetEpsilon(const MdtWorldID w)
{
    return w->keaParams.epsilon;
}

MeReal MEAPI MdtWorldGetGamma(const MdtWorldID w)
{
    return w->keaParams.gamma;
}

void MEAPI MdtWorldSetEpsilon(const MdtWorldID w, const MeReal e)
{
    // Kea's epsilon regularises its own matrix. Bullet's CFM has different units, so it is stored only.
    w->keaParams.epsilon = e;
}

void MEAPI MdtWorldSetGamma(const MdtWorldID w, const MeReal g)
{
    w->keaParams.gamma = g;
    impl(w)->world.getSolverInfo().m_erp = g;
}

void MEAPI MdtWorldGetGravity(const MdtWorldID w, MeVector3 g)
{
    g[0] = w->params.gravity[0];
    g[1] = w->params.gravity[1];
    g[2] = w->params.gravity[2];
}

void MEAPI MdtWorldSetGravity(const MdtWorldID w, const MeReal gx, const MeReal gy, const MeReal gz)
{
    w->params.gravity[0] = gx;
    w->params.gravity[1] = gy;
    w->params.gravity[2] = gz;
    impl(w)->world.setGravity(btVector3(gx, gy, gz));
}

MeReal MEAPI MdtWorldGetDefaultDensity(const MdtWorldID w)
{
    return w->params.defaultDensity;
}

} // extern "C"
