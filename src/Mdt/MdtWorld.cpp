#include <algorithm>
#include <cstring>
#include <numeric>
#include <unordered_map>

#include <MeMath.h>

#include "MdtImpl.h"

namespace {

// Without MdtBodyEnableNonSphericalInertia, Kea approximates the tensor by a sphere using its largest
// diagonal element. Bullet keeps a diagonal local inertia, so off-diagonal terms are dropped.
// ponytail: off-diagonal inertia ignored, needs a principal axis rigid body frame to support.
btVector3 bulletInertia(const MdtKeaBody& k)
{
    btVector3 d(k.I0[0], k.I1[1], k.I2[2]);
    if (k.flags & MdtKeaBodyFlagIsNonSpherical)
        return d;
    btScalar s = d[d.maxAxis()];
    return btVector3(s, s, s);
}

// Kea applies damping explicitly as v *= 1 - d h, and turns accumulated impulses into forces over the step.
void pushBody(MdtBodyImpl& b, MeReal dt)
{
    MdtBody& p = b.pub;
    MdtKeaBody& k = p.keaBody;
    b.rb.setMassProps(p.mass, bulletInertia(k));
    b.rb.setFlags(k.flags & MdtKeaBodyFlagAddCoriolisForce ? BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT : 0);
    b.rb.setCenterOfMassTransform(toBtTransform(p.comTM));
    b.rb.setLinearVelocity(toBt(k.vel) * (1 - p.damping * dt));
    b.rb.setAngularVelocity(toBt(k.velrot) * (1 - p.angularDamping * dt));
    b.rb.applyCentralForce(toBt(k.force) + toBt(p.impulseLinear) / dt);
    b.rb.applyTorque(toBt(k.torque) + toBt(p.impulseAngular) / dt);
}

void pullBody(MdtBodyImpl& b, MeReal dt)
{
    MdtBody& p = b.pub;
    MdtKeaBody& k = p.keaBody;
    const btTransform& t = b.rb.getCenterOfMassTransform();
    fromBtTransform(t, p.comTM);
    btTransform body(t.getBasis(), t.getOrigin() - t.getBasis() * toBt(p.com));
    fromBtTransform(body, p.bodyTM);
    fromBtQuat(t.getRotation(), k.qrot);
    btVector3 v = b.rb.getLinearVelocity(), w = b.rb.getAngularVelocity();
    fromBt((v - toBt(k.vel)) / dt, k.accel);
    fromBt((w - toBt(k.velrot)) / dt, k.accelrot);
    fromBt(v, k.vel);
    fromBt(w, k.velrot);
    MdtBodyResetForces(&p);
    MdtBodyResetImpulses(&p);
    // The original only ages bodies while auto-disable is on.
    if (p.world->partitionParams.autoDisable)
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

// Bodies joined by enabled constraints form a partition. With auto-disable on, a partition in which no body
// is moving is disabled as a whole. Constraints to the world do not join or keep alive anything.
void updatePartitions(MdtWorldImpl& w)
{
    size_t n = w.bodies.size();
    std::unordered_map<MdtBodyID, size_t> index;
    for (size_t i = 0; i < n; ++i)
        index[&w.bodies[i]->pub] = i;
    std::vector<size_t> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&](size_t i) {
        while (parent[i] != i)
            i = parent[i] = parent[parent[i]];
        return i;
    };
    for (MdtConstraintID c : w.enabled)
    {
        MdtBodyID a = c->head.mdtbody[0], b = c->head.mdtbody[1];
        if (a && b && MdtBodyIsEnabled(a) && MdtBodyIsEnabled(b))
            parent[root(index[a])] = root(index[b]);
    }

    const MdtPartitionParams& params = w.pub.partitionParams;
    std::vector<int> partOfRoot(n, -1);
    std::vector<bool> moving;
    for (size_t i = 0; i < n; ++i)
    {
        MdtBody& b = w.bodies[i]->pub;
        if (!MdtBodyIsEnabled(&b))
            continue;
        int& part = partOfRoot[root(i)];
        if (part < 0)
        {
            part = (int)moving.size();
            moving.push_back(false);
        }
        b.partitionIndex = part;
        if (MdtBodyIsMovingTest(&b, &params))
            moving[part] = true;
    }
    if (!params.autoDisable)
        return;
    for (MdtBodyImpl* b : w.bodies)
        if (MdtBodyIsEnabled(&b->pub) && !moving[b->pub.partitionIndex])
            MdtBodyDisable(&b->pub);
}

// Values the original sets in MdtWorldCreate and restores in MdtWorldReset.
void setDefaults(MdtWorldImpl& w)
{
    MdtWorld& p = w.pub;
    MeReal l = p.params.lengthScale;
    MdtWorldSetGravity(&p, MDTWORLD_DEFAULT_GRAVITY_X, MDTWORLD_DEFAULT_GRAVITY_Y, MDTWORLD_DEFAULT_GRAVITY_Z);
    MdtWorldSetEpsilon(&p, MDTWORLD_DEFAULT_EPSILON / p.params.massScale);
    MdtWorldSetGamma(&p, MDTWORLD_DEFAULT_GAMMA);
    MdtWorldSetMaxLCPIterations(&p, 10);
    p.keaParams.velocityZeroTol = MDTWORLD_DEFAULT_VELOCITY_ZERO_TOL;
    p.minSafeTime = MDTWORLD_DEFAULT_MIN_SAFETIME;

    // Only the linear thresholds scale with lengthScale, as observed in the original.
    MdtPartitionParams& pp = p.partitionParams;
    pp.autoDisable = 1;
    pp.vel_thresh = MDTWORLD_DEFAULT_VEL_THRESH * l;
    pp.velrot_thresh = MDTWORLD_DEFAULT_VELROT_THRESH;
    pp.acc_thresh = MDTWORLD_DEFAULT_ACC_THRESH * l;
    pp.accrot_thresh = MDTWORLD_DEFAULT_ACCROT_THRESH;
    pp.alive_time_thresh = MDTWORLD_DEFAULT_ALIVE_TIME_THRESH;
    pp.maxMatrixSize = MDTMAXMATRIXSIZE_MAX;
    pp.lodParams.frictionRatio = MDTWORLD_DEFAULT_FRICTION_RATIO;
    pp.lodParams.zeroRowBonus = MDTWORLD_DEFAULT_ZERO_ROW_BONUS;
    pp.lodParams.toWorldBonus = MDTWORLD_DEFAULT_TO_WORLD_BONUS;
    pp.lodParams.rowCountBias = MDTWORLD_ROW_COUNT_BIAS;
    pp.lodParams.penetrationBias = MDTWORLD_PENETRATION_BIAS;
    pp.lodParams.normVelBias = MDTWORLD_NORM_VEL_BIAS;
    pp.lodParams.nonAutoBonus = MDTWORLD_DEFAULT_NON_AUTO_BONUS;
    pp.debugOptions = MdtDebugDrawOptions(0);
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
    p.toolkitVersionString = "Karma 1.2 (Bullet)";
    setDefaults(*w);

    btContactSolverInfo& info = w->world.getSolverInfo();
    // Kea does not clamp the Coriolis term.
    info.m_maxGyroscopicForce = BT_LARGE_FLOAT;
    // mdtContactPrepare applies the per-contact restitution threshold itself.
    info.m_restitutionVelocityThreshold = 0;
    // Contact manifolds are not part of Bullet's island graph, so islands must not be split.
    w->world.getSimulationIslandManager()->setSplitIslands(false);
    w->world.addRigidBody(&w->fixedBody, 0, 0);
    return &w->pub;
}

// The original restores the world parameters and disables every constraint, keeping bodies and constraints.
void MEAPI MdtWorldReset(const MdtWorldID w)
{
    MdtWorldImpl* wi = impl(w);
    setDefaults(*wi);
    for (MdtConstraintID c : std::vector<MdtConstraintID>(wi->enabled.begin(), wi->enabled.end()))
        MdtConstraintDisable(c);
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
    updatePartitions(wi);
    for (MdtBodyImpl* b : wi.bodies)
        if (MdtBodyIsEnabled(&b->pub))
            pushBody(*b, stepSize);
    for (MdtConstraintID c : wi.constraints)
        if (wi.enabled.count(c))
            if (const MdtConstraintOps* ops = mdtConstraintOps(c->head.tag); ops && ops->prepare)
                ops->prepare(wi, c);

    wi.softDeltas.clear();
    wi.world.stepSimulation(stepSize, 0);

    mdtContactCollectForces(wi, stepSize);
    for (btPersistentManifold* m : wi.stepManifolds)
        wi.dispatcher.releaseManifold(m);
    wi.stepManifolds.clear();
    for (auto& c : wi.stepConstraints)
        wi.world.removeConstraint(c.get());
    wi.stepConstraints.clear();
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

MeReal MEAPI MdtWorldGetMinSafeTime(const MdtWorldID w)
{
    return w->minSafeTime;
}

// Contacts apply epsilon through their own CFM in mdtContactPrepare.
void MEAPI MdtWorldSetEpsilon(const MdtWorldID w, const MeReal e)
{
    w->keaParams.epsilon = e;
}

void MEAPI MdtWorldSetGamma(const MdtWorldID w, const MeReal g)
{
    w->keaParams.gamma = g;
    w->constantGamma = g;
    // Bullet uses m_erp for joints and shallow contacts, m_erp2 for the rest of the contacts.
    btContactSolverInfo& info = impl(w)->world.getSolverInfo();
    info.m_erp = info.m_erp2 = g;
}

void MEAPI MdtWorldSetGammaWithRefTimeStep(const MdtWorldID w, const MeReal aGamma, const MeReal aRefStep,
                                           const MeReal aTimeStep)
{
    MdtWorldSetGamma(w, aGamma * aTimeStep / aRefStep);
}

void MEAPI MdtWorldSetMinSafeTime(const MdtWorldID w, const MeReal t)
{
    w->minSafeTime = t;
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

MeBool MEAPI MdtWorldGetAutoDisable(const MdtWorldID w)
{
    return w->partitionParams.autoDisable;
}

void MEAPI MdtWorldSetAutoDisable(const MdtWorldID w, const MeBool d)
{
    w->partitionParams.autoDisable = d;
}

MeReal MEAPI MdtWorldGetAutoDisableVelocityThreshold(const MdtWorldID w)
{
    return w->partitionParams.vel_thresh;
}

void MEAPI MdtWorldSetAutoDisableVelocityThreshold(const MdtWorldID w, const MeReal vt)
{
    w->partitionParams.vel_thresh = vt;
}

MeReal MEAPI MdtWorldGetAutoDisableAngularVelocityThreshold(const MdtWorldID w)
{
    return w->partitionParams.velrot_thresh;
}

void MEAPI MdtWorldSetAutoDisableAngularVelocityThreshold(const MdtWorldID w, const MeReal avt)
{
    w->partitionParams.velrot_thresh = avt;
}

MeReal MEAPI MdtWorldGetAutoDisableAccelerationThreshold(const MdtWorldID w)
{
    return w->partitionParams.acc_thresh;
}

void MEAPI MdtWorldSetAutoDisableAccelerationThreshold(const MdtWorldID w, const MeReal at)
{
    w->partitionParams.acc_thresh = at;
}

MeReal MEAPI MdtWorldGetAutoDisableAngularAccelerationThreshold(const MdtWorldID w)
{
    return w->partitionParams.accrot_thresh;
}

void MEAPI MdtWorldSetAutoDisableAngularAccelerationThreshold(const MdtWorldID w, const MeReal aat)
{
    w->partitionParams.accrot_thresh = aat;
}

MeReal MEAPI MdtWorldGetAutoDisableAliveTime(const MdtWorldID w)
{
    return w->partitionParams.alive_time_thresh;
}

void MEAPI MdtWorldSetAutoDisableAliveTime(const MdtWorldID w, const MeReal aw)
{
    w->partitionParams.alive_time_thresh = aw;
}

// Bullet never builds a Kea matrix, so the size cap and LOD parameters are kept for the getters only.
int MEAPI MdtWorldGetMaxMatrixSize(const MdtWorldID w)
{
    return w->partitionParams.maxMatrixSize;
}

void MEAPI MdtWorldSetMaxMatrixSize(const MdtWorldID w, const int size)
{
    w->partitionParams.maxMatrixSize = size;
}

void MEAPI MdtWorldGetLODParams(const MdtWorldID w, MdtLODParams* const lodParams)
{
    *lodParams = w->partitionParams.lodParams;
}

void MEAPI MdtWorldSetLODParams(const MdtWorldID w, const MdtLODParams* const lodParams)
{
    w->partitionParams.lodParams = *lodParams;
}

int MEAPI MdtWorldGetMaxLCPIterations(const MdtWorldID w)
{
    return w->keaParams.max_iterations;
}

void MEAPI MdtWorldSetMaxLCPIterations(const MdtWorldID w, const int mi)
{
    w->keaParams.max_iterations = mi;
    impl(w)->world.getSolverInfo().m_numIterations = mi;
}

void MEAPI MdtWorldResetForces(const MdtWorldID w)
{
    for (MdtBodyImpl* b : impl(w)->bodies)
        MdtBodyResetForces(&b->pub);
}

void MEAPI MdtWorldForAllConstraints(const MdtWorldID w, MdtConstraintIteratorCBPtr cb, void* ccbdata)
{
    MdtWorldImpl* wi = impl(w);
    std::vector<MdtConstraintID> enabled;
    for (MdtConstraintID c : wi->constraints)
        if (wi->enabled.count(c))
            enabled.push_back(c);
    for (MdtConstraintID c : enabled)
        cb(c, ccbdata);
}

} // extern "C"
