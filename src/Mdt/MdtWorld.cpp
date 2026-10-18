#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <unordered_map>

#include <MeMath.h>

#include "MdtImpl.h"
#include "Unsupported.h"

namespace {

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

bool finite(const MeReal* v, int n)
{
    for (int i = 0; i < n; ++i)
        if (!std::isfinite(v[i]))
            return false;
    return true;
}

// Kea reports the bodies of a partition whose solution blew up. Bullet gives no such verdict, so the bodies
// are checked after the step instead, and no Kea constraints are passed along.
void checkSim(MdtWorldImpl& w)
{
    std::vector<MdtKeaBody*> bad;
    for (MdtBodyImpl* b : w.bodies)
    {
        MdtBody& p = b->pub;
        const MdtKeaBody& k = p.keaBody;
        bool ok = finite(k.vel, 3) && finite(k.velrot, 3) && finite(k.qrot, 4) && finite(p.comTM[3], 3);
        if (MdtBodyIsEnabled(&p) && !ok)
            bad.push_back(&p.keaBody);
    }
    if (!bad.empty() && w.pub.simErrorCallback)
        w.pub.simErrorCallback(nullptr, bad.data(), (int)bad.size(), w.pub.simErrorUserData);
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
    p.simErrorCallback = MdtDefaultSimErrorCallBack;
    setDefaults(*w);

    btContactSolverInfo& info = w->world.getSolverInfo();
    // Kea has no split impulse, and the contact points carrying slip rows use the distance as a target.
    info.m_splitImpulse = 0;
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
            mdtBodyPush(*b, stepSize);
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
            mdtBodyPull(*b, stepSize);
    if (wi.pub.checkSim)
        checkSim(wi);
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

// A world as MdtWorldCreate left it, with the same capacities and scales. Callbacks stay registered.
void MEAPI MdtWorldReCreate(const MdtWorldID w)
{
    MdtWorldImpl* wi = impl(w);
    while (!wi->constraints.empty())
        MdtConstraintDestroy(wi->constraints.back());
    while (!wi->bodies.empty())
        MdtBodyDestroy(&wi->bodies.back()->pub);
    setDefaults(*wi);
}

// Karma advances a body by no more than its safe time, but at least minSafeTime, so that it stops short of
// what it would otherwise pass through. Bullet steps the world as one, so the most restrictive enabled body
// sets the step for all of them.
void MEAPI MdtWorldStepSafeTime(const MdtWorldID w, const MeReal stepSize)
{
    MeReal h = stepSize;
    for (MdtBodyImpl* b : impl(w)->bodies)
        if (MdtBodyIsEnabled(&b->pub))
            h = std::min(h, std::max(b->pub.safeTime, w->minSafeTime));
    MdtWorldStep(w, h);
}

// Bullet manages the solver memory that Kea drew from its pool.
int MEAPI MdtWorldGetMaxMemoryPoolUsed(const MdtWorldID)
{
    return 0;
}

MeChunk* MEAPI MdtWorldGetKeaPoolChunk(const MdtWorldID)
{
    ME_UNSUPPORTED("MdtWorldGetKeaPoolChunk");
    return nullptr;
}

MeChunk* MEAPI MdtWorldGetPartitionOutputChunk(const MdtWorldID)
{
    ME_UNSUPPORTED("MdtWorldGetPartitionOutputChunk");
    return nullptr;
}

MeChunk* MEAPI MdtWorldGetKeaTMChunk(const MdtWorldID)
{
    ME_UNSUPPORTED("MdtWorldGetKeaTMChunk");
    return nullptr;
}

MeChunk* MEAPI MdtWorldGetKeaConstraintsChunk(const MdtWorldID)
{
    ME_UNSUPPORTED("MdtWorldGetKeaConstraintsChunk");
    return nullptr;
}

void MEAPI MdtWorldSetKeaDebugRequest(const MdtWorldID, const MdtKeaDebugDataRequest)
{
    ME_UNSUPPORTED("MdtWorldSetKeaDebugRequest");
}

void MEAPI MdtWorldSetDebugDrawing(const MdtWorldID, const MdtDebugDrawOptions) {}

// Kea never runs, so no matrix sizes are ever written to the log.
void MEAPI MdtWorldSetMatrixSizeLog(const MdtWorldID w, int* const sizeLog, const int logSize)
{
    impl(w)->matrixSizeLog = sizeLog;
    impl(w)->matrixSizeLogSize = logSize;
}

MeBool MEAPI MdtWorldGetCheckSim(const MdtWorldID w)
{
    return w->checkSim;
}

void MEAPI MdtWorldSetCheckSim(const MdtWorldID w, const MeBool c)
{
    w->checkSim = c;
}

MdtSimErrorCBPtr MEAPI MdtWorldGetSimErrorCB(const MdtWorldID w)
{
    return w->simErrorCallback;
}

void MEAPI MdtWorldSetSimErrorCB(const MdtWorldID w, MdtSimErrorCBPtr cb, void* secbdata)
{
    w->simErrorCallback = cb;
    w->simErrorUserData = secbdata;
}

void MEAPI MdtDefaultSimErrorCallBack(MdtKeaConstraints*, MdtKeaBody** kb, int nBodies, void*)
{
    // MdtBody starts with its MdtKeaBody, so the pointers are the body IDs.
    for (int i = 0; i < nBodies; ++i)
        MeWarning(0, "MdtWorldStep: body %p has a non-finite position or velocity", (void*)kb[i]);
}

} // extern "C"
