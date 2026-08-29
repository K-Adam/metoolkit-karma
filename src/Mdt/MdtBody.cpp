#include <algorithm>
#include <cstring>
#include <vector>

#include "MdtImpl.h"

namespace {

void setRotation(MdtBody& p, const btMatrix3x3& r)
{
    btTransform t(r, btVector3(p.bodyTM[3][0], p.bodyTM[3][1], p.bodyTM[3][2]));
    fromBtTransform(t, p.bodyTM);
    fromBtQuat(t.getRotation(), p.keaBody.qrot);
    mdtBodyUpdateCom(p);
}

void setInertia(MdtBody& p, const btMatrix3x3& i)
{
    btMatrix3x3 inv = i.inverse();
    MdtKeaBody& k = p.keaBody;
    MeReal* rows[] = {k.I0, k.I1, k.I2};
    MeReal* invRows[] = {k.invI0, k.invI1, k.invI2};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
        {
            rows[r][c] = i[r][c];
            invRows[r][c] = inv[r][c];
        }
}

void addVec(MeReal* acc, MeReal x, MeReal y, MeReal z)
{
    acc[0] += x;
    acc[1] += y;
    acc[2] += z;
}

void copy3(MeReal* out, const MeReal* in)
{
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
}

void setFlag(MdtBodyID b, int flag, bool on)
{
    b->keaBody.flags = on ? (b->keaBody.flags | flag) : (b->keaBody.flags & ~flag);
}

btVector3 armFromCom(MdtBodyID b, MeReal px, MeReal py, MeReal pz)
{
    return btVector3(px, py, pz) - toBt(b->comTM[3]);
}

bool touches(MdtConstraintID c, MdtBodyID b)
{
    return c->head.mdtbody[0] == b || c->head.mdtbody[1] == b;
}

bool isContact(MdtConstraintID c)
{
    return c->head.tag == MdtBclCONTACT || c->head.tag == MdtBclCONTACTGROUP;
}

// Karma keeps a body's enabled constraints in its own list, so iteration skips disabled ones.
std::vector<MdtConstraintID> enabledConstraints(MdtBodyID b)
{
    MdtWorldImpl* w = impl(b->world);
    std::vector<MdtConstraintID> out;
    for (MdtConstraintID c : w->constraints)
        if (touches(c, b) && w->enabled.count(c))
            out.push_back(c);
    return out;
}

void destroyIf(MdtBodyID b, bool contactsOnly)
{
    MdtWorldImpl* w = impl(b->world);
    for (size_t i = w->constraints.size(); i-- > 0;)
    {
        MdtConstraintID c = w->constraints[i];
        if (touches(c, b) && (!contactsOnly || isContact(c)))
            MdtConstraintDestroy(c);
    }
}

void disableIf(MdtBodyID b, bool contactsOnly)
{
    for (MdtConstraintID c : enabledConstraints(b))
        if (!contactsOnly || isContact(c))
            MdtConstraintDisable(c);
}

} // namespace

void mdtBodyUpdateCom(MdtBody& b)
{
    std::memcpy(b.comTM, b.bodyTM, sizeof(MeMatrix4));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            b.comTM[3][c] += b.com[r] * b.bodyTM[r][c];
}

extern "C" {

MdtBodyID MEAPI MdtBodyCreate(const MdtWorldID w)
{
    MdtWorldImpl* wi = impl(w);
    if ((int)wi->bodies.size() >= w->maxBodies)
    {
        MeWarning(0, "MdtBodyCreate: world already holds its maximum of %d bodies", w->maxBodies);
        return nullptr;
    }
    auto* b = new MdtBodyImpl(&wi->emptyShape);
    b->pub.world = w;
    MdtBodyReset(&b->pub);
    b->rb.setUserPointer(&b->pub);
    b->rb.forceActivationState(DISABLE_DEACTIVATION);
    wi->bodies.push_back(b);
    return &b->pub;
}

// Matches the original: the body is disabled and gets creation defaults, but keeps its sort key.
void MEAPI MdtBodyReset(const MdtBodyID b)
{
    MdtBodyDisable(b);
    MdtWorldID w = b->world;
    MeI32 sortKey = b->sortKey;
    void* model = b->model;
    *b = MdtBody{};
    b->world = w;
    b->sortKey = sortKey;
    b->model = model;
    b->keaBody.tag = MdtKeaBODYVER1;
    b->keaBody.qrot[0] = 1;
    for (int i = 0; i < 4; ++i)
        b->bodyTM[i][i] = b->comTM[i][i] = 1;
    b->keaBody.fastSpinAxis[0] = MDTBODY_DEFAULT_FAST_SPIN_X;
    b->keaBody.fastSpinAxis[1] = MDTBODY_DEFAULT_FAST_SPIN_Y;
    b->keaBody.fastSpinAxis[2] = MDTBODY_DEFAULT_FAST_SPIN_Z;
    b->damping = MDTBODY_DEFAULT_LINEAR_DAMPING;
    b->angularDamping = MDTBODY_DEFAULT_ANGULAR_DAMPING;
    b->partitionIndex = -1;
    b->safeTime = MEINFINITY;
    MdtBodySetMass(b, MDTBODY_DEFAULT_MASS * w->params.massScale);
    // The original scales the default inertia by mass only, not by lengthScale squared.
    MdtBodySetSphericalInertiaTensor(b, MDTBODY_DEFAULT_INERTIA * b->mass);
}

void MEAPI MdtBodyDestroy(const MdtBodyID b)
{
    if (!b)
        return;
    MdtWorldImpl* wi = impl(b->world);
    destroyIf(b, false);
    MdtBodyDisable(b);
    wi->bodies.erase(std::find(wi->bodies.begin(), wi->bodies.end(), impl(b)));
    delete impl(b);
}

// The copy starts disabled and without a collision model, like a newly created body.
MdtBodyID MEAPI MdtBodyCopy(const MdtBodyID b, const MdtWorldID w)
{
    MdtBodyID c = MdtBodyCreate(w);
    if (!c)
        return nullptr;
    *c = *b;
    c->world = w;
    c->model = nullptr;
    c->enabledTime = 0;
    c->partitionIndex = -1;
    return c;
}

void MEAPI MdtBodyEnable(const MdtBodyID b)
{
    if (MdtBodyIsEnabled(b))
        return;
    MdtWorldImpl* wi = impl(b->world);
    wi->world.addRigidBody(&impl(b)->rb, 0, 0);
    ++b->world->nEnabledBodies;
    b->enabledTime = 0;
    if (b->world->bodyEnableCallback)
        b->world->bodyEnableCallback(b);
}

void MEAPI MdtBodyDisable(const MdtBodyID b)
{
    if (!MdtBodyIsEnabled(b))
        return;
    impl(b->world)->world.removeRigidBody(&impl(b)->rb);
    --b->world->nEnabledBodies;
    b->enabledTime = 0;
    b->partitionIndex = -1;
    if (b->world->bodyDisableCallback)
        b->world->bodyDisableCallback(b);
}

MeBool MEAPI MdtBodyIsEnabled(const MdtBodyID b)
{
    return impl(b)->rb.isInWorld();
}

void MEAPI MdtBodyLog(const MdtBodyID b)
{
    const MdtKeaBody& k = b->keaBody;
    MeInfo(0, "MdtBody %p: enabled %d mass %g position (%g %g %g) velocity (%g %g %g) angular velocity (%g %g %g)",
           (void*)b, MdtBodyIsEnabled(b), b->mass, b->bodyTM[3][0], b->bodyTM[3][1], b->bodyTM[3][2], k.vel[0],
           k.vel[1], k.vel[2], k.velrot[0], k.velrot[1], k.velrot[2]);
}

void MEAPI MdtBodyDisableConstraints(const MdtBodyID b)
{
    disableIf(b, false);
}

void MEAPI MdtBodyDestroyConstraints(const MdtBodyID b)
{
    destroyIf(b, false);
}

void MEAPI MdtBodyDisableContacts(const MdtBodyID b)
{
    disableIf(b, true);
}

void MEAPI MdtBodyDestroyContacts(const MdtBodyID b)
{
    destroyIf(b, true);
}

MdtWorldID MEAPI MdtBodyGetWorld(const MdtBodyID b)
{
    return b->world;
}

int MEAPI MdtBodyGetPartition(const MdtBodyID b)
{
    return b->partitionIndex;
}

MeMatrix4Ptr MEAPI MdtBodyGetTransformPtr(const MdtBodyID b)
{
    return b->bodyTM;
}

void MEAPI MdtBodyGetTransform(const MdtBodyID b, MeMatrix4 m)
{
    std::memcpy(m, b->bodyTM, sizeof(MeMatrix4));
}

void MEAPI MdtBodySetTransform(const MdtBodyID b, const MeMatrix4 tm)
{
    std::memcpy(b->bodyTM, tm, sizeof(MeMatrix4));
    fromBtQuat(toBtTransform(tm).getRotation(), b->keaBody.qrot);
    mdtBodyUpdateCom(*b);
}

void* MEAPI MdtBodyGetUserData(const MdtBodyID b)
{
    return b->userData;
}

void MEAPI MdtBodySetUserData(const MdtBodyID b, void* d)
{
    b->userData = d;
}

MeReal MEAPI MdtBodyGetMass(const MdtBodyID b)
{
    return b->mass;
}

void MEAPI MdtBodySetMass(const MdtBodyID b, const MeReal mass)
{
    b->mass = mass;
    b->keaBody.invmass = 1 / mass;
}

void MEAPI MdtBodyGetInertiaTensor(const MdtBodyID b, MeMatrix3 i)
{
    copy3(i[0], b->keaBody.I0);
    copy3(i[1], b->keaBody.I1);
    copy3(i[2], b->keaBody.I2);
}

void MEAPI MdtBodySetInertiaTensor(const MdtBodyID b, const MeMatrix3 i)
{
    setInertia(*b, btMatrix3x3(i[0][0], i[0][1], i[0][2], i[1][0], i[1][1], i[1][2], i[2][0], i[2][1], i[2][2]));
}

void MEAPI MdtBodySetSphericalInertiaTensor(const MdtBodyID b, const MeReal i)
{
    setInertia(*b, btMatrix3x3(i, 0, 0, 0, i, 0, 0, 0, i));
}

void MEAPI MdtBodyGetPosition(const MdtBodyID b, MeVector3 p)
{
    copy3(p, b->bodyTM[3]);
}

void MEAPI MdtBodySetPosition(const MdtBodyID b, const MeReal x, const MeReal y, const MeReal z)
{
    b->bodyTM[3][0] = x;
    b->bodyTM[3][1] = y;
    b->bodyTM[3][2] = z;
    mdtBodyUpdateCom(*b);
}

void MEAPI MdtBodyGetQuaternion(const MdtBodyID b, MeVector4 q)
{
    std::memcpy(q, b->keaBody.qrot, sizeof(MeVector4));
}

void MEAPI MdtBodySetQuaternion(const MdtBodyID b, const MeReal qw, const MeReal qx, const MeReal qy, const MeReal qz)
{
    setRotation(*b, btMatrix3x3(toBtQuat(qw, qx, qy, qz).normalized()));
}

void MEAPI MdtBodyGetOrientation(const MdtBodyID b, MeMatrix3 R)
{
    for (int r = 0; r < 3; ++r)
        copy3(R[r], b->bodyTM[r]);
}

void MEAPI MdtBodySetOrientation(const MdtBodyID b, const MeMatrix3 R)
{
    MeMatrix4 tm;
    std::memcpy(tm, b->bodyTM, sizeof(MeMatrix4));
    for (int r = 0; r < 3; ++r)
        copy3(tm[r], R[r]);
    setRotation(*b, toBtTransform(tm).getBasis());
}

void MEAPI MdtBodyGetLinearVelocity(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.vel);
}

MeReal* MEAPI MdtBodyGetLinearVelocityPtr(const MdtBodyID b)
{
    return b->keaBody.vel;
}

void MEAPI MdtBodySetLinearVelocity(const MdtBodyID b, const MeReal dx, const MeReal dy, const MeReal dz)
{
    b->keaBody.vel[0] = dx;
    b->keaBody.vel[1] = dy;
    b->keaBody.vel[2] = dz;
}

void MEAPI MdtBodyGetAngularVelocity(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.velrot);
}

MeReal* MEAPI MdtBodyGetAngularVelocityPtr(const MdtBodyID b)
{
    return b->keaBody.velrot;
}

void MEAPI MdtBodySetAngularVelocity(const MdtBodyID b, const MeReal wx, const MeReal wy, const MeReal wz)
{
    b->keaBody.velrot[0] = wx;
    b->keaBody.velrot[1] = wy;
    b->keaBody.velrot[2] = wz;
}

void MEAPI MdtBodyGetVelocityAtPoint(const MdtBodyID b, MeVector3 p, MeVector3 v)
{
    btVector3 r = armFromCom(b, p[0], p[1], p[2]);
    fromBt(toBt(b->keaBody.vel) + toBt(b->keaBody.velrot).cross(r), v);
}

void MEAPI MdtBodyGetLinearAcceleration(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.accel);
}

void MEAPI MdtBodyGetAngularAcceleration(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.accelrot);
}

void MEAPI MdtBodyGetFastSpinAxis(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.fastSpinAxis);
}

// Bullet integrates rotation with the exponential map, which already handles fast spin, so the axis is
// only recorded.
void MEAPI MdtBodySetFastSpinAxis(const MdtBodyID b, const MeReal x, const MeReal y, const MeReal z)
{
    fromBt(btVector3(x, y, z).normalized(), b->keaBody.fastSpinAxis);
    setFlag(b, MdtKeaBodyFlagUseFastSpin, true);
}

void MEAPI MdtBodySetNoFastSpinAxis(const MdtBodyID b)
{
    setFlag(b, MdtKeaBodyFlagUseFastSpin, false);
}

MeReal MEAPI MdtBodyGetLinearVelocityDamping(const MdtBodyID b)
{
    return b->damping;
}

MeReal MEAPI MdtBodyGetAngularVelocityDamping(const MdtBodyID b)
{
    return b->angularDamping;
}

void MEAPI MdtBodySetLinearVelocityDamping(const MdtBodyID b, const MeReal d)
{
    b->damping = d;
}

void MEAPI MdtBodySetAngularVelocityDamping(const MdtBodyID b, const MeReal d)
{
    b->angularDamping = d;
}

void MEAPI MdtBodyEnableNonSphericalInertia(const MdtBodyID b)
{
    setFlag(b, MdtKeaBodyFlagIsNonSpherical, true);
}

void MEAPI MdtBodyDisableNonSphericalInertia(const MdtBodyID b)
{
    setFlag(b, MdtKeaBodyFlagIsNonSpherical, false);
}

MeBool MEAPI MdtBodyNonSphericalInertiaIsEnabled(const MdtBodyID b)
{
    return (b->keaBody.flags & MdtKeaBodyFlagIsNonSpherical) != 0;
}

void MEAPI MdtBodyEnableCoriolisForce(const MdtBodyID b)
{
    setFlag(b, MdtKeaBodyFlagAddCoriolisForce, true);
}

void MEAPI MdtBodyDisableCoriolisForce(const MdtBodyID b)
{
    setFlag(b, MdtKeaBodyFlagAddCoriolisForce, false);
}

MeBool MEAPI MdtBodyCoriolisForceIsEnabled(const MdtBodyID b)
{
    return (b->keaBody.flags & MdtKeaBodyFlagAddCoriolisForce) != 0;
}

void MEAPI MdtBodyGetForce(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.force);
}

void MEAPI MdtBodyGetTorque(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.torque);
}

MeReal MEAPI MdtBodyGetKineticEnergy(const MdtBodyID b)
{
    // The inertia tensor is in the body frame and the angular velocity in the world frame.
    btVector3 v = toBt(b->keaBody.vel);
    btVector3 w = toBtTransform(b->bodyTM).getBasis().transpose() * toBt(b->keaBody.velrot);
    btVector3 Iw(toBt(b->keaBody.I0).dot(w), toBt(b->keaBody.I1).dot(w), toBt(b->keaBody.I2).dot(w));
    return (b->mass * v.length2() + w.dot(Iw)) / 2;
}

MeReal MEAPI MdtBodyGetEnabledTime(const MdtBodyID b)
{
    return b->enabledTime;
}

// The original returns the body frame relative to the centre of mass, translation only.
void MEAPI MdtBodyGetCenterOfMassRelativeTransform(const MdtBodyID b, MeMatrix4 t)
{
    std::memset(t, 0, sizeof(MeMatrix4));
    for (int i = 0; i < 4; ++i)
        t[i][i] = 1;
    for (int i = 0; i < 3; ++i)
        t[3][i] = -b->com[i];
}

// Only the translation is used, as in the original.
void MEAPI MdtBodySetCenterOfMassRelativeTransform(const MdtBodyID b, const MeMatrix4 t)
{
    MdtBodySetCenterOfMassRelativePosition(b, t[3]);
}

void MEAPI MdtBodyGetCenterOfMassPosition(const MdtBodyID b, MeVector3 pos)
{
    copy3(pos, b->comTM[3]);
}

void MEAPI MdtBodySetCenterOfMassPosition(const MdtBodyID b, const MeVector3 pos)
{
    btVector3 rel = toBtTransform(b->bodyTM).invXform(toBt(pos));
    MeVector3 r;
    fromBt(rel, r);
    MdtBodySetCenterOfMassRelativePosition(b, r);
}

void MEAPI MdtBodyGetCenterOfMassRelativePosition(const MdtBodyID b, MeVector3 pos)
{
    copy3(pos, b->com);
}

void MEAPI MdtBodySetCenterOfMassRelativePosition(const MdtBodyID b, const MeVector3 pos)
{
    copy3(b->com, pos);
    b->useCom = pos[0] != 0 || pos[1] != 0 || pos[2] != 0;
    mdtBodyUpdateCom(*b);
}

MeMatrix4Ptr MEAPI MdtBodyGetCenterOfMassTransformPtr(const MdtBodyID b)
{
    return b->comTM;
}

MeReal MEAPI MdtBodyGetSafeTime(const MdtBodyID b)
{
    return b->safeTime;
}

void MEAPI MdtBodySetSafeTime(const MdtBodyID b, const MeReal t)
{
    b->safeTime = t;
}

MeI32 MEAPI MdtBodyGetSortKey(const MdtBodyID b)
{
    return b->sortKey;
}

void MEAPI MdtBodySetSortKey(const MdtBodyID b, const MeI32 key)
{
    b->sortKey = key;
}

MdtConstraintID MEAPI MdtBodyGetFirstConstraint(MdtBodyID b)
{
    std::vector<MdtConstraintID> cs = enabledConstraints(b);
    return cs.empty() ? nullptr : cs.front();
}

MdtConstraintID MEAPI MdtBodyGetNextConstraint(MdtBodyID b, MdtConstraintID c)
{
    std::vector<MdtConstraintID> cs = enabledConstraints(b);
    auto it = std::find(cs.begin(), cs.end(), c);
    return it == cs.end() || ++it == cs.end() ? nullptr : *it;
}

MdtBodyID MEAPI MdtBodyGetFirst(const MdtWorldID w)
{
    MdtWorldImpl* wi = impl(w);
    return wi->bodies.empty() ? nullptr : &wi->bodies.front()->pub;
}

// ponytail: linear search per call, keep an index in MdtBodyImpl if worlds grow large.
MdtBodyID MEAPI MdtBodyGetNext(const MdtBodyID b)
{
    std::vector<MdtBodyImpl*>& bodies = impl(b->world)->bodies;
    auto it = std::find(bodies.begin(), bodies.end(), impl(b));
    return it == bodies.end() || ++it == bodies.end() ? nullptr : &(*it)->pub;
}

void MEAPI MdtBodyForAllConstraints(const MdtBodyID b, MdtConstraintIteratorCBPtr cb, void* ccbdata)
{
    for (MdtConstraintID c : enabledConstraints(b))
        cb(c, ccbdata);
}

void MEAPI MdtBodyResetForces(const MdtBodyID b)
{
    std::memset(b->keaBody.force, 0, sizeof(MeVector4));
    std::memset(b->keaBody.torque, 0, sizeof(MeVector4));
}

void MEAPI MdtBodyResetImpulses(const MdtBodyID b)
{
    std::memset(b->impulseLinear, 0, sizeof(MeVector4));
    std::memset(b->impulseAngular, 0, sizeof(MeVector4));
    b->impulseAdded = 0;
}

void MEAPI MdtBodyAddForce(const MdtBodyID b, const MeReal fx, const MeReal fy, const MeReal fz)
{
    addVec(b->keaBody.force, fx, fy, fz);
}

void MEAPI MdtBodyAddTorque(const MdtBodyID b, const MeReal tx, const MeReal ty, const MeReal tz)
{
    addVec(b->keaBody.torque, tx, ty, tz);
}

void MEAPI MdtBodyAddForceAtPosition(const MdtBodyID b, const MeReal fx, const MeReal fy, const MeReal fz,
                                     const MeReal px, const MeReal py, const MeReal pz)
{
    btVector3 t = armFromCom(b, px, py, pz).cross(btVector3(fx, fy, fz));
    addVec(b->keaBody.force, fx, fy, fz);
    addVec(b->keaBody.torque, t.x(), t.y(), t.z());
}

void MEAPI MdtBodyAddImpulse(const MdtBodyID b, const MeReal ix, const MeReal iy, const MeReal iz)
{
    addVec(b->impulseLinear, ix, iy, iz);
    b->impulseAdded = 1;
}

void MEAPI MdtBodyAddImpulseAtPosition(const MdtBodyID b, const MeReal ix, const MeReal iy, const MeReal iz,
                                       const MeReal px, const MeReal py, const MeReal pz)
{
    btVector3 t = armFromCom(b, px, py, pz).cross(btVector3(ix, iy, iz));
    addVec(b->impulseLinear, ix, iy, iz);
    addVec(b->impulseAngular, t.x(), t.y(), t.z());
    b->impulseAdded = 1;
}

// The thresholds are compared against squared magnitudes, as the original does.
MeBool MEAPI MdtBodyIsMovingTest(const MdtBodyID b, const MdtPartitionParams* params)
{
    const MdtKeaBody& k = b->keaBody;
    return b->enabledTime < params->alive_time_thresh || toBt(k.vel).length2() > params->vel_thresh ||
           toBt(k.velrot).length2() > params->velrot_thresh || toBt(k.accel).length2() > params->acc_thresh ||
           toBt(k.accelrot).length2() > params->accrot_thresh;
}

} // extern "C"
