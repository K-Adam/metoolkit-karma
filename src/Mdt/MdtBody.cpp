#include <algorithm>
#include <cstring>

#include "MdtImpl.h"

namespace {

void setRotation(MdtBody& p, const btMatrix3x3& r)
{
    btTransform t(r, btVector3(p.bodyTM[3][0], p.bodyTM[3][1], p.bodyTM[3][2]));
    fromBtTransform(t, p.bodyTM);
    std::memcpy(p.comTM, p.bodyTM, sizeof(MeMatrix4));
    fromBtQuat(t.getRotation(), p.keaBody.qrot);
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

} // namespace

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
    MdtBody& p = b->pub;
    p.keaBody.tag = MdtKeaBODYVER1;
    p.world = w;
    p.keaBody.qrot[0] = 1;
    for (int i = 0; i < 4; ++i)
        p.bodyTM[i][i] = p.comTM[i][i] = 1;
    p.keaBody.fastSpinAxis[0] = MDTBODY_DEFAULT_FAST_SPIN_X;
    p.keaBody.fastSpinAxis[1] = MDTBODY_DEFAULT_FAST_SPIN_Y;
    p.keaBody.fastSpinAxis[2] = MDTBODY_DEFAULT_FAST_SPIN_Z;

    MeReal l = w->params.lengthScale;
    MdtBodySetMass(&p, MDTBODY_DEFAULT_MASS * w->params.massScale);
    MdtBodySetSphericalInertiaTensor(&p, MDTBODY_DEFAULT_INERTIA * p.mass * l * l);

    b->rb.setUserPointer(&p);
    b->rb.forceActivationState(DISABLE_DEACTIVATION);
    wi->bodies.push_back(b);
    return &p;
}

void MEAPI MdtBodyDestroy(const MdtBodyID b)
{
    if (!b)
        return;
    MdtWorldImpl* wi = impl(b->world);
    for (size_t i = wi->constraints.size(); i-- > 0;)
    {
        MdtConstraintID c = wi->constraints[i];
        if (c->head.mdtbody[0] == b || c->head.mdtbody[1] == b)
            MdtConstraintDestroy(c);
    }
    MdtBodyDisable(b);
    wi->bodies.erase(std::find(wi->bodies.begin(), wi->bodies.end(), impl(b)));
    delete impl(b);
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
    if (b->world->bodyDisableCallback)
        b->world->bodyDisableCallback(b);
}

MeBool MEAPI MdtBodyIsEnabled(const MdtBodyID b)
{
    return impl(b)->rb.isInWorld();
}

MdtWorldID MEAPI MdtBodyGetWorld(const MdtBodyID b)
{
    return b->world;
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
    std::memcpy(b->comTM, tm, sizeof(MeMatrix4));
    fromBtQuat(toBtTransform(tm).getRotation(), b->keaBody.qrot);
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

// Bullet keeps a diagonal local inertia, so off-diagonal terms are stored here but not simulated yet.
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
    b->bodyTM[3][0] = b->comTM[3][0] = x;
    b->bodyTM[3][1] = b->comTM[3][1] = y;
    b->bodyTM[3][2] = b->comTM[3][2] = z;
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
    btVector3 r = toBt(p) - toBt(b->comTM[3]);
    fromBt(toBt(b->keaBody.vel) + toBt(b->keaBody.velrot).cross(r), v);
}

void MEAPI MdtBodyGetForce(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.force);
}

void MEAPI MdtBodyGetTorque(const MdtBodyID b, MeVector3 v)
{
    copy3(v, b->keaBody.torque);
}

void MEAPI MdtBodyResetForces(const MdtBodyID b)
{
    std::memset(b->keaBody.force, 0, sizeof(MeVector4));
    std::memset(b->keaBody.torque, 0, sizeof(MeVector4));
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
    btVector3 f(fx, fy, fz);
    btVector3 t = (btVector3(px, py, pz) - toBt(b->comTM[3])).cross(f);
    addVec(b->keaBody.force, fx, fy, fz);
    addVec(b->keaBody.torque, t.x(), t.y(), t.z());
}

MeReal MEAPI MdtBodyGetEnabledTime(const MdtBodyID b)
{
    return b->enabledTime;
}

} // extern "C"
