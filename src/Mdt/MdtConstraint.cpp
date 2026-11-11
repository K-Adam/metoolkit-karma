#include <algorithm>
#include <functional>

#include <MeMath.h>

#include "MdtImpl.h"

namespace {

const MdtConstraintOps* gOps[MdtBclUSER + 1];

// Bullet solves every joint through this class. The joint type supplies its rows each step through its ops.
class MdtBulletJoint : public btTypedConstraint
{
public:
    MdtBulletJoint(btRigidBody& a, btRigidBody& b, MdtConstraintID c)
        : btTypedConstraint(D6_CONSTRAINT_TYPE, a, b), c(c)
    {
    }

    btTransform frameA, frameB;

    void getInfo1(btConstraintInfo1* info) override
    {
        const MdtWorld& w = c->head.world[0];
        r.a = &getRigidBodyA();
        r.b = &getRigidBodyB();
        r.fa = getRigidBodyA().getCenterOfMassTransform() * frameA;
        r.fb = getRigidBodyB().getCenterOfMassTransform() * frameB;
        r.h = w.keaParams.stepsize;
        r.gamma = w.keaParams.gamma;
        r.epsilon = w.keaParams.epsilon;
        // A null second body is the world, which may move at the velocity set on the constraint.
        const MdtConstraintHeader& head = c->head;
        r.worldAng = head.mdtbody[1] ? btVector3(0, 0, 0) : toBt(head.worldAngVel);
        r.worldLin = head.mdtbody[1] ? btVector3(0, 0, 0) : toBt(head.worldLinVel) - r.worldAng.cross(r.fb.getOrigin());
        r.n = 0;
        r.softDeltas = &impl(c->head.world)->softDeltas;
        mdtConstraintOps(c->head.tag)->rows(c, r);
        r.commitSoftRows();
        info->m_numConstraintRows = r.n;
        info->nub = 0;
    }

    void getInfo2(btConstraintInfo2* info) override
    {
        for (int i = 0; i < r.n; ++i)
        {
            const MdtRow& row = r.rows[i];
            int s = i * info->rowskip;
            for (int k = 0; k < 3; ++k)
            {
                info->m_J1linearAxis[s + k] = row.linA[k];
                info->m_J1angularAxis[s + k] = row.angA[k];
                info->m_J2linearAxis[s + k] = row.linB[k];
                info->m_J2angularAxis[s + k] = row.angB[k];
            }
            info->m_constraintError[s] = row.rhs - row.linB.dot(r.worldLin) - row.angB.dot(r.worldAng);
            info->cfm[s] = row.cfm;
            info->m_lowerLimit[s] = row.lo;
            info->m_upperLimit[s] = row.hi;
        }
    }

    void setParam(int, btScalar, int) override {}
    btScalar getParam(int, int) const override { return 0; }

private:
    MdtConstraintID c;
    MdtJointRows r;
};

MdtJointBullet* joint(MdtConstraintID c)
{
    const MdtConstraintOps* ops = mdtConstraintOps(c->head.tag);
    return ops && ops->joint ? ops->joint(c) : nullptr;
}

void setWorldPosition(MdtConstraintID c, unsigned i, const btVector3& p)
{
    btTransform r = mdtRef(c, i);
    r.setOrigin(mdtBodyTransform(c->head.mdtbody[i]).invXform(p));
    mdtSetRef(c, i, r);
}

// Columns are the primary axis, the orthogonal axis and their cross product.
btMatrix3x3 basis(btVector3 p, btVector3 o)
{
    p.normalize();
    o = (o - p * p.dot(o)).normalized();
    btVector3 t = p.cross(o);
    return btMatrix3x3(p.x(), o.x(), t.x(), p.y(), o.y(), t.y(), p.z(), o.z(), t.z());
}

void setRelativeAxes(MdtConstraintID c, unsigned i, const btMatrix3x3& rel)
{
    btTransform r = mdtRef(c, i);
    r.setBasis(rel);
    mdtSetRef(c, i, r);
}

void setWorldAxes(MdtConstraintID c, unsigned i, const btMatrix3x3& world)
{
    setRelativeAxes(c, i, mdtBodyTransform(c->head.mdtbody[i]).getBasis().transpose() * world);
}

bool isEnabled(MdtConstraintID c)
{
    return (c->head.flags & MdtEntityEnabledFlag) != 0;
}

// An enabled constraint is listed in its world and in each of its bodies, as Karma's partitioner walks them.
void link(MdtConstraintID c)
{
    MdtConstraintHeader& h = c->head;
    mdtDictAdd(h.world->constraintDict, h.worldNode, c);
    for (int i = 0; i < MdtKeaMAXBODYCONSTRAINT; ++i)
        if (h.mdtbody[i])
            mdtDictAdd(h.mdtbody[i]->constraintDict, h.bodyNode[i], c);
}

void unlink(MdtConstraintID c)
{
    MdtConstraintHeader& h = c->head;
    MeDictDelete(&h.world->constraintDict, &h.worldNode);
    for (int i = 0; i < MdtKeaMAXBODYCONSTRAINT; ++i)
        if (h.mdtbody[i])
            MeDictDelete(&h.mdtbody[i]->constraintDict, &h.bodyNode[i]);
}

} // namespace

int mdtComparePointers(const void* a, const void* b)
{
    std::less<const void*> less;
    return less(a, b) ? -1 : less(b, a) ? 1 : 0;
}

void mdtDictAdd(MeDict& d, MeDictNode& n, void* object)
{
    MeDictNodeInit(&n, object);
    MeDictInsert(&d, &n, object);
}

void mdtPrepare(MdtWorldImpl& w, MdtConstraintID c)
{
    if (const MdtConstraintOps* ops = mdtConstraintOps(c->head.tag); ops && ops->prepare)
        ops->prepare(w, c);
}

int mdtKeaConstraintCount(MdtConstraintID c)
{
    MdtContactGroupID g = MdtConstraintDCastContactGroup(c);
    return g ? g->count : 1;
}

void mdtRegisterConstraintOps(int tag, const MdtConstraintOps* ops)
{
    gOps[tag] = ops;
}

const MdtConstraintOps* mdtConstraintOps(int tag)
{
    return tag >= 0 && tag <= MdtBclUSER ? gOps[tag] : nullptr;
}

void mdtConstraintInit(MdtConstraintHeader& head, MdtWorldID w, int tag)
{
    head.world = w;
    head.tag = tag;
    head.bodyindex[0] = head.bodyindex[1] = MdtBclNO_BODY;
    impl(w)->constraints.push_back(reinterpret_cast<MdtConstraintID>(&head));
}

void mdtConstraintResetFrames(MdtConstraintHeader& head)
{
    MeMatrix4TMMakeIdentity(head.ref1);
    MeMatrix4TMMakeIdentity(head.ref2);
}

btTransform mdtBodyTransform(MdtBodyID b)
{
    return b ? toBtTransform(b->bodyTM) : btTransform::getIdentity();
}

btTransform mdtRef(MdtConstraintID c, unsigned i)
{
    return toBtTransform(i == 0 ? c->head.ref1 : c->head.ref2);
}

void mdtSetRef(MdtConstraintID c, unsigned i, const btTransform& t)
{
    fromBtTransform(t, i == 0 ? c->head.ref1 : c->head.ref2);
}

btTransform mdtFrameWorld(MdtConstraintID c, unsigned i)
{
    // mdtRef treats any index past 0 as body 2, so the body lookup must too.
    i = btMin(i, 1u);
    return mdtBodyTransform(c->head.mdtbody[i]) * mdtRef(c, i);
}

// Matches the orthogonal axis Karma picks when only the primary axis is given.
btVector3 mdtOrthogonal(const btVector3& p)
{
    btVector3 e = btFabs(p.x()) > btFabs(p.y()) ? btVector3(0, 1, 0) : btVector3(1, 0, 0);
    return e.cross(p).normalized();
}

btVector3 mdtRotationError(const btMatrix3x3& ra, const btMatrix3x3& rb)
{
    btQuaternion q;
    (ra * rb.transpose()).getRotation(q);
    if (q.w() < 0)
        q = -q;
    btVector3 v(q.x(), q.y(), q.z());
    btScalar s = v.length();
    return s < SIMD_EPSILON ? v * 2 : v * (2 * btAtan2(s, q.w()) / s);
}

MdtRow MdtJointRows::linear(const btVector3& dir, const btVector3& onA, const btVector3& onB) const
{
    MdtRow r;
    r.linA = dir;
    r.angA = (onA - a->getCenterOfMassPosition()).cross(dir);
    r.linB = -dir;
    r.angB = -(onB - b->getCenterOfMassPosition()).cross(dir);
    return r;
}

MdtRow MdtJointRows::angular(const btVector3& axis) const
{
    MdtRow r;
    r.angA = axis;
    r.angB = -axis;
    return r;
}

btScalar MdtJointRows::velocity(const MdtRow& r) const
{
    return r.linA.dot(a->getLinearVelocity()) + r.angA.dot(a->getAngularVelocity()) +
           r.linB.dot(b->getLinearVelocity() + worldLin) + r.angB.dot(b->getAngularVelocity() + worldAng);
}

btScalar MdtJointRows::effectiveMass(const MdtRow& r) const
{
    return a->getInvMass() * r.linA.length2() + r.angA.dot(a->getInvInertiaTensorWorld() * r.angA) +
           b->getInvMass() * r.linB.length2() + r.angB.dot(b->getInvInertiaTensorWorld() * r.angB);
}

namespace {

// The velocity change this step's applied forces will cause, which Bullet adds before solving.
btScalar forcedVelocity(const btRigidBody& body, const btVector3& lin, const btVector3& ang, btScalar h)
{
    return h * (lin.dot(body.getTotalForce() * body.getInvMass()) +
                ang.dot(body.getInvInertiaTensorWorld() * body.getTotalTorque()));
}

} // namespace

// A spring kp, kd integrated implicitly over h is the constraint with erp = h kp / (h kp + kd) and, in
// impulse units, cfm C = 1 / (h (h kp + kd)). As observed in the original, Kea takes the world epsilon off
// that, down to zero, and then regularises the row by epsilon h in force units like any other, so a larger
// epsilon stiffens springs. Bullet's joint iteration only converges for a cfm below the row's effective mass m, which a soft
// spring exceeds, so the row's regularised impulse is solved here on its own and fixed through its bounds.
// Soft rows see the impulses fixed by earlier joints this step, one Gauss-Seidel sweep, so springs sharing a
// light body do not each push it the whole way.
// Soft rows ignore coupling with hard rows, solve them together if that shows.
MdtRow* MdtJointRows::lock(const MdtRow& j, btScalar error, btScalar kp, btScalar kd)
{
    btScalar d = h * kp + kd;
    if (kp < MEINFINITY && d <= 0)
        return nullptr;
    MdtRow& r = rows[n++] = j;
    if (kp >= MEINFINITY)
    {
        r.rhs = -gamma / h * error;
        return &r;
    }
    btScalar v = velocity(r) + forcedVelocity(*a, r.linA, r.angA, h) + forcedVelocity(*b, r.linB, r.angB, h);
    auto pending = [&](const btRigidBody* body, const btVector3& lin, const btVector3& ang) {
        auto it = softDeltas->find(body);
        return it == softDeltas->end() ? btScalar(0) : lin.dot(it->second.lin) + ang.dot(it->second.ang);
    };
    v += pending(a, r.linA, r.angA) + pending(b, r.linB, r.angB);
    r.lo = r.hi = (-kp / d * error - v) / (effectiveMass(r) + btMax(1 / d - epsilon, btScalar(0)) / h + epsilon);
    return &r;
}

// lo == hi marks a soft row.
void MdtJointRows::commitSoftRows()
{
    auto apply = [&](const btRigidBody* body, const btVector3& lin, const btVector3& ang, btScalar impulse) {
        if (body->getInvMass() == 0)
            return;
        MdtSoftDelta& d = (*softDeltas)[body];
        d.lin += lin * (body->getInvMass() * impulse);
        d.ang += body->getInvInertiaTensorWorld() * ang * impulse;
    };
    for (int i = 0; i < n; ++i)
        if (rows[i].lo == rows[i].hi && rows[i].lo != 0)
        {
            apply(a, rows[i].linA, rows[i].angA, rows[i].lo);
            apply(b, rows[i].linB, rows[i].angB, rows[i].lo);
        }
}

MdtRow& MdtJointRows::motor(const MdtRow& j, btScalar v, btScalar fmax)
{
    MdtRow& r = rows[n++] = j;
    r.rhs = v;
    r.lo = -fmax * h;
    r.hi = fmax * h;
    return r;
}

void MdtJointRows::oneSided(MdtRow& r, bool upper)
{
    if (upper)
    {
        r.hi = btMin(r.hi, btScalar(0));
        r.lo = btMin(r.lo, r.hi);
    }
    else
    {
        r.lo = btMax(r.lo, btScalar(0));
        r.hi = btMax(r.hi, r.lo);
    }
}

void MdtJointRows::lockPoint()
{
    btVector3 pa = fa.getOrigin(), pb = fb.getOrigin();
    for (int k = 0; k < 3; ++k)
    {
        btVector3 e(0, 0, 0);
        e[k] = 1;
        lock(linear(e, pa, pb), pa[k] - pb[k]);
    }
}

void MdtJointRows::lockAxis(btScalar kp, btScalar kd)
{
    btVector3 ab = axisB(0).cross(axisA(0));
    for (int k = 1; k < 3; ++k)
        lock(angular(axisA(k)), ab.dot(axisA(k)), kp, kd);
}

void MdtJointRows::lockRotation(btScalar kp, btScalar kd)
{
    btVector3 e = mdtRotationError(fa.getBasis(), fb.getBasis());
    for (int k = 0; k < 3; ++k)
        lock(angular(axisA(k)), e.dot(axisA(k)), kp, kd);
}

void mdtJointRemove(MdtWorldImpl& w, MdtJointBullet& j)
{
    if (j.inWorld)
        w.world.removeConstraint(j.c.get());
    j.inWorld = false;
    j.feedback = btJointFeedback{};
}

// A joint is in Bullet's world only while both its bodies are. MdtBodyDisable takes a body out of the world
// but leaves its constraints attached.
void mdtJointPrepare(MdtWorldImpl& w, MdtConstraintID c)
{
    MdtJointBullet& j = *joint(c);
    MdtBodyID b0 = c->head.mdtbody[0], b1 = c->head.mdtbody[1];
    if (!b0 || !MdtBodyIsEnabled(b0) || (b1 && !MdtBodyIsEnabled(b1)))
    {
        mdtJointRemove(w, j);
        return;
    }
    btRigidBody* a = mdtRigidBody(w, b0);
    btRigidBody* b = mdtRigidBody(w, b1);
    if (!j.c || &j.c->getRigidBodyA() != a || &j.c->getRigidBodyB() != b)
    {
        mdtJointRemove(w, j);
        j.c = std::make_unique<MdtBulletJoint>(*a, *b, c);
        j.c->setJointFeedback(&j.feedback);
        j.c->enableFeedback(true);
    }
    auto* bj = static_cast<MdtBulletJoint*>(j.c.get());
    bj->frameA = a->getCenterOfMassTransform().inverse() * mdtBodyTransform(b0) * mdtRef(c, 0);
    bj->frameB = b->getCenterOfMassTransform().inverse() * mdtBodyTransform(b1) * mdtRef(c, 1);
    if (!j.inWorld)
        w.world.addConstraint(j.c.get(), true);
    j.inWorld = true;
}

extern "C" {

void MEAPI MdtConstraintDestroy(const MdtConstraintID c)
{
    if (!c)
        return;
    MdtConstraintDisable(c);
    MdtWorldImpl* w = impl(c->head.world);
    auto it = std::find(w->constraints.begin(), w->constraints.end(), c);
    if (it != w->constraints.end())
        w->constraints.erase(it);
    mdtConstraintOps(c->head.tag)->destroy(c);
}

void MEAPI MdtConstraintEnable(const MdtConstraintID c)
{
    if (isEnabled(c))
        return;
    link(c);
    c->head.flags |= MdtEntityEnabledFlag;
    ++c->head.world->nEnabledConstraints;
}

void MEAPI MdtConstraintDisable(const MdtConstraintID c)
{
    MdtWorldImpl* w = impl(c->head.world);
    if (MdtJointBullet* j = joint(c))
        mdtJointRemove(*w, *j);
    if (!isEnabled(c))
        return;
    unlink(c);
    c->head.flags &= ~MdtEntityEnabledFlag;
    --c->head.world->nEnabledConstraints;
}

MeBool MEAPI MdtConstraintIsEnabled(const MdtConstraintID c)
{
    return isEnabled(c);
}

MdtBodyID MEAPI MdtConstraintGetBody(const MdtConstraintID c, const unsigned int bodyindex)
{
    return bodyindex < MdtKeaMAXBODYCONSTRAINT ? c->head.mdtbody[bodyindex] : nullptr;
}

void MEAPI MdtConstraintSetBodies(const MdtConstraintID c, const MdtBodyID b1, const MdtBodyID b2)
{
    // The Bullet constraint references the old bodies, which may be destroyed before the next step.
    if (MdtJointBullet* j = joint(c); j && (c->head.mdtbody[0] != b1 || c->head.mdtbody[1] != b2))
    {
        mdtJointRemove(*impl(c->head.world), *j);
        j->c.reset();
    }
    bool enabled = isEnabled(c);
    if (enabled)
        unlink(c);
    c->head.mdtbody[0] = b1;
    c->head.mdtbody[1] = b2;
    if (enabled)
        link(c);
    if (c->head.setBodyFunc)
        c->head.setBodyFunc(c, b1, b2);
}

void* MEAPI MdtConstraintGetUserData(const MdtConstraintID c)
{
    return c->head.userData;
}

void MEAPI MdtConstraintSetUserData(const MdtConstraintID c, void* d)
{
    c->head.userData = d;
}

MdtWorldID MEAPI MdtConstraintGetWorld(const MdtConstraintID c)
{
    return c->head.world;
}

MeI32 MEAPI MdtConstraintGetSortKey(const MdtConstraintID c)
{
    return c->head.sortKey;
}

void MEAPI MdtConstraintSetSortKey(const MdtConstraintID c, MeI32 key)
{
    c->head.sortKey = key;
}

void MEAPI MdtConstraintGetForce(const MdtConstraintID c, const unsigned int bodyindex, MeVector3 f)
{
    if (const MdtJointBullet* j = joint(c))
        fromBt(bodyindex == 0 ? j->feedback.m_appliedForceBodyA : j->feedback.m_appliedForceBodyB, f);
    else
        MeVector3Copy(f, c->head.resultForce[bodyindex & 1]);
}

void MEAPI MdtConstraintGetTorque(const MdtConstraintID c, const unsigned int bodyindex, MeVector3 t)
{
    if (const MdtJointBullet* j = joint(c))
        fromBt(bodyindex == 0 ? j->feedback.m_appliedTorqueBodyA : j->feedback.m_appliedTorqueBodyB, t);
    else
        MeVector3Copy(t, c->head.resultTorque[bodyindex & 1]);
}

int MEAPI MdtConstraintGetRowCount(const MdtConstraintID c)
{
    return c->head.maxRows;
}

void MEAPI MdtConstraintGetPosition(const MdtConstraintID c, MeVector3 p)
{
    fromBt(mdtFrameWorld(c, 0).getOrigin(), p);
}

void MEAPI MdtConstraintBodyGetPosition(const MdtConstraintID c, const unsigned int bodyindex, MeVector3 p)
{
    fromBt(mdtFrameWorld(c, bodyindex).getOrigin(), p);
}

void MEAPI MdtConstraintBodyGetPositionRel(const MdtConstraintID c, const unsigned int bodyindex, MeVector3 p)
{
    fromBt(mdtRef(c, bodyindex).getOrigin(), p);
}

void MEAPI MdtConstraintBodyGetAxis(const MdtConstraintID c, const unsigned int bodyindex, MeVector3 a)
{
    fromBt(mdtFrameWorld(c, bodyindex).getBasis().getColumn(0), a);
}

void MEAPI MdtConstraintGetAxes(const MdtConstraintID c, MeVector3 p, MeVector3 o)
{
    MdtConstraintBodyGetAxes(c, 0, p, o);
}

void MEAPI MdtConstraintBodyGetAxes(const MdtConstraintID c, const unsigned int bodyindex, MeVector3 p,
                                    MeVector3 o)
{
    btMatrix3x3 m = mdtFrameWorld(c, bodyindex).getBasis();
    fromBt(m.getColumn(0), p);
    fromBt(m.getColumn(1), o);
}

void MEAPI MdtConstraintBodyGetAxesRel(const MdtConstraintID c, const unsigned int bodyindex, MeVector3 p,
                                       MeVector3 o)
{
    btMatrix3x3 m = mdtRef(c, bodyindex).getBasis();
    fromBt(m.getColumn(0), p);
    fromBt(m.getColumn(1), o);
}

void MEAPI MdtConstraintGetWorldLinearVelocity(const MdtConstraintID c, MeVector3 lv)
{
    MeVector3Copy(lv, c->head.worldLinVel);
}

void MEAPI MdtConstraintGetWorldAngularVelocity(const MdtConstraintID c, MeVector3 av)
{
    MeVector3Copy(av, c->head.worldAngVel);
}

void MEAPI MdtConstraintSetWorldLinearVelocity(const MdtConstraintID c, const MeReal vx, const MeReal vy,
                                               const MeReal vz)
{
    MeVector3Set(c->head.worldLinVel, vx, vy, vz);
}

void MEAPI MdtConstraintSetWorldAngularVelocity(const MdtConstraintID c, const MeReal ax, const MeReal ay,
                                                const MeReal az)
{
    MeVector3Set(c->head.worldAngVel, ax, ay, az);
}

void MEAPI MdtConstraintBodySetPosition(const MdtConstraintID c, const unsigned int bodyindex, const MeReal x,
                                        const MeReal y, const MeReal z)
{
    setWorldPosition(c, bodyindex, btVector3(x, y, z));
}

void MEAPI MdtConstraintBodySetPositionRel(const MdtConstraintID c, const unsigned int bodyindex, const MeReal x,
                                           const MeReal y, const MeReal z)
{
    btTransform r = mdtRef(c, bodyindex);
    r.setOrigin(btVector3(x, y, z));
    mdtSetRef(c, bodyindex, r);
}

void MEAPI MdtConstraintSetPosition(const MdtConstraintID c, const MeReal x, const MeReal y, const MeReal z)
{
    setWorldPosition(c, 0, btVector3(x, y, z));
    setWorldPosition(c, 1, btVector3(x, y, z));
}

void MEAPI MdtConstraintSetAxis(const MdtConstraintID c, const MeReal px, const MeReal py, const MeReal pz)
{
    if (c->head.setAxisFunc)
    {
        c->head.setAxisFunc(c, px, py, pz);
        return;
    }
    btVector3 o = mdtOrthogonal(btVector3(px, py, pz));
    MdtConstraintSetAxes(c, px, py, pz, o.x(), o.y(), o.z());
}

void MEAPI MdtConstraintSetAxes(const MdtConstraintID c, const MeReal px, const MeReal py, const MeReal pz,
                                const MeReal ox, const MeReal oy, const MeReal oz)
{
    btMatrix3x3 m = basis(btVector3(px, py, pz), btVector3(ox, oy, oz));
    setWorldAxes(c, 0, m);
    setWorldAxes(c, 1, m);
}

void MEAPI MdtConstraintBodySetAxes(const MdtConstraintID c, const unsigned int bodyindex, const MeReal px,
                                    const MeReal py, const MeReal pz, const MeReal ox, const MeReal oy,
                                    const MeReal oz)
{
    setWorldAxes(c, bodyindex, basis(btVector3(px, py, pz), btVector3(ox, oy, oz)));
}

void MEAPI MdtConstraintBodySetAxesRel(const MdtConstraintID c, const unsigned int bodyindex, const MeReal px,
                                       const MeReal py, const MeReal pz, const MeReal ox, const MeReal oy,
                                       const MeReal oz)
{
    setRelativeAxes(c, bodyindex, basis(btVector3(px, py, pz), btVector3(ox, oy, oz)));
}

// Karma's world dictionary holds the enabled constraints.
MdtConstraintID MEAPI MdtConstraintGetFirst(const MdtWorldID w)
{
    return mdtDictGet<MdtConstraintID>(MeDictFirst(&w->constraintDict));
}

MdtConstraintID MEAPI MdtConstraintGetNext(const MdtConstraintID c)
{
    if (!isEnabled(c))
        return nullptr;
    return mdtDictGet<MdtConstraintID>(MeDictNext(&c->head.world->constraintDict, &c->head.worldNode));
}

} // extern "C"
