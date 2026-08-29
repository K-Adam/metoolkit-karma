#include <algorithm>

#include "MdtImpl.h"

namespace {

void destroyContact(MdtConstraintID c)
{
    auto* k = reinterpret_cast<MdtContactID>(c);
    if (k->contactGroup)
        MdtContactGroupRemoveContact(k->contactGroup, k);
    delete k;
}

void prepareContact(MdtWorldImpl& w, MdtConstraintID c)
{
    mdtContactPrepare(w, reinterpret_cast<MdtContactID>(c));
}

void set3(MeReal* v, MeReal x, MeReal y, MeReal z)
{
    v[0] = x;
    v[1] = y;
    v[2] = z;
}

// Kea's friction rows have a fixed force bound per step, which Bullet's Coulomb rows cannot express. Each
// row bounds the impulse by limit h and drives the relative velocity along dir towards velocity.
class ContactFriction : public btTypedConstraint
{
public:
    struct Row
    {
        btVector3 dir;
        btScalar limit, velocity;
    };

    ContactFriction(btRigidBody& a, btRigidBody& b, const btVector3& armA, const btVector3& armB)
        : btTypedConstraint(CONTACT_CONSTRAINT_TYPE, a, b), m_armA(armA), m_armB(armB)
    {
    }

    void addRow(const Row& r) { m_rows[m_count++] = r; }

    void getInfo1(btConstraintInfo1* info) override
    {
        info->m_numConstraintRows = m_count;
        info->nub = 0;
    }

    void getInfo2(btConstraintInfo2* info) override
    {
        btScalar h = 1 / info->fps;
        for (int i = 0; i < m_count; ++i)
        {
            const Row& r = m_rows[i];
            int s = i * info->rowskip;
            btVector3 angA = m_armA.cross(r.dir), angB = m_armB.cross(r.dir);
            for (int k = 0; k < 3; ++k)
            {
                info->m_J1linearAxis[s + k] = r.dir[k];
                info->m_J1angularAxis[s + k] = angA[k];
                info->m_J2linearAxis[s + k] = -r.dir[k];
                info->m_J2angularAxis[s + k] = -angB[k];
            }
            info->m_constraintError[s] = r.velocity;
            info->m_lowerLimit[s] = -r.limit * h;
            info->m_upperLimit[s] = r.limit * h;
        }
    }

    void setParam(int, btScalar, int) override {}
    btScalar getParam(int, int) const override { return 0; }

private:
    btVector3 m_armA, m_armB;
    Row m_rows[2];
    int m_count = 0;
};

// The normal force model scales the previous step's normal force, taken as this contact's share of its
// group's total.
btScalar frictionLimit(const MdtContact& c, MdtFrictionModel model, MeReal box, MeReal coeff)
{
    if (model == MdtFrictionModelBox)
        return box >= MEINFINITY ? BT_LARGE_FLOAT : box;
    MdtContactGroupID g = c.contactGroup;
    return g && g->count ? coeff * g->normalForce / g->count : 0;
}

btScalar surfaceVelocity(const MdtContact& c, const btVector3& dir, int slideOption, MeReal slide)
{
    btScalar v = c.params.options & slideOption ? slide : 0;
    if ((c.params.options & MdtBclContactOptionUseWorldVelocity) && !c.head.mdtbody[1])
        v += dir.dot(toBt(c.worldVel));
    return v;
}

void addFriction(MdtWorldImpl& w, const MdtContact& c, btRigidBody& a, btRigidBody& b, const btVector3& onA,
                 const btVector3& onB, const btVector3& normal)
{
    const MdtBclContactParams& p = c.params;
    if (p.type != MdtContactTypeFriction1D && p.type != MdtContactTypeFriction2D)
        return;
    btVector3 dir1, dir2;
    btPlaneSpace1(normal, dir1, dir2);
    btVector3 user = toBt(c.direction);
    user -= normal * normal.dot(user);
    if ((p.options & MdtBclContactOptionUseDirection) && user.length2() > SIMD_EPSILON)
        dir1 = user.normalized();
    dir2 = normal.cross(dir1);

    auto f = std::make_unique<ContactFriction>(a, b, onA - a.getCenterOfMassPosition(),
                                               onB - b.getCenterOfMassPosition());
    f->addRow({dir1, frictionLimit(c, p.model1, p.friction1, p.frictioncoeff1),
               surfaceVelocity(c, dir1, MdtBclContactOptionSlide1, p.slide1)});
    if (p.type == MdtContactTypeFriction2D)
        f->addRow({dir2, frictionLimit(c, p.model2, p.friction2, p.frictioncoeff2),
                   surfaceVelocity(c, dir2, MdtBclContactOptionSlide2, p.slide2)});
    w.world.addConstraint(f.get());
    w.stepConstraints.push_back(std::move(f));
}

void resetContact(MdtContactID c)
{
    set3(c->cpos, 0, 0, 0);
    set3(c->normal, 0, 1, 0);
    set3(c->direction, 1, 0, 0);
    set3(c->worldVel, 0, 0, 0);
    c->penetration = 0;
    MdtContactParamsReset(&c->params);
}

} // namespace

const MdtConstraintOps mdtContactOps = {destroyContact, prepareContact};

// The contact normal points from body2 to body1, as the bridge copies it from McdContact. Bullet's
// normalWorldOnB has the same direction with A as body1, and its distance is the negative penetration.
void mdtContactPrepare(MdtWorldImpl& w, MdtContactID c)
{
    MdtBodyID b1 = c->head.mdtbody[0], b2 = c->head.mdtbody[1];
    // After waking and partitioning, a two-body contact's bodies are enabled or disabled together.
    if (!b1 || !MdtBodyIsEnabled(b1))
        return;
    const MdtBclContactParams& p = c->params;
    MeReal h = w.pub.keaParams.stepsize;
    btRigidBody* rbA = mdtRigidBody(w, b1);
    btRigidBody* rbB = mdtRigidBody(w, b2);
    btVector3 normalOnB = toBt(c->normal);
    btScalar distance = -c->penetration;
    btVector3 onB = toBt(c->cpos) - normalOnB * (distance / 2);
    btVector3 onA = onB + normalOnB * distance;

    btManifoldPoint pt(rbA->getWorldTransform().invXform(onA), rbB->getWorldTransform().invXform(onB), normalOnB,
                       distance);
    pt.m_positionWorldOnA = onA;
    pt.m_positionWorldOnB = onB;
    pt.m_userPersistentData = c;

    btVector3 relVel = rbA->getVelocityInLocalPoint(onA - rbA->getCenterOfMassPosition()) -
                       rbB->getVelocityInLocalPoint(onB - rbB->getCenterOfMassPosition());
    bool bounce = (p.options & MdtBclContactOptionBounce) && btFabs(relVel.dot(normalOnB)) > p.velThreshold;
    pt.m_combinedRestitution = bounce ? p.restitution : 0;

    // Kea regularises the normal row by epsilon h plus the softness, both in force units. Bullet divides
    // the contact CFM by the step to get impulse units.
    pt.m_contactCFM = w.pub.keaParams.epsilon * h + (p.options & MdtBclContactOptionSoft ? p.softness : 0);
    pt.m_contactPointFlags |= BT_CONTACT_FLAG_HAS_CONTACT_CFM;
    // Friction comes from addFriction, so Bullet's own friction rows stay inactive.
    pt.m_combinedFriction = 0;
    addFriction(w, *c, *rbA, *rbB, onA, onB, normalOnB);

    btPersistentManifold* m = w.dispatcher.getNewManifold(rbA, rbB);
    // The bodies' shapes are empty, which would give a zero breaking threshold. Mcd already decided this
    // contact exists, including ones still separated within the contact tolerance.
    m->setContactBreakingThreshold(BT_LARGE_FLOAT);
    m->addManifoldPoint(pt);
    w.stepManifolds.push_back(m);
}

extern "C" {

const MdtContactID MdtContactInvalidID = nullptr;

MdtContactID MEAPI MdtContactCreate(const MdtWorldID w)
{
    auto* c = new MdtContact{};
    mdtRegisterConstraintOps(MdtBclCONTACT, &mdtContactOps);
    mdtConstraintInit(c->head, w, MdtBclCONTACT);
    resetContact(c);
    return c;
}

// Like the original, a reset disables the contact and keeps its bodies. Group membership is kept too, since
// clearing it would leave the group's list pointing at the contact.
void MEAPI MdtContactReset(const MdtContactID c)
{
    MdtConstraintDisable(MdtContactQuaConstraint(c));
    resetContact(c);
}

MdtConstraintID MEAPI MdtContactQuaConstraint(const MdtContactID c)
{
    return reinterpret_cast<MdtConstraintID>(c);
}

MdtContactID MEAPI MdtConstraintDCastContact(const MdtConstraintID c)
{
    return c && c->head.tag == MdtBclCONTACT ? reinterpret_cast<MdtContactID>(c) : nullptr;
}

void MEAPI MdtContactGetPosition(const MdtContactID c, MeVector3 v)
{
    std::copy_n(c->cpos, 3, v);
}

void MEAPI MdtContactSetPosition(const MdtContactID c, const MeReal x, const MeReal y, const MeReal z)
{
    set3(c->cpos, x, y, z);
}

void MEAPI MdtContactGetNormal(const MdtContactID c, MeVector3 v)
{
    std::copy_n(c->normal, 3, v);
}

void MEAPI MdtContactSetNormal(const MdtContactID c, const MeReal x, const MeReal y, const MeReal z)
{
    set3(c->normal, x, y, z);
}

MeReal MEAPI MdtContactGetPenetration(const MdtContactID c)
{
    return c->penetration;
}

void MEAPI MdtContactSetPenetration(const MdtContactID c, const MeReal p)
{
    c->penetration = p;
}

void MEAPI MdtContactGetDirection(const MdtContactID c, MeVector3 v)
{
    std::copy_n(c->direction, 3, v);
}

void MEAPI MdtContactSetDirection(const MdtContactID c, const MeReal x, const MeReal y, const MeReal z)
{
    set3(c->direction, x, y, z);
    c->params.options |= MdtBclContactOptionUseDirection;
}

void MEAPI MdtContactGetWorldVelocity(const MdtContactID c, MeVector3 v)
{
    std::copy_n(c->worldVel, 3, v);
}

void MEAPI MdtContactSetWorldVelocity(const MdtContactID c, const MeReal x, const MeReal y, const MeReal z)
{
    set3(c->worldVel, x, y, z);
    c->params.options |= MdtBclContactOptionUseWorldVelocity;
}

// Velocity of body1 relative to body2 at the contact position, in world coordinates.
void MEAPI MdtContactGetRelativeVelocity(const MdtContactID c, MeVector3 v)
{
    MeVector3 v1 = {0, 0, 0}, v2 = {0, 0, 0};
    if (MdtBodyID b1 = c->head.mdtbody[0])
        MdtBodyGetVelocityAtPoint(b1, c->cpos, v1);
    if (MdtBodyID b2 = c->head.mdtbody[1])
        MdtBodyGetVelocityAtPoint(b2, c->cpos, v2);
    set3(v, v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]);
}

MdtContactParamsID MEAPI MdtContactGetParams(const MdtContactID c)
{
    return &c->params;
}

void MEAPI MdtContactSetParams(const MdtContactID c, const MdtContactParamsID p)
{
    c->params = *p;
}

MdtContactID MEAPI MdtContactGetNext(const MdtContactID c)
{
    return c->nextContact;
}

void MEAPI MdtContactSetNext(const MdtContactID c, const MdtContactID nc)
{
    c->nextContact = nc;
}

MdtContactGroupID MEAPI MdtContactGetContactGroup(const MdtContactID c)
{
    return c->contactGroup;
}

} // extern "C"
