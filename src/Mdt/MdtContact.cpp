#include <algorithm>

#include <MeMath.h>

#include "MdtImpl.h"

namespace {

void destroyContact(MdtConstraintID c)
{
    auto* k = reinterpret_cast<MdtContactID>(c);
    if (k->contactGroup)
        MdtContactGroupRemoveContact(k->contactGroup, k);
    delete k;
}

// A grouped contact is prepared by its group.
void prepareContact(MdtWorldImpl& w, MdtConstraintID c)
{
    auto* k = reinterpret_cast<MdtContactID>(c);
    if (!k->contactGroup)
        mdtContactPrepare(w, k);
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

    ContactFriction(MdtContact& c, btRigidBody& a, btRigidBody& b, const btVector3& armA, const btVector3& armB)
        : btTypedConstraint(CONTACT_CONSTRAINT_TYPE, a, b), contact(c), m_armA(armA), m_armB(armB)
    {
        setJointFeedback(&feedback);
    }

    MdtContact& contact;
    btJointFeedback feedback{};

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

// As observed in the original, the normal force model gives each contact an equal share of its group's
// previous normal force, and a contact outside a group falls back to box friction.
btScalar frictionLimit(const MdtContact& c, MdtFrictionModel model, MeReal box, MeReal coeff)
{
    MdtContactGroupID g = c.contactGroup;
    if (model == MdtFrictionModelNormalForce && g && g->count)
        return coeff * g->normalForce / g->count;
    return box >= MEINFINITY ? BT_LARGE_FLOAT : box;
}

btScalar surfaceVelocity(const MdtContact& c, const btVector3& dir, int slideOption, MeReal slide)
{
    btScalar v = c.params.options & slideOption ? slide : 0;
    if ((c.params.options & MdtBclContactOptionUseWorldVelocity) && !c.head.mdtbody[1])
        v += dir.dot(toBt(c.worldVel));
    return v;
}

void addFriction(MdtWorldImpl& w, MdtContact& c, btRigidBody& a, btRigidBody& b, const btVector3& onA,
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

    auto f = std::make_unique<ContactFriction>(c, a, b, onA - a.getCenterOfMassPosition(),
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
    MeVector3Set(c->cpos, 0, 0, 0);
    MeVector3Set(c->normal, 0, 1, 0);
    MeVector3Set(c->direction, 1, 0, 0);
    MeVector3Set(c->worldVel, 0, 0, 0);
    c->penetration = 0;
    MdtContactParamsReset(&c->params);
}

void addResult(MeVector4 out, const btVector3& v)
{
    fromBt(toBt(out) + v, out);
}

// Adds to the contact and to its group what this step applied to body1 and body2.
void addResults(MdtContact& c, const btVector3& fA, const btVector3& tA, const btVector3& fB, const btVector3& tB)
{
    auto add = [&](MdtConstraintHeader& h) {
        addResult(h.resultForce[0], fA);
        addResult(h.resultTorque[0], tA);
        // The world is not a body, so it reports nothing.
        if (c.head.mdtbody[1])
        {
            addResult(h.resultForce[1], fB);
            addResult(h.resultTorque[1], tB);
        }
    };
    add(c.head);
    if (c.contactGroup)
        add(c.contactGroup->head);
}

const MdtConstraintOps kContactOps = {destroyContact, prepareContact};

} // namespace

void mdtContactCollectForces(MdtWorldImpl& w, MeReal dt)
{
    for (btPersistentManifold* m : w.stepManifolds)
        for (int i = 0; i < m->getNumContacts(); ++i)
        {
            const btManifoldPoint& pt = m->getContactPoint(i);
            auto* c = static_cast<MdtContactID>(pt.m_userPersistentData);
            btScalar n = pt.m_appliedImpulse / dt;
            btVector3 f = pt.m_normalWorldOnB * n;
            // pullBody has not run yet, so comTM is still the centre of mass the solve used.
            MdtBodyID b2 = c->head.mdtbody[1];
            btVector3 armA = pt.m_positionWorldOnA - toBt(c->head.mdtbody[0]->comTM[3]);
            btVector3 armB = b2 ? pt.m_positionWorldOnB - toBt(b2->comTM[3]) : btVector3(0, 0, 0);
            addResults(*c, f, armA.cross(f), -f, armB.cross(-f));
            if (c->contactGroup)
                c->contactGroup->normalForce += n;
        }
    // Bullet's feedback holds forces and torques about each body's centre of mass.
    for (auto& t : w.stepConstraints)
    {
        auto& f = static_cast<ContactFriction&>(*t);
        const btJointFeedback& fb = f.feedback;
        addResults(f.contact, fb.m_appliedForceBodyA, fb.m_appliedTorqueBodyA, fb.m_appliedForceBodyB,
                   fb.m_appliedTorqueBodyB);
    }
}

// The contact normal points from body2 to body1, as the bridge copies it from McdContact. Bullet's
// normalWorldOnB has the same direction with A as body1, and its distance is the negative penetration.
void mdtContactPrepare(MdtWorldImpl& w, MdtContactID c)
{
    mdtClearResults(c->head);
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
    // Grouped contacts stay in the world's list too, as the original counts them.
    auto* c = new MdtContact{};
    mdtRegisterConstraintOps(MdtBclCONTACT, &kContactOps);
    mdtConstraintInit(c->head, w, MdtBclCONTACT);
    c->head.maxRows = MdtBclGETMAXROWSCONTACT;
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
    MeVector3Set(c->cpos, x, y, z);
}

void MEAPI MdtContactGetNormal(const MdtContactID c, MeVector3 v)
{
    std::copy_n(c->normal, 3, v);
}

void MEAPI MdtContactSetNormal(const MdtContactID c, const MeReal x, const MeReal y, const MeReal z)
{
    MeVector3Set(c->normal, x, y, z);
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
    MeVector3Set(c->direction, x, y, z);
    c->params.options |= MdtBclContactOptionUseDirection;
}

void MEAPI MdtContactGetWorldVelocity(const MdtContactID c, MeVector3 v)
{
    std::copy_n(c->worldVel, 3, v);
}

void MEAPI MdtContactSetWorldVelocity(const MdtContactID c, const MeReal x, const MeReal y, const MeReal z)
{
    MeVector3Set(c->worldVel, x, y, z);
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
    MeVector3Subtract(v, v1, v2);
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
