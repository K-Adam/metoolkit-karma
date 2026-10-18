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

// Marks the Bullet contact points that carry a slip row rather than the contact's normal row.
const int kSlipPoint = 1;

// Kea's friction and adhesion rows, which Bullet's contact rows cannot express. A friction row bounds its
// impulse by the friction force times h. The adhesive row pulls along the normal up to the adhesive force,
// beside Bullet's pushing row. Each row's cfm is its CFM in impulse units: Kea regularises every row by
// epsilon h in force units, and a slip row adds its slip.
class ContactRows : public btTypedConstraint
{
public:
    ContactRows(MdtWorldImpl& w, MdtContact& c, btRigidBody& a, btRigidBody& b, const btVector3& onA,
                const btVector3& onB, const btVector3& normal)
        : btTypedConstraint(CONTACT_CONSTRAINT_TYPE, a, b), contact(c), normal(normal), m_onA(onA), m_onB(onB)
    {
        m_r.a = &a;
        m_r.b = &b;
        m_r.h = w.pub.keaParams.stepsize;
        m_r.epsilon = w.pub.keaParams.epsilon;
        setJointFeedback(&feedback);
    }

    MdtContact& contact;
    btVector3 normal;
    btJointFeedback feedback{};

    // Adds a row along dir with force bounds lo and hi, driving the relative velocity to velocity, softened
    // by a slip or softness in force units.
    void addRow(const btVector3& dir, btScalar lo, btScalar hi, btScalar soft, btScalar velocity)
    {
        MdtRow& r = m_r.rows[m_r.n++] = m_r.linear(dir, m_onA, m_onB);
        r.lo = lo * m_r.h;
        r.hi = hi * m_r.h;
        r.rhs = velocity;
        r.cfm = m_r.epsilon + soft / m_r.h;
    }

    bool empty() const { return m_r.n == 0; }

    void getInfo1(btConstraintInfo1* info) override
    {
        info->m_numConstraintRows = m_r.n;
        info->nub = 0;
    }

    // Bullet's joint iteration only converges for a cfm below the row's effective mass, so a row gets at
    // most half of that.
    // ponytail: a bounded slip row softer than that is solved too stiff, needs its own solve if that shows.
    void getInfo2(btConstraintInfo2* info) override
    {
        for (int i = 0; i < m_r.n; ++i)
        {
            const MdtRow& row = m_r.rows[i];
            int s = i * info->rowskip;
            for (int k = 0; k < 3; ++k)
            {
                info->m_J1linearAxis[s + k] = row.linA[k];
                info->m_J1angularAxis[s + k] = row.angA[k];
                info->m_J2linearAxis[s + k] = row.linB[k];
                info->m_J2angularAxis[s + k] = row.angB[k];
            }
            info->m_constraintError[s] = row.rhs;
            info->cfm[s] = btMin(row.cfm / m_r.effectiveMass(row), btScalar(0.5));
            info->m_lowerLimit[s] = row.lo;
            info->m_upperLimit[s] = row.hi;
        }
    }

    void setParam(int, btScalar, int) override {}
    btScalar getParam(int, int) const override { return 0; }

private:
    btVector3 m_onA, m_onB;
    MdtJointRows m_r;
};

// Bullet's contact rows, unlike its joint rows, converge for any CFM. So a slip row with unbounded force
// becomes two contact points, one pushing along dir and one against it, which together drive the relative
// velocity along dir to the surface velocity, softened by cfm in force units. A contact point with an ERP
// of 1 drives its normal velocity to minus its distance over the step.
void addSlipPoints(MdtWorldImpl& w, MdtContact& c, btRigidBody& a, btRigidBody& b, const btVector3& onA,
                   const btVector3& onB, const btVector3& dir, btScalar surface, btScalar cfm)
{
    btPersistentManifold* m = w.dispatcher.getNewManifold(&a, &b);
    m->setContactBreakingThreshold(BT_LARGE_FLOAT);
    for (int side = -1; side <= 1; side += 2)
    {
        btManifoldPoint pt(a.getWorldTransform().invXform(onA), b.getWorldTransform().invXform(onB), dir * side,
                           -side * surface * w.pub.keaParams.stepsize);
        pt.m_positionWorldOnA = onA;
        pt.m_positionWorldOnB = onB;
        pt.m_userPersistentData = &c;
        pt.m_index0 = kSlipPoint;
        pt.m_contactCFM = cfm;
        pt.m_contactERP = 1;
        pt.m_contactPointFlags |= BT_CONTACT_FLAG_HAS_CONTACT_CFM | BT_CONTACT_FLAG_HAS_CONTACT_ERP;
        pt.m_combinedFriction = 0;
        pt.m_combinedRestitution = 0;
        m->addManifoldPoint(pt);
    }
    w.stepManifolds.push_back(m);
}

// As observed in the original, the normal force model gives each contact an equal share of its group's
// previous normal force, and a contact outside a group falls back to box friction.
btScalar frictionLimit(const MdtContact& c, MdtFrictionModel model, MeReal box, MeReal coeff)
{
    MdtContactGroupID g = c.contactGroup;
    if (model == MdtFrictionModelNormalForce && g && g->count)
        return coeff * g->normalForce / g->count;
    return box >= MEINFINITY ? BT_LARGE_FLOAT : box;
}

// Velocity of body2's surface relative to body2, which drags body1 along through friction.
btVector3 surfaceVelocity(const MdtContact& c, const btVector3& normal, const btVector3& dir1, const btVector3& dir2)
{
    const MdtBclContactParams& p = c.params;
    btVector3 v(0, 0, 0);
    if ((p.options & MdtBclContactOptionUseWorldVelocity) && !c.head.mdtbody[1])
        v = toBt(c.worldVel) - normal * normal.dot(toBt(c.worldVel));
    if (p.options & MdtBclContactOptionSlide1)
        v += dir1 * p.slide1;
    if (p.options & MdtBclContactOptionSlide2)
        v += dir2 * p.slide2;
    return v;
}

btScalar slipOf(const MdtBclContactParams& p, int option, MeReal value)
{
    return p.options & option ? value : 0;
}

// Without a user direction, Kea turns the friction box of a sliding contact so the sliding velocity lies
// along its diagonal, scaling the box so the friction along the diagonal keeps its set size, and lays a 1D
// row along the sliding velocity. A contact that is not sliding takes the axis Karma picks orthogonal to
// the normal, and has no 1D row. Returns whether the contact has friction rows.
bool frictionAxis(const MdtContact& c, const btVector3& normal, const btVector3& relVel, btVector3& dir,
                  btScalar& scale)
{
    const MdtBclContactParams& p = c.params;
    bool twoD = p.type == MdtContactTypeFriction2D;
    btVector3 user = toBt(c.direction);
    user -= normal * normal.dot(user);
    btVector3 slide = relVel - normal * normal.dot(relVel);
    dir = mdtOrthogonal(normal);
    scale = 1;
    if ((p.options & MdtBclContactOptionUseDirection) && user.length2() > SIMD_EPSILON)
        dir = user.normalized();
    else if (slide.fuzzyZero())
        return twoD;
    else if (twoD)
    {
        slide.normalize();
        dir = (slide - normal.cross(slide)) * SIMDSQRT12;
        scale = SIMDSQRT12;
    }
    else
        dir = slide.normalized();
    return twoD || p.type == MdtContactTypeFriction1D;
}

void addRows(MdtWorldImpl& w, MdtContact& c, btRigidBody& a, btRigidBody& b, const btVector3& onA,
             const btVector3& onB, const btVector3& normal, const btVector3& relVel, btScalar adhesionVelocity)
{
    const MdtBclContactParams& p = c.params;
    btVector3 dir1;
    btScalar scale;
    bool friction = frictionAxis(c, normal, relVel, dir1, scale);
    bool adhesive = (p.options & MdtBclContactOptionAdhesive) && p.max_adhesive_force > 0;
    if (!friction && !adhesive)
        return;
    btVector3 dir2 = normal.cross(dir1);

    btVector3 surface = surfaceVelocity(c, normal, dir1, dir2);
    auto rows = std::make_unique<ContactRows>(w, c, a, b, onA, onB, normal);
    auto addFriction = [&](const btVector3& dir, MdtFrictionModel model, MeReal box, MeReal coeff, btScalar slip) {
        btScalar limit = frictionLimit(c, model, box, coeff);
        if (slip > 0 && limit >= BT_LARGE_FLOAT)
            addSlipPoints(w, c, a, b, onA, onB, dir, dir.dot(surface),
                          w.pub.keaParams.epsilon * w.pub.keaParams.stepsize + slip);
        else
            rows->addRow(dir, -scale * limit, scale * limit, slip, dir.dot(surface));
    };
    if (friction)
        addFriction(dir1, p.model1, p.friction1, p.frictioncoeff1, slipOf(p, MdtBclContactOptionSlip1, p.slip1));
    if (friction && p.type == MdtContactTypeFriction2D)
        addFriction(dir2, p.model2, p.friction2, p.frictioncoeff2, slipOf(p, MdtBclContactOptionSlip2, p.slip2));
    if (adhesive)
    {
        btScalar max = p.max_adhesive_force >= MEINFINITY ? BT_LARGE_FLOAT : p.max_adhesive_force;
        rows->addRow(normal, -max, 0, p.options & MdtBclContactOptionSoft ? p.softness : 0, adhesionVelocity);
    }
    if (rows->empty())
        return;
    w.world.addConstraint(rows.get());
    w.stepConstraints.push_back(std::move(rows));
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
            // mdtBodyPull has not run yet, so comTM is still the centre of mass the solve used.
            MdtBodyID b2 = c->head.mdtbody[1];
            btVector3 armA = pt.m_positionWorldOnA - toBt(c->head.mdtbody[0]->comTM[3]);
            btVector3 armB = b2 ? pt.m_positionWorldOnB - toBt(b2->comTM[3]) : btVector3(0, 0, 0);
            addResults(*c, f, armA.cross(f), -f, armB.cross(-f));
            if (c->contactGroup && pt.m_index0 != kSlipPoint)
                c->contactGroup->normalForce += n;
        }
    // Bullet's feedback holds forces and torques about each body's centre of mass. Friction rows are
    // tangential, so the normal part is the adhesive force.
    for (auto& t : w.stepConstraints)
    {
        auto& r = static_cast<ContactRows&>(*t);
        const btJointFeedback& fb = r.feedback;
        addResults(r.contact, fb.m_appliedForceBodyA, fb.m_appliedTorqueBodyA, fb.m_appliedForceBodyB,
                   fb.m_appliedTorqueBodyB);
        if (r.contact.contactGroup)
            r.contact.contactGroup->normalForce += fb.m_appliedForceBodyA.dot(r.normal);
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

    // Kea drives a separated contact closed at gamma times its gap per step, where Bullet would let it
    // close the whole gap.
    MeReal gamma = w.pub.keaParams.gamma;
    btManifoldPoint pt(rbA->getWorldTransform().invXform(onA), rbB->getWorldTransform().invXform(onB), normalOnB,
                       distance > 0 ? gamma * distance : distance);
    pt.m_positionWorldOnA = onA;
    pt.m_positionWorldOnB = onB;
    pt.m_userPersistentData = c;

    btVector3 relVel = rbA->getVelocityInLocalPoint(onA - rbA->getCenterOfMassPosition()) -
                       rbB->getVelocityInLocalPoint(onB - rbB->getCenterOfMassPosition());
    btScalar normalVel = relVel.dot(normalOnB);
    bool bounce = (p.options & MdtBclContactOptionBounce) && btFabs(normalVel) > p.velThreshold;
    pt.m_combinedRestitution = bounce ? p.restitution : 0;

    // Kea regularises the normal row by epsilon h plus the softness, both in force units. Bullet divides
    // the contact CFM by the step to get impulse units.
    pt.m_contactCFM = w.pub.keaParams.epsilon * h + (p.options & MdtBclContactOptionSoft ? p.softness : 0);
    pt.m_contactPointFlags |= BT_CONTACT_FLAG_HAS_CONTACT_CFM;
    // Friction comes from addRows, so Bullet's own friction rows stay inactive.
    pt.m_combinedFriction = 0;
    // The adhesive row targets the normal velocity Bullet's row does, so the two never fight.
    btScalar bounceVel = bounce ? btMax(btScalar(0), -p.restitution * normalVel) : 0;
    addRows(w, *c, *rbA, *rbB, onA, onB, normalOnB, relVel, bounceVel + gamma * c->penetration / h);

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
