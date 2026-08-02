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

btScalar friction(const MdtBclContactParams& p)
{
    if (p.type == MdtContactTypeFrictionZero)
        return 0;
    // Bullet only has Coulomb friction. Box friction (a force limit) has no equivalent yet.
    return p.frictioncoeff1;
}

} // namespace

const MdtConstraintOps mdtContactOps = {destroyContact, prepareContact};

// The contact normal points from body2 to body1, as the bridge copies it from McdContact. Bullet's
// normalWorldOnB has the same direction with A as body1, and its distance is the negative penetration.
void mdtContactPrepare(MdtWorldImpl& w, MdtContactID c)
{
    MdtBodyID b1 = c->head.mdtbody[0], b2 = c->head.mdtbody[1];
    if (!b1)
        return;
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
    pt.m_combinedFriction = friction(c->params);
    pt.m_combinedRestitution = c->params.options & MdtBclContactOptionBounce ? c->params.restitution : 0;

    btPersistentManifold* m = w.dispatcher.getNewManifold(rbA, rbB);
    // The bodies' shapes are empty, which would give a zero breaking threshold. Mcd already decided this
    // contact exists, including ones still separated within the contact tolerance.
    m->setContactBreakingThreshold(BT_LARGE_FLOAT);
    m->addManifoldPoint(pt);
    w.stepManifolds.push_back(m);
}

extern "C" {

MdtContactID MEAPI MdtContactCreate(const MdtWorldID w)
{
    auto* c = new MdtContact{};
    mdtRegisterConstraintOps(MdtBclCONTACT, &mdtContactOps);
    mdtConstraintInit(c->head, w, MdtBclCONTACT);
    MdtContactParamsReset(&c->params);
    return c;
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
    v[0] = c->cpos[0];
    v[1] = c->cpos[1];
    v[2] = c->cpos[2];
}

void MEAPI MdtContactSetPosition(const MdtContactID c, const MeReal x, const MeReal y, const MeReal z)
{
    c->cpos[0] = x;
    c->cpos[1] = y;
    c->cpos[2] = z;
}

void MEAPI MdtContactGetNormal(const MdtContactID c, MeVector3 v)
{
    v[0] = c->normal[0];
    v[1] = c->normal[1];
    v[2] = c->normal[2];
}

void MEAPI MdtContactSetNormal(const MdtContactID c, const MeReal x, const MeReal y, const MeReal z)
{
    c->normal[0] = x;
    c->normal[1] = y;
    c->normal[2] = z;
}

MeReal MEAPI MdtContactGetPenetration(const MdtContactID c)
{
    return c->penetration;
}

void MEAPI MdtContactSetPenetration(const MdtContactID c, const MeReal p)
{
    c->penetration = p;
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
