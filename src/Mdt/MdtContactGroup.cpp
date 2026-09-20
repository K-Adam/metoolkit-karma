#include "MdtImpl.h"

namespace {

void destroyGroup(MdtConstraintID c)
{
    auto* g = reinterpret_cast<MdtContactGroupID>(c);
    MdtWorldID w = g->head.world;
    if (w->contactGroupDestroyCallback)
        w->contactGroupDestroyCallback(g);
    while (g->first)
        MdtContactGroupDestroyContact(g, g->first);
    delete g;
}

void prepareGroup(MdtWorldImpl& w, MdtConstraintID c)
{
    auto* g = reinterpret_cast<MdtContactGroupID>(c);
    for (MdtContactID k = g->first; k; k = k->nextContact)
    {
        k->head.mdtbody[0] = g->head.mdtbody[0];
        k->head.mdtbody[1] = g->head.mdtbody[1];
        mdtContactPrepare(w, k);
    }
    // Friction reads last step's values while preparing, then MdtWorldStep accumulates this step's.
    g->normalForce = 0;
    mdtClearResults(g->head);
}

const MdtConstraintOps kGroupOps = {destroyGroup, prepareGroup};

} // namespace

extern "C" {

MdtContactGroupID MEAPI MdtContactGroupCreate(const MdtWorldID w)
{
    auto* g = new MdtContactGroup{};
    mdtRegisterConstraintOps(MdtBclCONTACTGROUP, &kGroupOps);
    mdtConstraintInit(g->head, w, MdtBclCONTACTGROUP);
    return g;
}

void MEAPI MdtContactGroupDestroy(MdtContactGroupID g)
{
    MdtConstraintDestroy(MdtContactGroupQuaConstraint(g));
}

void MEAPI MdtContactGroupReset(const MdtContactGroupID g)
{
    while (g->first)
        MdtContactGroupDestroyContact(g, g->first);
}

MdtConstraintID MEAPI MdtContactGroupQuaConstraint(const MdtContactGroupID g)
{
    return reinterpret_cast<MdtConstraintID>(g);
}

MdtContactGroupID MEAPI MdtConstraintDCastContactGroup(const MdtConstraintID c)
{
    return c && c->head.tag == MdtBclCONTACTGROUP ? reinterpret_cast<MdtContactGroupID>(c) : nullptr;
}

MdtContactID MEAPI MdtContactGroupGetFirstContact(MdtContactGroupID g)
{
    return g->first;
}

MdtContactID MEAPI MdtContactGroupGetNextContact(MdtContactGroupID, MdtContactID c)
{
    return c->nextContact;
}

MdtContactID MEAPI MdtContactGroupCreateContact(MdtContactGroupID g)
{
    MdtContactID c = MdtContactCreate(g->head.world);
    c->head.mdtbody[0] = g->head.mdtbody[0];
    c->head.mdtbody[1] = g->head.mdtbody[1];
    MdtContactGroupAppendContact(g, c);
    return c;
}

// The group owns its contacts and destroys them with itself, as the original does.
void MEAPI MdtContactGroupAppendContact(MdtContactGroupID g, MdtContactID c)
{
    if (c->contactGroup)
        MdtContactGroupRemoveContact(c->contactGroup, c);
    // The group enables its contacts, so an enabled contact must not also join partitions on its own.
    MdtConstraintDisable(MdtContactQuaConstraint(c));
    c->contactGroup = g;
    c->nextContact = nullptr;
    c->prevContact = g->last;
    if (g->last)
        g->last->nextContact = c;
    else
        g->first = c;
    g->last = c;
    ++g->count;
    g->head.maxRows += MdtBclGETMAXROWSCONTACT;
}

void MEAPI MdtContactGroupRemoveContact(MdtContactGroupID g, MdtContactID c)
{
    if (c->prevContact)
        c->prevContact->nextContact = c->nextContact;
    else
        g->first = c->nextContact;
    if (c->nextContact)
        c->nextContact->prevContact = c->prevContact;
    else
        g->last = c->prevContact;
    c->nextContact = c->prevContact = nullptr;
    c->contactGroup = nullptr;
    --g->count;
    g->head.maxRows -= MdtBclGETMAXROWSCONTACT;
}

void MEAPI MdtContactGroupDestroyContact(MdtContactGroupID g, MdtContactID c)
{
    MdtContactGroupRemoveContact(g, c);
    MdtConstraintDestroy(MdtContactQuaConstraint(c));
}

MeBool MEAPI MdtContactGroupIsSwapped(MdtContactGroupID g)
{
    return g->swapped;
}

int MEAPI MdtContactGroupGetCount(MdtContactGroupID g)
{
    return g->count;
}

MeReal MEAPI MdtContactGroupGetNormalForce(MdtContactGroupID g)
{
    return g->normalForce;
}

void MEAPI MdtContactGroupSetGenerator(MdtContactGroupID g, void* generator)
{
    g->generator = generator;
}

void* MEAPI MdtContactGroupGetGenerator(MdtContactGroupID g)
{
    return g->generator;
}

} // extern "C"
