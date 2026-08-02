#include "MdtImpl.h"

namespace {

void deleteContact(MdtContactID k)
{
    impl(k->head.world)->enabled.erase(reinterpret_cast<MdtConstraintID>(k));
    delete k;
}

void destroyGroup(MdtConstraintID c)
{
    auto* g = reinterpret_cast<MdtContactGroupID>(c);
    MdtWorldID w = g->head.world;
    if (w->contactGroupDestroyCallback)
        w->contactGroupDestroyCallback(g);
    for (MdtContactID k = g->first; k;)
    {
        MdtContactID next = k->nextContact;
        deleteContact(k);
        k = next;
    }
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

// Group contacts are owned by the group and never enter the world's constraint list.
MdtContactID MEAPI MdtContactGroupCreateContact(MdtContactGroupID g)
{
    auto* c = new MdtContact{};
    // MdtContactDestroy reaches group contacts through the generic constraint path.
    mdtRegisterConstraintOps(MdtBclCONTACT, &mdtContactOps);
    c->head.world = g->head.world;
    c->head.tag = MdtBclCONTACT;
    MdtContactParamsReset(&c->params);
    MdtContactGroupAppendContact(g, c);
    return c;
}

void MEAPI MdtContactGroupAppendContact(MdtContactGroupID g, MdtContactID c)
{
    c->contactGroup = g;
    c->nextContact = nullptr;
    c->prevContact = g->last;
    if (g->last)
        g->last->nextContact = c;
    else
        g->first = c;
    g->last = c;
    ++g->count;
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
}

void MEAPI MdtContactGroupDestroyContact(MdtContactGroupID g, MdtContactID c)
{
    MdtContactGroupRemoveContact(g, c);
    deleteContact(c);
}

MeBool MEAPI MdtContactGroupIsSwapped(MdtContactGroupID g)
{
    return g->swapped;
}

int MEAPI MdtContactGroupGetCount(MdtContactGroupID g)
{
    return g->count;
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
