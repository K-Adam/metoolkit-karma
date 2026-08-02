#include <algorithm>

#include "MdtImpl.h"

namespace {

const MdtConstraintOps* gOps[MdtBclUSER + 1];

} // namespace

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

extern "C" {

void MEAPI MdtConstraintDestroy(const MdtConstraintID c)
{
    MdtWorldImpl* w = impl(c->head.world);
    w->enabled.erase(c);
    // Group contacts are owned by their group and are not in this list.
    auto it = std::find(w->constraints.begin(), w->constraints.end(), c);
    if (it != w->constraints.end())
        w->constraints.erase(it);
    mdtConstraintOps(c->head.tag)->destroy(c);
}

void MEAPI MdtConstraintEnable(const MdtConstraintID c)
{
    impl(c->head.world)->enabled.insert(c);
}

void MEAPI MdtConstraintDisable(const MdtConstraintID c)
{
    impl(c->head.world)->enabled.erase(c);
}

MeBool MEAPI MdtConstraintIsEnabled(const MdtConstraintID c)
{
    return impl(c->head.world)->enabled.count(c) != 0;
}

MdtBodyID MEAPI MdtConstraintGetBody(const MdtConstraintID c, const unsigned int bodyindex)
{
    return bodyindex < MdtKeaMAXBODYCONSTRAINT ? c->head.mdtbody[bodyindex] : nullptr;
}

void MEAPI MdtConstraintSetBodies(const MdtConstraintID c, const MdtBodyID b1, const MdtBodyID b2)
{
    c->head.mdtbody[0] = b1;
    c->head.mdtbody[1] = b2;
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

} // extern "C"
