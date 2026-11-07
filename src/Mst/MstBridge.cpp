#include <algorithm>

#include <Mst.h>

#include "Unsupported.h"

namespace {

const unsigned int kDefaultPairBuffer = 256;
const unsigned int kDefaultContactBuffer = 1024;

// Material pair parameters are symmetric, so both orders share one entry.
MstMaterialPair* materialPair(MstBridgeID b, MstMaterialID m1, MstMaterialID m2)
{
    if (m1 >= b->maxMaterials || m2 >= b->maxMaterials)
    {
        MeWarning(0, "MstBridge: material pair (%u, %u) out of range", m1, m2);
        return nullptr;
    }
    return &b->materialPairArray[std::min(m1, m2) * b->maxMaterials + std::max(m1, m2)];
}

MdtContactGroupID groupFor(McdModelPair* p, MdtWorldID w, MdtBodyID b1, MdtBodyID b2)
{
    auto* g = static_cast<MdtContactGroupID>(p->responseData);
    if (g)
        return g;
    g = MdtContactGroupCreate(w);
    // Dynamics wants the first body set, so a pair whose first model is static is swapped.
    g->swapped = b1 == nullptr;
    MdtConstraintSetBodies(MdtContactGroupQuaConstraint(g), g->swapped ? b2 : b1, g->swapped ? b1 : b2);
    MdtContactGroupSetGenerator(g, p);
    MdtConstraintEnable(MdtContactGroupQuaConstraint(g));
    p->responseData = g;
    return g;
}

void fillContacts(McdIntersectResult& r, MdtContactGroupID g, MstMaterialPair& mp)
{
    MeReal sign = g->swapped ? -1 : 1;
    for (int i = 0; i < r.contactCount; ++i)
    {
        McdContact& c = r.contacts[i];
        MdtContactID dc = MdtContactGroupCreateContact(g);
        MdtContactSetPosition(dc, c.position[0], c.position[1], c.position[2]);
        MdtContactSetNormal(dc, sign * c.normal[0], sign * c.normal[1], sign * c.normal[2]);
        MdtContactSetPenetration(dc, -c.separation);
        MdtContactSetParams(dc, &mp.cp);
        if (mp.contactCB && !mp.contactCB(&r, &c, dc))
            MdtContactGroupDestroyContact(g, dc);
    }
    if (mp.pairCB && !mp.pairCB(&r, g))
        MdtContactGroupReset(g);
}

} // namespace

extern "C" {

MstBridgeID MEAPI MstBridgeCreate(McdFrameworkID frame, const unsigned int maxMaterials)
{
    (void)frame;
    auto* b = new MstBridge{};
    // The material pair table is square, so this keeps its size within 32 bits.
    b->maxMaterials = std::clamp(maxMaterials, 1u, 65535u);
    b->usedMaterials = 1;
    b->materialPairArray = new MstMaterialPair[b->maxMaterials * b->maxMaterials]{};
    for (unsigned int i = 0; i < b->maxMaterials * b->maxMaterials; ++i)
        MdtContactParamsReset(&b->materialPairArray[i].cp);
    b->pairs = McdModelPairContainerCreate(kDefaultPairBuffer);
    b->contactsMaxCount = kDefaultContactBuffer;
    b->contacts = new McdContact[b->contactsMaxCount];
    return b;
}

void MEAPI MstBridgeDestroy(const MstBridgeID b)
{
    McdModelPairContainerDestroy(b->pairs);
    delete[] b->contacts;
    delete[] b->materialPairArray;
    delete b;
}

void MEAPI MstBridgeSetModelPairBufferSize(const MstBridgeID b, const unsigned int s)
{
    McdModelPairContainerDestroy(b->pairs);
    b->pairs = McdModelPairContainerCreate((int)s);
}

void MEAPI MstBridgeSetContactBufferSize(const MstBridgeID b, const unsigned int s)
{
    delete[] b->contacts;
    b->contactsMaxCount = s;
    b->contacts = new McdContact[s];
}

MstMaterialID MEAPI MstBridgeGetNewMaterial(const MstBridgeID b)
{
    if (b->usedMaterials >= b->maxMaterials)
    {
        MeWarning(0, "MstBridgeGetNewMaterial: all %u materials are in use", b->maxMaterials);
        return 0;
    }
    return b->usedMaterials++;
}

MdtContactParamsID MEAPI MstBridgeGetContactParams(const MstBridgeID b, const MstMaterialID m1, const MstMaterialID m2)
{
    MstMaterialPair* mp = materialPair(b, m1, m2);
    return mp ? &mp->cp : nullptr;
}

MstPerPairCBPtr MEAPI MstBridgeGetPerPairCB(const MstBridgeID b, const MstMaterialID m1, const MstMaterialID m2)
{
    MstMaterialPair* mp = materialPair(b, m1, m2);
    return mp ? mp->pairCB : nullptr;
}

MstPerContactCBPtr MEAPI MstBridgeGetPerContactCB(const MstBridgeID b, const MstMaterialID m1, const MstMaterialID m2)
{
    MstMaterialPair* mp = materialPair(b, m1, m2);
    return mp ? mp->contactCB : nullptr;
}

MstIntersectCBPtr MEAPI MstBridgeGetIntersectCB(const MstBridgeID b, const MstMaterialID m1, const MstMaterialID m2)
{
    MstMaterialPair* mp = materialPair(b, m1, m2);
    return mp ? mp->intersectCB : nullptr;
}

void MEAPI MstBridgeSetPerPairCB(const MstBridgeID b, const MstMaterialID m1, const MstMaterialID m2,
                                 const MstPerPairCBPtr cb)
{
    if (MstMaterialPair* mp = materialPair(b, m1, m2))
        mp->pairCB = cb;
}

void MEAPI MstBridgeSetPerContactCB(const MstBridgeID b, const MstMaterialID m1, const MstMaterialID m2,
                                    const MstPerContactCBPtr cb)
{
    if (MstMaterialPair* mp = materialPair(b, m1, m2))
        mp->contactCB = cb;
}

void MEAPI MstBridgeSetIntersectCB(const MstBridgeID b, const MstMaterialID m1, const MstMaterialID m2,
                                   const MstIntersectCBPtr cb)
{
    if (MstMaterialPair* mp = materialPair(b, m1, m2))
        mp->intersectCB = cb;
}

void MEAPI MstHandleTransitions(McdModelPairContainer* pairs, const McdSpaceID s, const MdtWorldID w,
                                const MstBridgeID b)
{
    (void)s;
    (void)w;
    (void)b;
    for (int i = pairs->goodbyeFirst; i < pairs->goodbyeEnd; ++i)
    {
        McdModelPair* p = pairs->array[i];
        if (auto* g = static_cast<MdtContactGroupID>(p->responseData))
            MdtContactGroupDestroy(g);
        p->responseData = nullptr;
        McdGoodbye(p);
    }
    McdHelloEach(pairs);
}

void MEAPI MstHandleCollisions(McdModelPairContainer* pairs, const McdSpaceID s, const MdtWorldID w,
                               const MstBridgeID b)
{
    (void)s;
    for (int i = pairs->helloFirst; i < pairs->stayingEnd; ++i)
    {
        McdModelPair* p = pairs->array[i];
        MdtBodyID b1 = McdModelGetBody(p->model1), b2 = McdModelGetBody(p->model2);
        bool awake1 = b1 && MdtBodyIsEnabled(b1), awake2 = b2 && MdtBodyIsEnabled(b2);
        if (!awake1 && !awake2)
            continue;
        MstMaterialPair* mp = materialPair(b, (MstMaterialID)McdModelGetMaterial(p->model1),
                                           (MstMaterialID)McdModelGetMaterial(p->model2));
        if (!mp)
            continue;

        McdIntersectResult r{};
        r.contacts = b->contacts;
        r.contactMaxCount = (int)b->contactsMaxCount;
        McdIntersect(p, &r);
        if (mp->intersectCB)
            mp->intersectCB(&r);

        if (auto* g = static_cast<MdtContactGroupID>(p->responseData))
            MdtContactGroupReset(g);
        if (r.touch)
            fillContacts(r, groupFor(p, w, b1, b2), *mp);
    }
}

void MEAPI MstBridgeUpdateTransitions(const MstBridgeID b, const McdSpaceID s, const MdtWorldID w)
{
    McdSpacePairIterator it;
    McdSpacePairIteratorBegin(s, &it);
    int more;
    do
    {
        more = McdSpaceGetTransitions(s, &it, b->pairs);
        MstHandleTransitions(b->pairs, s, w, b);
    } while (more);
}

void MEAPI MstBridgeUpdateContacts(const MstBridgeID b, const McdSpaceID s, const MdtWorldID w)
{
    McdSpacePairIterator it;
    McdSpacePairIteratorBegin(s, &it);
    int more;
    do
    {
        more = McdSpaceGetPairs(s, &it, b->pairs);
        MstHandleTransitions(b->pairs, s, w, b);
        MstHandleCollisions(b->pairs, s, w, b);
    } while (more);
}

// Declared but absent from the original library, so no caller depends on it. The bridge does not know its space
// and world.
void MEAPI MstBridgeStepAll(const MstBridgeID, const MeReal)
{
    ME_UNSUPPORTED("MstBridgeStepAll");
}

} // extern "C"
