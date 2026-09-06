#include <cstring>

#include "McdImpl.h"

extern "C" {

void MEAPI McdGeometryInstanceReset(McdGeometryInstanceID ins)
{
    *ins = McdGeometryInstance{};
}

McdGeometryID MEAPI McdGeometryInstanceGetGeometry(McdGeometryInstanceID ins)
{
    return ins->mGeometry;
}

void MEAPI McdGeometryInstanceSetGeometry(McdGeometryInstanceID ins, McdGeometryID g)
{
    ins->mGeometry = g;
}

void MEAPI McdGeometryInstanceSetTransformPtr(const McdGeometryInstanceID ins, const MeMatrix4Ptr geometryTM)
{
    ins->mTM = geometryTM;
}

MeMatrix4Ptr MEAPI McdGeometryInstanceGetTransformPtr(McdGeometryInstanceID ins)
{
    return ins->mTM;
}

void MEAPI McdGeometryInstanceGetBSphere(McdGeometryInstanceID ins, MeVector3 center, MeReal* radius)
{
    MeVector3 local = {0, 0, 0};
    *radius = 0;
    McdGeometryGetBSphere(ins->mGeometry, local, radius);
    fromBt(toBtTransform(ins->mTM) * toBt(local), center);
}

void MEAPI McdGeometryInstanceUpdateAABB(McdGeometryInstanceID ins, const MeMatrix4Ptr finalTM, MeBool tight)
{
    const McdGeometryVTable* v = mcdVTable(ins->mGeometry);
    if (v && v->getAABB)
    {
        v->getAABB(ins, finalTM, tight);
        return;
    }
    btVector3 lo, hi;
    mcdGeometryShape(ins->mGeometry)->getAabb(toBtTransform(finalTM), lo, hi);
    fromBt(lo, ins->min);
    fromBt(hi, ins->max);
}

void MEAPI McdGeometryInstanceMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    const McdGeometryVTable* v = mcdVTable(ins->mGeometry);
    if (v && v->maximumPoint)
        v->maximumPoint(ins, inDir, outPoint);
}

void MEAPI McdGeometryInstanceGetAABB(McdGeometryInstanceID ins, MeVector3 minCorner, MeVector3 maxCorner)
{
    std::memcpy(minCorner, ins->min, sizeof(MeVector3));
    std::memcpy(maxCorner, ins->max, sizeof(MeVector3));
}

void MEAPI McdGeometryInstanceSetMaterial(McdGeometryInstanceID ins, int material)
{
    ins->mMaterial = material;
}

unsigned int MEAPI McdGeometryInstanceGetMaterial(McdGeometryInstanceID ins)
{
    return (unsigned int)ins->mMaterial;
}

McdGeometryInstanceID MEAPI McdGeometryInstanceGetChild(McdGeometryInstanceID ins, int i)
{
    McdGeometryInstance* c = ins->child;
    while (c && i-- > 0)
        c = c->next;
    return c;
}

MeBool MEAPI McdGeometryInstanceOverlap(McdGeometryInstanceID ins1, McdGeometryInstanceID ins2)
{
    for (int k = 0; k < 3; ++k)
        if (ins1->max[k] < ins2->min[k] || ins2->max[k] < ins1->min[k])
            return 0;
    return 1;
}

McdGeometryType MEAPI McdGeometryInstanceGetGeometryType(McdGeometryInstanceID ins)
{
    return (McdGeometryType)McdGeometryGetTypeId(ins->mGeometry);
}

} // extern "C"
